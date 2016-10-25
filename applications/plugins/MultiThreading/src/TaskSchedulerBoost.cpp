/*                               nulstein @ Evoke 2009
*
*
* ____________________________________
* Copyright 2009 Intel Corporation
* All Rights Reserved
*
* Permission is granted to use, copy, distribute and prepare derivative works of this
* software for any purpose and without fee, provided, that the above copyright notice
* and this statement appear in all copies.  Intel makes no representations about the
* suitability of this software for any purpose.  THIS SOFTWARE IS PROVIDED "AS IS."
* INTEL SPECIFICALLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, AND ALL LIABILITY,
* INCLUDING CONSEQUENTIAL AND OTHER INDIRECT DAMAGES, FOR THE USE OF THIS SOFTWARE,
* INCLUDING LIABILITY FOR INFRINGEMENT OF ANY PROPRIETARY RIGHTS, AND INCLUDING THE
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  Intel does not
* assume any responsibility for any errors which may appear in this software nor any
* responsibility to update it.
* ____________________________________
*
*
* A multicore tasking engine in some 500 lines of C
* This is the code corresponding to the seminar on writing a task-scheduler suitable 
* for use in multicore optimisation of small prods by Jerome Muffat-Meridol.
*
* Credits :
* -=-=-=-=-
*  .music taken from M40-Southbound, by Ghaal (c)2009
*  .liposuction advice from Matt Pietrek
*     http://www.microsoft.com/msj/archive/S572.aspx
*  .ordering display list ideas based on Christer Ericson's article 
*     http://realtimecollisiondetection.net/blog/?p=86
*  .Approximate Math Library by Alex Klimovitski, Intel GmbH
*  .kkrunchy packed this exe, kudos to ryg/farbrausch
*     http://www.farbrausch.de/~fg/kkrunchy/
*/

#include "TaskSchedulerBoost.h"
#include <sofa/helper/system/thread/CTime.h>


namespace sofa
{

namespace simulation
{

boost::thread_specific_ptr<WorkerThread> TaskScheduler::mWorkerThreadTLS;

TaskScheduler::TaskScheduler()
:mRootTaskStatus(NULL)
,mThreadCount(0)
,mMainThreadIndex(0)
,mIsInitialized(false)
,mIsClosing(false)
,mHasWorkToDo(false)
{   
    //setup the main WorkerThread
    mThread[mMainThreadIndex] = new WorkerThread(this, mMainThreadIndex);
    setWorkerThreadLifeTimeThreadLocal(mThread[mMainThreadIndex]);
}

TaskScheduler::~TaskScheduler()
{
}

void TaskScheduler::setWorkerThreadLifeTimeThreadLocal(WorkerThread* worker)
{
    // the lifetime of this instance of WorkerThread is now tied to the thread that executed this method.
    // once the thread exits, it will destroy the WorkerThread instance as well.
    mWorkerThreadTLS.reset(worker);
}

WorkerThread* TaskScheduler::GetCurrentWorkerThread()
{
    return mWorkerThreadTLS.get();
}

unsigned TaskScheduler::GetHardwareThreadsCount()
{
    return boost::thread::hardware_concurrency();
}

WorkerThread* TaskScheduler::getWorkerThread(const unsigned int index) const
{
    WorkerThread* thread = 0;
    if ( index < mThreadCount ) 
    {
        thread = mThread[index];
    }
    return thread;
}

bool TaskScheduler::isMainWorkerThread(const WorkerThread* worker) const
{
    return worker->getThreadIndex() == mMainThreadIndex;
}

bool TaskScheduler::start(const unsigned int NbThread )
{
    if ( mIsInitialized ) 
    {
        stop();
    }
    
    mIsClosing      = false;
    mRootTaskStatus = NULL;
    mHasWorkToDo    = false;

    // only physical cores. no advantage from hyperthreading.
    mThreadCount = GetHardwareThreadsCount() / 2;

    if (NbThread > 0 && NbThread <= MAX_THREADS)
    {
        mThreadCount = NbThread;
    }

    // start worker threads
    for (unsigned int iThread = 1; iThread < mThreadCount; ++iThread)
    {
        mThread[iThread] = new WorkerThread(this, iThread);
        mThread[iThread]->start();
    }

    mIsInitialized = true;
    return true;
}

bool TaskScheduler::stop()
{
    notifyWorkersForClosing();

    if ( mIsInitialized ) 
    {
        // eventually everyone will exit the WorkerThread::run method and join 
        for(unsigned iThread=1; iThread<mThreadCount; ++iThread)
        {
            mThread[iThread]->join();
        }

        mIsInitialized = false;
        mHasWorkToDo   = false;
    }
    return true;
}



void TaskScheduler::notifyWorkersForWork(Task::Status* status)
{
    boost::lock_guard<boost::mutex> lock(mWakeUpMutex);
    mHasWorkToDo = true;
    if (mRootTaskStatus == NULL)
    {
        mRootTaskStatus = status;
    }
    else
    {
        std::stringstream msg;
        msg << __FUNCTION__ << ": WorkerThread(" << GetCurrentWorkerThread()->getThreadIndex() << "): scheduler has already a root TaskStatus!\n";
        std::cerr << msg.str();
    }
    // notify all the threads that have gone Idle, that there is some work to do for them now.
    mWakeUpEvent.notify_all();
}

void TaskScheduler::notifyWorkersForClosing()
{
    boost::lock_guard<boost::mutex> lock(mWakeUpMutex);
    mIsClosing = true;
    // make all Idle threads wake up so that they can see we are closing.
    // see WorkerThread::run() break conditions in the while loop. 
    mWakeUpEvent.notify_all();
}

bool TaskScheduler::goIdle()
{
    boost::lock_guard<boost::mutex> lock(mWakeUpMutex);
    mHasWorkToDo   = false;
    mRootTaskStatus = NULL;
    return true;
}

void TaskScheduler::idleWorkerUntilNotified(const WorkerThread* worker)
{
    // The callee wil wait here until the scheduler either has some more work to do or is closing.
    // see notifyWorkersForWork and notifyWorkersForClosing

    // calling this method on the main thread is a very bad idea.
    if (isMainWorkerThread(worker))
    {
        std::cerr << __FUNCTION__ << ": cannot be called by the main thread, otherwise we will hang forever" << std::endl;
        return;
    }

    boost::unique_lock<boost::mutex> lock(mWakeUpMutex);
    mWakeUpEvent.wait(lock, [this] { return mHasWorkToDo == true || mIsClosing == true;  });
}


WorkerThread::WorkerThread(TaskScheduler* const& pScheduler, int index)
:mTaskScheduler(pScheduler)
,mStealableTaskCount(0)
,mSpecificTaskCount(0)
,mCurrentStatus(NULL)
,mThreadIndex(index)
,mTaskLogEnabled(false)
{
    assert(pScheduler);
    mTaskMutex.v_ = 0L;
}


WorkerThread::~WorkerThread()
{
    if (mThread &&  mThread->joinable())
    {
        std::stringstream msg;
        msg << __FUNCTION__ << ": WorkerThread(" << mThreadIndex << ") has not joined yet!\n";
        std::cerr << msg.str();
    }
}

void WorkerThread::join()
{
    if (!mThread)
    {
        std::cerr << __FUNCTION__ << ": must not be called by the main thread" << std::endl;
    }
    else
    {
        mThread->join();
    }
}

void WorkerThread::start()
{
    mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&WorkerThread::run, this)));
}

WorkerThread* WorkerThread::GetCurrent()
{
    return TaskScheduler::GetCurrentWorkerThread();
}

void WorkerThread::run(void)
{
    // Thread Local Storage to retrieve the WorkerThread from within a task will manage the lifetime of this object, ie call
    mTaskScheduler->setWorkerThreadLifeTimeThreadLocal(this);

    // main loop 
    for(;;)
    {
        idle();

        if ( mTaskScheduler->isClosing() ) 
            break;


        while (mTaskScheduler->getRootTaskStatus())
        {
            doWork(0);
            if (mTaskScheduler->isClosing())
            {
                break;
            }
        }
    }
}

boost::thread::id WorkerThread::getId() const
{
    return mThread->get_id();
}

unsigned WorkerThread::getThreadIndex() const
{
    return mThreadIndex;
}

void WorkerThread::enableTaskLog(bool val)
{
    mTaskLogEnabled = val;
    if (!val)
    {
        mTaskLog.clear();
    }
}

void WorkerThread::clearTaskLog()
{
    mTaskLog.clear();
}

const std::vector<Task*>& WorkerThread::getTaskLog() const
{
    return mTaskLog;
}


void WorkerThread::idle()
{
    mTaskScheduler->idleWorkerUntilNotified(this);
}

void WorkerThread::doWork(Task::Status* status)
{
    //NOTE:
    //If status is NULL, then we'll work until there is nothing left to do. This
    //is normally happening only in the case of a worker's thread loop (above).

    //if it isn't NULL, then it means the caller is waiting for this particular thing
    //to complete (and will want to carry on something once it is). We will do our work
    //and steal some until the condition happens. This is normally happening when as
    //part of WorkUntilDone (below)
    

    // NOTE: This method needs to be reentrant (!)
    //A task can be spawing more tasks and may have to wait for their completion.
    //So, as part of our pTask->runTask() we can be called again, via the WorkUntilDone
    //method, below.
    //
    do
    {
        Task*		    pTask       = NULL;
        while (popTask(&pTask))
        {
            // run
            Task::Status*	pPrevStatus = mCurrentStatus;
            mCurrentStatus = pTask->getStatus();
            
            if (mTaskLogEnabled)
                mTaskLog.push_back(pTask);

            pTask->runTask(this);
            mCurrentStatus->MarkBusy(false);
            mCurrentStatus = pPrevStatus;

            // check if work we're expecting is done
            if ( status && !status->IsBusy() ) 
                return;
        }

        /* check if root work is finished */ 
        if (!mTaskScheduler->getRootTaskStatus() )
            return;

    } while (stealTasks());

#ifdef TASKSCHEDULER_DEBUG
    {
        std::stringstream msg;
        msg << __FUNCTION__ << ": WorkerThread(" << mThreadIndex << ") is leaving probably to go Idle\n";
        std::cout << msg.str();
    }
#endif 

    // Nothing left to do, for now
    return;
}


void WorkerThread::workUntilDone(Task::Status* status)
{

    while (status->IsBusy())
    {
        doWork(status);
    }
    
    if (mTaskScheduler->getRootTaskStatus() == status)
    {
        // This is the root task status. As this is finished, the scheduler can go idle.		
        // What happens next: (eventually,) each worker thread will see that there		
        // is no main task status any more and go idle until they are notified
        // that new work needs to be done or that we are closing (see WorkerThread::run)
        mTaskScheduler->goIdle();
    }
}


bool WorkerThread::popTask(Task** outTask)
{
    SpinMutexLock lock( &mTaskMutex );
    
    Task* task=NULL;
    unsigned* taskCount=NULL;
    ///< deal with specific task list first.
    if(mSpecificTaskCount > 0)
    {
        taskCount =&mSpecificTaskCount;
        task      =mSpecificTask[*taskCount-1];
    }
    else if(mStealableTaskCount > 0)
    {
        taskCount=&mStealableTaskCount;
        task     =mStealableTask[*taskCount-1];
    }

    if(task == NULL || taskCount==NULL)
    {
        // there is no work
        return false;
    }

    // pop from top of the pile
    *outTask = task;
    --*taskCount;
    return true;
}


bool WorkerThread::pushTask(Task* task, Task* taskArray[], unsigned* taskCount )
{
    // if we're single threaded return false
    if ( mTaskScheduler->getThreadCount()<2 ) 
        return false;

    {
        SpinMutexLock lock( &mTaskMutex );

        if (*taskCount >= Max_TasksPerThread )
            return false;
        if( task->getStatus()==NULL ) {
          return false;
        }
        task->getStatus()->MarkBusy(true);
        taskArray[*taskCount] = task;
        ++*taskCount;
    }

    if (!mTaskScheduler->getRootTaskStatus() )
    {
        mTaskScheduler->notifyWorkersForWork(task->getStatus());
    }

    return true;
}

bool WorkerThread::addStealableTask(Task* task)
{
    if (pushTask(task,mStealableTask,&mStealableTaskCount))
        return true;
    
    if (mTaskLogEnabled)
        mTaskLog.push_back(task);

    // if we can't queue it, run it 
    // we don't touch the task status since it is not necessary - ie MarkBusy method : not set, not cleared 
    task->runTask(this);

    return false;
}

bool WorkerThread::addSpecificTask(Task* task)
{
    if (pushTask(task,mSpecificTask,&mSpecificTaskCount))
        return true;
    
    if (mTaskLogEnabled)
        mTaskLog.push_back(task);

    // if we can't queue it, run it 
    // we don't touch the task status since it is not necessary - ie MarkBusy method : not set, not cleared 
    task->runTask(this);

    return false;
}

void WorkerThread::runTask(Task* task)
{
    if (mTaskLogEnabled)
        mTaskLog.push_back(task);

    task->runTask(this);
}

bool WorkerThread::giveUpSomeWork(WorkerThread* idleThread)
{
    SpinMutexLock lock;

    if ( !lock.try_lock( &mTaskMutex ) ) 
        return false;

    // anything to share ? 
    if (!mStealableTaskCount)
        return false;

    SpinMutexLock	lockIdleThread;

    if ( !lockIdleThread.try_lock( &idleThread->mTaskMutex ) )
        return false; 

    if ( idleThread->mStealableTaskCount )
        return false;

    // grab half the remaining tasks (rounding up) 
    unsigned int count = (mStealableTaskCount+1) /2;

    Task** p = idleThread->mStealableTask;

    unsigned int iTask;
    for( iTask=0; iTask< count; ++iTask)
    {
        *p++ = mStealableTask[iTask];
        mStealableTask[iTask] = NULL;
    }
    idleThread->mStealableTaskCount = count;

    // move remaning task down
    for( p = mStealableTask; iTask<mStealableTaskCount; ++iTask)
    {
        *p++ = mStealableTask[iTask];
    }
    mStealableTaskCount -= count;

    return true;
}


bool WorkerThread::stealTasks()
{

    // avoid always starting with same other thread. This aims at avoiding a potential	
    // for problematic patterns. Note: the necessity of doing is largely speculative.
    // Offset = (GetWorkerIndex() + GetTickCount()) % m_pTaskPool->m_ThreadCount;

    for( unsigned int iThread=0; iThread<mTaskScheduler->getThreadCount(); ++iThread)
    {
        //WorkerThread*	pThread;

        WorkerThread* pThread = mTaskScheduler->getWorkerThread( (iThread /* + Offset */)% mTaskScheduler->getThreadCount() );
        if ( pThread == this) 
            continue;

        if ( pThread->giveUpSomeWork(this) ) 
            return true;

        if ( mStealableTaskCount ) 
            return true;
    }

    return false;
}

} // namespace simulation

} // namespace sofa
