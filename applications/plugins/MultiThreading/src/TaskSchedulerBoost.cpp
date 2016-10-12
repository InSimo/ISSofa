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

//#define TASKSCHEDULER_DEBUG



namespace sofa
{

namespace simulation
{

boost::thread_specific_ptr<WorkerThread> TaskScheduler::mWorkerThreadIndex;

TaskScheduler& TaskScheduler::getInstance()
{
	static TaskScheduler instance;
	return instance;
}

TaskScheduler::TaskScheduler()
{
    mIsInitialized = false;
    mThreadCount = 0;
    mIsClosing = false;

    readyForWork = false;

    //set ourselves up as thread[0]
    mThread[0] = new WorkerThread( this, 0 );
    mThread[0]->attachToThisThread( this );

}

TaskScheduler::~TaskScheduler()
{
    if ( mIsInitialized ) 
    {
        //stop();
    }
    if ( mThread[0] != 0 )
    {
        //delete mThread[0]; 
    }
}

unsigned TaskScheduler::GetHardwareThreadsCount()
{
    return boost::thread::hardware_concurrency();
}


WorkerThread* TaskScheduler::getWorkerThread(const unsigned int index) 
{
    WorkerThread* thread = 0;
    if ( index < mThreadCount ) 
    {
        thread = mThread[index];
    }
    return thread;
}

bool TaskScheduler::start(const unsigned int NbThread )
{

    if ( mIsInitialized ) 
    {
        stop();
    }

    //if ( !mIsInitialized ) 
    {
        mIsClosing		= false;
        mWorkersIdle    = false;
        mainTaskStatus	= NULL;

        // only physical cores. no advantage from hyperthreading.
        mThreadCount = GetHardwareThreadsCount() / 2;

        if ( NbThread > 0 && NbThread <= MAX_THREADS  )
        {
            mThreadCount = NbThread;
        }			


        //mThread[0] =  new WorkerThread( this ) ;
        //mThread[0]->attachToThisThread( this );

        /* start worker threads */ 
        for( unsigned int iThread=1; iThread<mThreadCount; ++iThread)
        {
            //mThread[iThread] = boost::shared_ptr<WorkerThread>(new WorkerThread(this) );
            mThread[iThread] = new WorkerThread(this, iThread);
            mThread[iThread]->create_and_attach();
            mThread[iThread]->start();
        }

        mWorkerCount = mThreadCount;
        mIsInitialized = true;
        return true;
    }
    //else
    //{
    //	return false;
    //}

}



bool TaskScheduler::stop()
{
    unsigned iThread;

    mIsClosing = true;

    if ( mIsInitialized ) 
    {
        // wait for all to finish
        WaitForWorkersToBeReady();
        wakeUpWorkers();

        for(iThread=1; iThread<mThreadCount; ++iThread)
        {
            while (!mThread[iThread]->mFinished)
            {
                //spin
            }
        }
        for(iThread=1; iThread<mThreadCount; ++iThread)
        {
            mThread[iThread] = 0;
        }


        mIsInitialized = false;
        mWorkerCount = 1;
    }


    return true;
}



void TaskScheduler::wakeUpWorkers()
{
#ifdef TASKSCHEDULER_DEBUG
    if (mWorkersIdle == false)
    {
        std::cerr << __FUNCTION__ << ": expect mWorkersIdle to be true" << std::endl;
    }
#endif

    mWorkersIdle = false;
    {
        boost::lock_guard<boost::mutex> lock(wakeUpMutex);
        readyForWork = true;
    }
    wakeUpEvent.notify_all();
}


void TaskScheduler::WaitForWorkersToBeReady()
{
#ifdef TASKSCHEDULER_DEBUG
    if (mWorkersIdle == true)
    {
        std::cerr << __FUNCTION__ << ": expect mWorkersIdle to be false" << std::endl;
    }
#endif

    // MISSING: should wait on a condition variable that would be released once each worker thread would has notified it has gone Idle.
    for(unsigned i=0; i<mThreadCount-1; ++i)
    {}

    mWorkersIdle = true;
}




unsigned TaskScheduler::size()	const volatile
{
    return mWorkerCount;
}



WorkerThread::WorkerThread(TaskScheduler* const& pScheduler, int index)
:mTaskScheduler(pScheduler)
,mStealableTaskCount(0)
,mSpecificTaskCount(0)
,mCurrentStatus(NULL)
,mThreadIndex(index)
,mTaskLogEnabled(false)
,mFinished(false)
{
    assert(pScheduler);
    mTaskMutex.v_ = 0L;
}


WorkerThread::~WorkerThread()
{
    //{
    //	release( this->mThread );
    //}
}		

bool WorkerThread::attachToThisThread(TaskScheduler* pScheduler)
{

    mStealableTaskCount		= 0;
    mFinished		= false;			

    TaskScheduler::mWorkerThreadIndex.reset( this );

    return true;
}



bool WorkerThread::start()
{
    mCurrentStatus = NULL;

    return  mThread != 0;
}




boost::shared_ptr<boost::thread> WorkerThread::create_and_attach()
{
    mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&WorkerThread::run, this)));
    return mThread;
}


bool WorkerThread::release()
{

    if ( mThread.get() != 0 )
    {
        mThread->join();

        return true;
    }

    return false;
}


WorkerThread* WorkerThread::getCurrent()
{
    return TaskScheduler::mWorkerThreadIndex.get();
}


void WorkerThread::run(void)
{

    // Thread Local Storage 
    TaskScheduler::mWorkerThreadIndex.reset( this );

    // main loop 
    for(;;)
    {
        Idle();

        if ( mTaskScheduler->isClosing() ) 
            break;


        while (mTaskScheduler->mainTaskStatus)
        {

            doWork(0);


            if (mTaskScheduler->isClosing() ) 
                break;
        }

    }

    mFinished = true;

    return;
}


boost::thread::id WorkerThread::getId()
{
    return mThread->get_id();
}

int WorkerThread::getThreadIndex()
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

const std::vector<Task*>& WorkerThread::getTaskLog()
{
    return mTaskLog;
}


void WorkerThread::Idle()
{
    // CHANGE: original intel code was also advertising the scheduler this thread was going to sleep, 
    //         one of the consequences is that the mWorkersIdle member variable is never set to true.



    // Sleep until there is work
    boost::unique_lock<boost::mutex> lock( mTaskScheduler->wakeUpMutex );

    while(!mTaskScheduler->readyForWork)
    {
        mTaskScheduler->wakeUpEvent.wait(lock);
    }

    return;
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
        Task::Status*	pPrevStatus = NULL;

        while (popTask(&pTask))
        {
            // run
            pPrevStatus = mCurrentStatus;
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

        /* check if main work is finished */ 
        if (!mTaskScheduler->mainTaskStatus) 
            return;

    } while (stealTasks());	

    // Nothing left to do, for now
    return;
}


void WorkerThread::workUntilDone(Task::Status* status)
{

    while (status->IsBusy())
    {
        doWork(status);
    }
    
    if (mTaskScheduler->mainTaskStatus == status)
    {	
        // This is the root task status. As this is finished, the scheduler can go idle.		
        // What happens next: (eventually,) each worker thread will see that there		
        // is no main task status any more and go idle waiting for semaphore to signal	
        // that new work nees to be done (see WorkerThread::run)				
        mTaskScheduler->mainTaskStatus = NULL;

        // CHANGE: these two lines are not part of the original intel code, where there is no attempt whatsoever to acquire a wakeUpMutex
        boost::lock_guard<boost::mutex> lock(mTaskScheduler->wakeUpMutex);
        mTaskScheduler->readyForWork = false;    
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
    // MISSING: this was part of the original code 
    // if (!mTaskScheduler->mainTaskStatus)
    // {
    //    mTaskScheduler->WaitForWorkersToBeReady();
    // }

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

    if (!mTaskScheduler->mainTaskStatus)
    {
        // Mark this task status as the root task status ( see WorkUntilDone )
        mTaskScheduler->mainTaskStatus = task->getStatus();
        // notify all the threads that have gone Idle, that there is some work to do for them now.
        mTaskScheduler->wakeUpWorkers();
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

        WorkerThread* pThread = mTaskScheduler->mThread[ (iThread /* + Offset */)% mTaskScheduler->getThreadCount() ];
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
