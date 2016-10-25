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
* �A multicore tasking engine in some 500 lines of C�
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

#ifndef TaskSchedulerBoost_h__
#define TaskSchedulerBoost_h__

#include "initMultiThreading.h"
#include "Tasks.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/tss.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

namespace sofa
{

namespace simulation
{

class TaskScheduler;
class WorkerThread;

class SpinMutexLock
{
public:

    SpinMutexLock() : mMutex(0)
    {
    }

    SpinMutexLock(boost::detail::spinlock* pMutex, bool bLock = true)
        : mMutex(pMutex)
    {

        if (bLock) 
        {
            mMutex->lock();
        }
    }

    bool try_lock(boost::detail::spinlock* pMutex)
    {
        if (!pMutex->try_lock()) 
        {
            return false;
        }

        mMutex = pMutex;
        return true;
    }

    ~SpinMutexLock()
    {
        if (mMutex) 
            mMutex->unlock();
    }

private:
    boost::detail::spinlock* mMutex;
};

class SOFA_MULTITHREADING_PLUGIN_API WorkerThread
{
public:

    WorkerThread(TaskScheduler* const& taskScheduler, int index);

    ~WorkerThread();

    static WorkerThread* GetCurrent();

    void start();

    void join();

    /// queue task if there is space, and run it otherwise
    bool addStealableTask(Task* pTask);

    /// queue a task to the specific task list. It cannot be stealed, and therefore be executed only by this thread. 
    bool addSpecificTask(Task* pTask);
    
    /// run the given task directly
    void runTask(Task* pTask);

    void workUntilDone(Task::Status* status);
        
    unsigned getThreadIndex() const;
    
    void enableTaskLog(bool val);

    void clearTaskLog();
    
    const std::vector<Task*>& getTaskLog() const;

    Task::Status* getCurrentStatus() const { return mCurrentStatus; }

    const TaskScheduler* getTaskScheduler() { return mTaskScheduler; }

private:
    // non copy-able
    WorkerThread(const WorkerThread&) {}

    boost::thread::id getId() const;

    // queue task if there is space (or do nothing)
    bool pushTask(Task* pTask, Task* taskArray[], unsigned* taskCount );

    // pop task from queue
    bool popTask(Task** ppTask);

    // steal and queue some task from another thread 
    bool stealTasks();

    // give an idle thread some work
    bool giveUpSomeWork(WorkerThread* pIdleThread);	 

    void doWork(Task::Status* status);		

    void run();

    void idle();

private:

    enum 
    {
        Max_TasksPerThread = 256
    };

    boost::detail::spinlock	          mTaskMutex;
    TaskScheduler*                    mTaskScheduler; 
    Task*		                      mStealableTask[Max_TasksPerThread];///< shared task list, stealable by other threads
    Task*                             mSpecificTask[Max_TasksPerThread];///< thread specific task list, not stealable. They have a higher priority compared to the shared task list
    unsigned			              mStealableTaskCount;///< current size of the shared task list
    unsigned                          mSpecificTaskCount;///< current size of the specific task list								
    Task::Status*	                  mCurrentStatus;	
    unsigned                          mThreadIndex;
    bool                              mTaskLogEnabled;
    boost::shared_ptr<boost::thread>  mThread;
    std::vector<Task*>                mTaskLog;
};

class SOFA_MULTITHREADING_PLUGIN_API TaskScheduler
{
    enum
    {
        MAX_THREADS = 16
    };

public:

    static WorkerThread* GetCurrentWorkerThread();

    static unsigned GetHardwareThreadsCount();

    TaskScheduler();

    ~TaskScheduler();

    void setWorkerThreadLifeTimeThreadLocal(WorkerThread* worker);
    
    bool start(const unsigned int NbThread = 0);

    bool stop();

    void notifyWorkersForWork(Task::Status* status);

    void notifyWorkersForClosing();

    bool goIdle();

    void idleWorkerUntilNotified(const WorkerThread* worker);

    WorkerThread* getWorkerThread(const unsigned int index) const;

    bool isMainWorkerThread(const WorkerThread* worker) const;

    volatile Task::Status* getRootTaskStatus() const { return mRootTaskStatus; }

    bool isClosing() const { return mIsClosing; }

    unsigned int getThreadCount(void) const { return mThreadCount; }

private:

    TaskScheduler(const TaskScheduler&) {}

    static boost::thread_specific_ptr<WorkerThread>	mWorkerThreadTLS;

    WorkerThread*               mThread[MAX_THREADS];

    // The following members may be accessed by _multiple_ threads at the same time:
    volatile Task::Status*	    mRootTaskStatus;

    unsigned                    mThreadCount;
    unsigned                    mMainThreadIndex;
    bool                        mIsInitialized;
    bool                        mIsClosing; // guarded by wakeUpMutex
    bool                        mHasWorkToDo; // guarded by wakeUpMutex
    boost::mutex                mWakeUpMutex;
    boost::condition_variable   mWakeUpEvent;

};		

} // namespace simulation

} // namespace sofa


#endif // TaskSchedulerBoost_h__
