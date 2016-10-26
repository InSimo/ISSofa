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

#ifndef TaskSchedulerBoost_h__
#define TaskSchedulerBoost_h__

#include <MultiThreading/config.h>
#include "Tasks.h"

#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>

namespace sofa
{

namespace simulation
{

class TaskScheduler;
class WorkerThread;


namespace detail
{
// detail::createThreadLocalWorkerThreadInstance is the "factory" method for WorkerThreads.
// on top of constructing a WorkerThread object, its purpose is to guarantee that each execution thread 
// can have at most one WorkerThread instance attached to it.
WorkerThread* createThreadLocalWorkerThreadInstance(TaskScheduler* scheduler, unsigned index);

// the main method for threads created by the TaskScheduler.
void run(TaskScheduler* scheduler, unsigned index);

}

class SOFA_MULTITHREADING_PLUGIN_API WorkerThread
{
public:
    ~WorkerThread();
    
    static WorkerThread* GetCurrent();
    
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
    // detail::createThreadLocalWorkerThreadInstance is the "factory" method for WorkerThreads.
    WorkerThread(TaskScheduler* const& taskScheduler, int index);
    
    // non copy-able
    WorkerThread(const WorkerThread&) {}
    
    // queue task if there is space (or do nothing)
    bool pushTask(Task* pTask, Task* taskArray[], unsigned* taskCount );

    // pop task from queue
    bool popTask(Task** ppTask);

    // steal and queue some task from another thread 
    bool stealTasks();

    // give an idle thread some work
    bool giveUpSomeWork(WorkerThread* pIdleThread);	 

    void doWork(Task::Status* status);		

    void idle();

    friend void detail::run(TaskScheduler*, unsigned);

    friend WorkerThread* detail::createThreadLocalWorkerThreadInstance(TaskScheduler* scheduler, unsigned index);

private:

    enum 
    {
        Max_TasksPerThread = 256
    };

    std::mutex                        mTaskMutex;
    TaskScheduler*                    mTaskScheduler; 
    Task*		                      mStealableTask[Max_TasksPerThread];///< shared task list, stealable by other threads
    Task*                             mSpecificTask[Max_TasksPerThread];///< thread specific task list, not stealable. They have a higher priority compared to the shared task list
    unsigned			              mStealableTaskCount;///< current size of the shared task list
    unsigned                          mSpecificTaskCount;///< current size of the specific task list								
    Task::Status*	                  mCurrentStatus;	
    unsigned                          mThreadIndex;
    bool                              mTaskLogEnabled;
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
        
    bool start(const unsigned int NbThread = 0);

    bool stop();
    
    void notifyWorkersForWork(Task::Status* status);

    void notifyWorkersForClosing();

    bool goIdle();

    void idleWorkerUntilNotified(const WorkerThread* worker);

    void setWorkerThread(WorkerThread* worker, unsigned index);

    WorkerThread* getWorkerThread(const unsigned int index) const;

    bool isMainWorkerThread(const WorkerThread* worker) const;

    Task::Status* getRootTaskStatus() const { return mRootTaskStatus; }

    bool isClosing() const { return mIsClosing; }

    unsigned int getThreadCount(void) const { return mThreadCount; }

private:

    TaskScheduler(const TaskScheduler&) {}

    void notifyWorkerThreadCreated(WorkerThread*);

    void waitUntilWorkerThreadsAreCreated();

    WorkerThread*               mWorker[MAX_THREADS];
    std::thread                 mThread[MAX_THREADS];

    Task::Status*	            mRootTaskStatus;
    unsigned                    mThreadCount;
    unsigned                    mMainThreadIndex;
    unsigned                    mWorkerThreadCreateCount; // guarded by startMutex
    bool                        mIsInitialized;
    bool                        mIsClosing; // guarded by wakeUpMutex
    bool                        mHasWorkToDo; // guarded by wakeUpMutex
    
    std::mutex                  mWorkerThreadCreateCountMutex;
    std::condition_variable     mWorkerThreadCreateEvent;

    std::mutex                  mWakeUpMutex;
    std::condition_variable     mWakeUpEvent;
};



} // namespace simulation

} // namespace sofa


#endif // TaskSchedulerBoost_h__
