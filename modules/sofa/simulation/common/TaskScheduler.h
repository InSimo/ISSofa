/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 MGH, INRIA, USTL, UJF, CNRS, InSimo                *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
/**********                   ORIGINAL CODE FROM:                   **********/
/*                               nulstein Evoke 2009
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

#ifndef SOFA_SIMULATION_TASKSCHEDULER_H
#define SOFA_SIMULATION_TASKSCHEDULER_H

#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>
#include <vector>
#include <sofa/SofaSimulation.h>
namespace sofa
{

namespace simulation
{

class TaskScheduler;
class WorkerThread;
class Task;
class TaskStatus;

namespace detail
{
// detail::createThreadLocalWorkerThreadInstance is the "factory" method for WorkerThreads.
// on top of constructing a WorkerThread object, its purpose is to guarantee that each execution thread 
// can have at most one WorkerThread instance attached to it.
WorkerThread* createThreadLocalWorkerThreadInstance(TaskScheduler* scheduler, unsigned index);

// the main method for threads created by the TaskScheduler.
void run(TaskScheduler* scheduler, unsigned index);

}

class SOFA_SIMULATION_COMMON_API WorkerThread
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

    void workUntilDone(const TaskStatus* status);
        
    unsigned getThreadIndex() const;
    
    void enableTaskLog(bool val);

    void clearTaskLog();
    
    const std::vector<Task*>& getTaskLog() const;

    const TaskStatus* getCurrentStatus() const { return mCurrentStatuses.empty() ? nullptr : mCurrentStatuses.back(); }
    const Task* getCurrentTask() const { return mCurrentTasks.empty() ? nullptr : mCurrentTasks.back(); }

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

    void doWork(const TaskStatus* status);

    void idle();

    friend void detail::run(TaskScheduler*, unsigned);

    friend WorkerThread* detail::createThreadLocalWorkerThreadInstance(TaskScheduler* scheduler, unsigned index);

private:

    enum 
    {
        Max_TasksPerThread = 1024
    };

    std::mutex                        mTaskMutex;
    TaskScheduler*                    mTaskScheduler; 
    Task*		                      mStealableTask[Max_TasksPerThread];///< shared task list, stealable by other threads
    Task*                             mSpecificTask[Max_TasksPerThread];///< thread specific task list, not stealable. They have a higher priority compared to the shared task list
    unsigned			              mStealableTaskCount;///< current size of the shared task list
    unsigned                          mSpecificTaskCount;///< current size of the specific task list								
    std::vector<const TaskStatus*>    mCurrentStatuses;///< current statuses stack (grows each time a task is run within another)
    std::vector<const Task*>          mCurrentTasks;///< current task stack (grows each time a task is run within another)
    unsigned                          mThreadIndex;
    bool                              mTaskLogEnabled;
    std::vector<Task*>                mTaskLog;
};

class SOFA_SIMULATION_COMMON_API TaskScheduler
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
    
    void notifyWorkersForWork(TaskStatus* status);

    void notifyWorkersForClosing();

    bool goIdle();

    void idleWorkerUntilNotified(const WorkerThread* worker);

    void setWorkerThread(WorkerThread* worker, unsigned index);

    WorkerThread* getWorkerThread(const unsigned int index) const;

    bool isMainWorkerThread(const WorkerThread* worker) const;

    TaskStatus* getRootTaskStatus() const { return mRootTaskStatus; }

    bool isClosing() const { return mIsClosing; }

    unsigned int getThreadCount(void) const { return mThreadCount; }

private:

    TaskScheduler(const TaskScheduler&) {}

    void notifyWorkerThreadCreated(WorkerThread*);

    void waitUntilWorkerThreadsAreCreated();

    WorkerThread*               mWorker[MAX_THREADS];
    std::thread                 mThread[MAX_THREADS];

    TaskStatus*	                mRootTaskStatus; // should only be set from main worker thread
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


#endif // SOFA_SIMULATION_TASKSCHEDULER_H
