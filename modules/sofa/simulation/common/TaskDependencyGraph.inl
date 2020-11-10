/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_TASKDEPENDENCYGRAPH_INL
#define ISPHYSICS_BASE_TASKDEPENDENCYGRAPH_INL

#include "TaskDependencyGraph.h"

#include <MultiThreading/src/Tasks.h>
#include <MultiThreading/src/TaskSchedulerBoost.h>

ISPHYSICS_PUBLIC

namespace sofa
{
namespace simulation
{

template<typename TTaskDependencyTraits>
TaskDependencyGraph<TTaskDependencyTraits>::~TaskDependencyGraph()
{}

template<typename TTaskDependencyTraits>
void TaskDependencyGraph<TTaskDependencyTraits>::clear()
{
    m_taskContainer.clear();
    m_taskWrite.clear();
    //m_taskRead.clear();
}

namespace
{

template<typename TTaskDependencyTraits, typename TWriteTasks>
struct RegisterWriteTask : public std::binary_function<typename TTaskDependencyTraits::TaskOutputData *, DependencyTask *, void>
{
    RegisterWriteTask(TWriteTasks& a_writeTasks)
        : writeTasks(a_writeTasks)
    {}

    void operator () (typename TTaskDependencyTraits::TaskOutputData * pData, DependencyTask *pTask) const
    {
        typename TTaskDependencyTraits::TaskOutputID id = TTaskDependencyTraits::getID(pData);
        typename TWriteTasks::iterator itFind = writeTasks.find(id);
        if (itFind == writeTasks.end()) {
            writeTasks.insert(std::make_pair(id, pTask));
        }
        else
        {
            DependencyTask *pPredecessorTask = itFind->second;
            if (pPredecessorTask != pTask)
            {
                pPredecessorTask->addSuccessor(pTask);
                itFind->second = pTask;
            }
        }
    }

    TWriteTasks& writeTasks;
};

template<typename TTaskDependencyTraits, typename TWriteTasks>
struct GenerateDepencencies : public std::binary_function<typename TTaskDependencyTraits::TaskOutputData *, DependencyTask *, void>
{
    GenerateDepencencies(TWriteTasks& a_writeTask)
        : writeTasks(a_writeTask)
    {}
        
    void operator () (typename TTaskDependencyTraits::TaskOutputData * pData, DependencyTask *pTask) const
    {
        typename TTaskDependencyTraits::TaskOutputID id = TTaskDependencyTraits::getID(pData);
        typename TWriteTasks::iterator itWriteTask = writeTasks.find(id);
        if (itWriteTask != writeTasks.end())
        {
            DependencyTask *pPredecessorTask = itWriteTask->second;
            if (pPredecessorTask != pTask)
            {
                pPredecessorTask->addSuccessor(pTask);
            }
        }
    }

    TWriteTasks&   writeTasks;
};

/*
template<typename TTask, typename TReadTask>
struct FillReadTasks : public std::binary_function<typename TTask::Data *, TTask *, void>
{
    FillReadTasks(TReadTask& a_readTask)
        : readTask(a_readTask)
    {}
        
    void operator () (typename TTask::Data *pData, TTask *pTask) const
    {
        typename TReadTask::iterator itFind = readTask.find(pData);
        if (itFind == readTask.end()) {
            readTask[pData].push_back(pTask);
        } else {
            itFind->second.push_back(pTask);
        }
    }

    TReadTask&    readTask;
};
*/

} // namespace


template< typename TTaskDependencyTraits>
template< typename TTaskInputContainer, typename TTaskOutputContainer>
void TaskDependencyGraph<TTaskDependencyTraits>::addTask(Task *pTask, const TTaskInputContainer& taskInputs, const TTaskOutputContainer& taskOutputs, enable_if_contains_TaskInputData<TTaskInputContainer>*, enable_if_contains_TaskOutputData<TTaskOutputContainer>*)
{
    assert(pTask);
    {
        m_taskContainer.push_back(pTask);
    }

    {
        const RegisterWriteTask<TTaskDependencyTraits, TaskWrite> fnRegisterWriteTask(m_taskWrite);
        std::for_each(taskOutputs.begin(), taskOutputs.end(), std::bind2nd(fnRegisterWriteTask, pTask));
    }
        
    {
        const GenerateDepencencies<TTaskDependencyTraits, TaskWrite> fnGenerateDepencencies(m_taskWrite);
        std::for_each(taskInputs.begin(), taskInputs.end(), std::bind2nd(fnGenerateDepencencies, pTask));
    }
}

template< typename TTaskDependencyTraits>
template< typename TTaskOutputContainer >
void TaskDependencyGraph<TTaskDependencyTraits>::addTask(Task *pTask, TaskInputData* taskInput, const TTaskOutputContainer& taskOutputs, enable_if_contains_TaskOutputData<TTaskOutputContainer>*)
{
    assert(pTask);
    {
        m_taskContainer.push_back(pTask);
    }

    {
        const RegisterWriteTask<TTaskDependencyTraits, TaskWrite> fnRegisterWriteTask(m_taskWrite);
        std::for_each(taskOutputs.begin(), taskOutputs.end(), std::bind2nd(fnRegisterWriteTask, pTask));
    }
        
    {
        const GenerateDepencencies<TTaskDependencyTraits, TaskWrite> fnGenerateDepencencies(m_taskWrite);
        fnGenerateDepencencies(taskInput, pTask);
    }
}

template< typename TTaskDependencyTraits>
template< typename TTaskInputContainer >
void TaskDependencyGraph<TTaskDependencyTraits>::addTask(Task *pTask, const TTaskInputContainer& taskInputs, TaskOutputData* taskOutput, enable_if_contains_TaskInputData<TTaskInputContainer>*)
{
    assert(pTask);
    {
        m_taskContainer.push_back(pTask);
    }

    {
        const RegisterWriteTask<TTaskDependencyTraits, TaskWrite> fnRegisterWriteTask(m_taskWrite);
        fnRegisterWriteTask(taskOutput, pTask);
    }
        
    {
        const GenerateDepencencies<TTaskDependencyTraits, TaskWrite> fnGenerateDepencencies(m_taskWrite);
        std::for_each(taskInputs.begin(), taskInputs.end(), std::bind2nd(fnGenerateDepencencies, pTask));
    }
}

template< typename TTaskDependencyTraits>
void TaskDependencyGraph<TTaskDependencyTraits>::addTask(Task *pTask, TaskInputData* taskInput, TaskOutputData* taskOutput)
{
    assert(pTask);
    {
        m_taskContainer.push_back(pTask);
    }

    {
        const RegisterWriteTask<TTaskDependencyTraits, TaskWrite> fnRegisterWriteTask(m_taskWrite);
        fnRegisterWriteTask(taskOutput, pTask);
    }
        
    {
        const GenerateDepencencies<TTaskDependencyTraits, TaskWrite> fnGenerateDepencencies(m_taskWrite);
        fnGenerateDepencencies(taskInput, pTask);
    }
}

template<typename TTaskDependencyTraits>
void TaskDependencyGraph<TTaskDependencyTraits>::computeGraph()
{
}

namespace
{

template<typename TTask>
struct QueueIfStealable : public std::unary_function<TTask *, void>
{
    QueueIfStealable(WorkerThread& a_thread)
        : thread(a_thread)
    {}
        
    void operator () (TTask *pTask) const
    {
        if (!pTask->hasPredecessors()) {
            thread.addStealableTask(pTask);
        }
    }

    WorkerThread& thread;
};

template<typename TTask>
struct Run : public std::binary_function<TTask *, WorkerThread&, void>
{
    void operator () (TTask* task, WorkerThread& thread) const
    {
        task->runTask(&thread);
    }
};

} // namespace

template<typename TTaskDependencyTraits>
void TaskDependencyGraph<TTaskDependencyTraits>::runTasks(WorkerThread& thread, sofa::simulation::Task::Status *pTaskStatus) const
{
    std::for_each(m_taskContainer.begin(), m_taskContainer.end(), QueueIfStealable<Task>(thread));
        
    thread.workUntilDone(pTaskStatus);
}

template<typename TTaskDependencyTraits>
void TaskDependencyGraph<TTaskDependencyTraits>::runTasksSequential(WorkerThread& thread, sofa::simulation::Task::Status *) const
{
    std::for_each(m_taskContainer.begin(), m_taskContainer.end(), std::bind2nd(Run<Task>(), thread));
}

namespace
{

template<typename TTask>
struct PrintTaskInfo : public std::binary_function<TTask *, unsigned int, void>
{

    PrintTaskInfo(std::ostream& out = std::cout)
        :out(out)
    {
    }

    void operator () (const TTask *pTask, unsigned int level = 0) const
    {
        for (unsigned int i = 0; i < level; i++) {
            out << "   --> ";
        }
            
        out << pTask->getName() << " refCount: " << pTask->getRefCount() << std::endl;

        if (level == 0)
        {
            out << "   Successors:" << std::endl;

            PrintTaskInfo<DependencyTask> printTaskInfo(out);

            std::for_each(pTask->getSuccessors().begin(), pTask->getSuccessors().end(), std::bind2nd(printTaskInfo, level + 1));

            out << "   Predecessors:" << std::endl;
            std::for_each(pTask->getPredecessors().begin(), pTask->getPredecessors().end(), std::bind2nd(printTaskInfo, level + 1));
        }
    }

private:
    std::ostream& out;
};

template<typename TTaskID, typename TTask >
struct PrintTaskInfoId
{
    PrintTaskInfoId(std::ostream& out = std::cout)
        :out(out)
    {
    }

    void operator () (std::pair<const TTaskID, TTask * > pTask) const
    {
        out << "id " << pTask.first << pTask.second->getName() << " refCount: " << pTask.second->getRefCount() << std::endl;
    }

private:
    std::ostream& out;
};

} // namespace

template<typename TTaskDependencyTraits>
void TaskDependencyGraph<TTaskDependencyTraits>::dumpGraph( std::ostream& out ) const
{
    out << std::endl << "Tasks dump: begin" << std::endl;
    
    //out << std::endl << "Ids:" << std::endl;
    //PrintTaskInfoId<const TaskOutputID, Task> printTaskInfoId(out);
    //std::for_each(m_taskWrite.begin(), m_taskWrite.end(), printTaskInfoId);

    //out << std::endl << "Dependencies:" << std::endl;
    PrintTaskInfo<DependencyTask> printTaskInfo(out);
    std::for_each(m_taskContainer.begin(), m_taskContainer.end(), printTaskInfo);

    out << "Tasks dump: end" << std::endl << std::endl;
}
    
} // namespace simulation
} // namespace sofa

#endif
