/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_SIMULATION_TASKDEPENDENCYGRAPH_H
#define SOFA_SIMULATION_TASKDEPENDENCYGRAPH_H

#include "TaskScheduler.h"
#include "DependencyTask.h"
#include <unordered_map>
#include <sofa/helper/vector.h>
#include <iostream>



namespace sofa
{
namespace simulation
{

template< typename TTask>
struct TaskDependencyTraits
{
    typedef void        TaskInputData;
    typedef void        TaskOutputData;
    typedef const void* TaskOutputID;
    static TaskOutputID getID(const TaskOutputData* o)
    {
        return o;
    }
};

template< typename TTaskDependencyTraits >
class TaskDependencyGraph
{
public:
    typedef DependencyTask                                  Task;
    typedef TTaskDependencyTraits                           TaskTraits;

    typedef typename TaskTraits::TaskInputData              TaskInputData;
    typedef typename TaskTraits::TaskOutputData             TaskOutputData;
    typedef typename TaskTraits::TaskOutputID               TaskOutputID;

    typedef sofa::helper::vector<Task *>                    TaskContainer;
    typedef std::unordered_map<TaskOutputID, Task *>        TaskWrite;
    //typedef std::unordered_map< TaskInputData *, std::vector<Task *> > TaskRead;

    ~TaskDependencyGraph();
        
    void clear();
    
    template<class C, typename T>
    using enable_if_contains = typename std::enable_if<std::is_same<typename C::value_type, T>::value>::type;
    template<class C>
    using enable_if_contains_TaskInputData = enable_if_contains<C, TaskInputData*>;
    template<class C>
    using enable_if_contains_TaskOutputData = enable_if_contains<C, TaskOutputData*>;
    
    // Should have used this instead but MSVC didn't like it :/
    /*template<typename C>
    using enable_if_has_begin_end = typename std::enable_if<
        std::is_same<decltype(static_cast<typename C::const_iterator(C::*)() const>(&C::begin)), typename C::const_iterator(C::*)() const>::value
        && std::is_same<decltype(static_cast<typename C::const_iterator(C::*)() const>(&C::end)), typename C::const_iterator(C::*)() const>::value
        >::type;*/
    
    template< typename TTaskInputContainer, typename TTaskOutputContainer>
    void    addTask(Task *pTask, const TTaskInputContainer& taskInputs, const TTaskOutputContainer& taskOuputs, enable_if_contains_TaskInputData <TTaskInputContainer>* = 0, enable_if_contains_TaskOutputData <TTaskOutputContainer>* = 0);
    template< typename TTaskOutputContainer>
    void    addTask(Task *pTask, TaskInputData* taskInput, const TTaskOutputContainer& taskOutputs, enable_if_contains_TaskOutputData <TTaskOutputContainer>* = 0);
    template< typename TTaskInputContainer>
    void    addTask(Task *pTask, const TTaskInputContainer& taskInputs, TaskOutputData* taskOuput, enable_if_contains_TaskInputData <TTaskInputContainer>* = 0);
    void    addTask(Task *pTask, TaskInputData* taskInput, TaskOutputData* taskOutput);
    
    void    computeGraph();
    
    void    runTasks(WorkerThread& thread, sofa::simulation::Task::Status *pTaskStatus) const;
    void    runTasksSequential(WorkerThread& thread, sofa::simulation::Task::Status *) const;

    void    dumpGraph( std::ostream& out = std::cout ) const;

private:
    TaskContainer                                           m_taskContainer;

    TaskWrite                                               m_taskWrite;
    //TaskRead                                                m_taskRead;
};
    
} // namespace simulation
} // namespace sofa

#endif
