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
