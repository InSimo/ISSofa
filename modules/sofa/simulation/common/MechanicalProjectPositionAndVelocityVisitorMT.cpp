/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#include "MechanicalProjectPositionAndVelocityVisitorMT.h"

#include <sofa/core/BaseState.h>
#include <sofa/core/ExecParams.h>
#ifndef ISSOFA_VERSION 
#include <sofa/simulation/Node.h>
#else
#include <sofa/simulation/common/Node.h>
#endif // !ISSOFA_VERSION


#include "TaskDependencyGraph.inl"

#include <utility>

namespace sofa
{
namespace simulation
{

MechanicalProjectPositionAndVelocityVisitorMT::MechanicalProjectPositionAndVelocityVisitorMT(
    const VisitorInfo& visitorInfo,
    MapProjectTask& projectTasks,
    TaskDependencyGraph& taskDependencyGraph,
    sofa::simulation::Task::Status& taskStatus
)
    : BaseMechanicalVisitor(visitorInfo.pExecParams)
    , m_visitorInfo(visitorInfo)
    , m_projectTasks(projectTasks)
    , m_taskDependencyGraph(taskDependencyGraph)
    , m_taskStatus(taskStatus)
{
}

namespace
{

template<typename TTaskCreator>
ProjectiveConstraintTask* findOrCreate(sofa::core::behavior::BaseProjectiveConstraintSet* projectiveConstraintSet,
                                   MechanicalProjectPositionAndVelocityVisitorMT::MapProjectTask& projectTasks)
{
    ProjectiveConstraintTask* task = NULL;
    
    MechanicalProjectPositionAndVelocityVisitorMT::MapProjectTask::iterator itFind = projectTasks.find(projectiveConstraintSet->getExecUID());
    if (itFind != projectTasks.end()) {
        task = itFind->second;
    }
    else
    {
        task = TTaskCreator()(projectiveConstraintSet);
        projectTasks.insert(std::make_pair(projectiveConstraintSet->getExecUID(), task));
    }

    return task;
}

struct ProjectConstraintTaskCreator
{
    ProjectiveConstraintTask* operator () (sofa::core::behavior::BaseProjectiveConstraintSet* projectiveConstraintSet)
    {
        return new ProjectiveConstraintTask(projectiveConstraintSet);
    }
};

} // namespace


sofa::simulation::Visitor::Result MechanicalProjectPositionAndVelocityVisitorMT::fwdProjectiveConstraintSet(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseProjectiveConstraintSet* projectiveConstraintSet)
{        
    if (projectiveConstraintSet )
    {
        ProjectiveConstraintTask* task = findOrCreate<ProjectConstraintTaskCreator>(projectiveConstraintSet, m_projectTasks);

        task->enable(&m_taskStatus, m_visitorInfo);

        m_taskDependencyGraph.addTask(task, projectiveConstraintSet->getModels(), projectiveConstraintSet->getModels());            
    }

    return Visitor::RESULT_CONTINUE;
}

sofa::simulation::Visitor::Result MechanicalProjectPositionAndVelocityVisitorMT::fwdMechanicalMapping(sofa::simulation::Node* /*node*/, sofa::core::BaseMapping* /*mapping*/)
{        
    return Visitor::RESULT_PRUNE;
}



} // namespace simulation
} // namespace sofa
