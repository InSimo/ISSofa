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
