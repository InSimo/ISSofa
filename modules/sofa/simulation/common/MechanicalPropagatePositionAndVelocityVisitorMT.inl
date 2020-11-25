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
#ifndef SOFA_SIMULATION_MECHANICALPROPAGATEPOSITIONANDVELOCITYVISITORMT_INL
#define SOFA_SIMULATION_MECHANICALPROPAGATEPOSITIONANDVELOCITYVISITORMT_INL

#include "MechanicalPropagatePositionAndVelocityVisitorMT.h"
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


template<typename TTask>
TTask* findOrCreate(sofa::core::BaseMapping* mapping,
                    typename TaskTraits<TTask>::TaskContainer& tasks,
                    const std::string& taskBaseName)
{
    TTask* task = NULL;
    
    using TaskTraits = TaskTraits<TTask>;
    typename TaskTraits::TaskContainer::iterator itFind = tasks.find(mapping->getExecUID());
    if (itFind != tasks.end()) {
        task = itFind->second;
    }
    else
    {
        task = TaskTraits::create(mapping, taskBaseName);
        tasks.insert(std::make_pair(mapping->getExecUID(), task));
    }

    return task;
}

template<typename TTask>
TMechanicalPropagatePositionAndVelocityVisitorMT<TTask>::TMechanicalPropagatePositionAndVelocityVisitorMT(
    const VisitorInfo& visitorInfo,
    MapPropagationTask& propagationTasks,
    TaskDependencyGraph& taskDependencyGraph,
    sofa::simulation::Task::Status& taskStatus,
    std::string taskBaseName
)
    : BaseMechanicalVisitor(visitorInfo.pExecParams)
    , m_visitorInfo(visitorInfo)
    , m_propagationTasks(propagationTasks)
    , m_taskDependencyGraph(taskDependencyGraph)
    , m_taskStatus(taskStatus)
    , m_taskBaseName(taskBaseName)
{
}

template<typename TTask>
sofa::simulation::Visitor::Result TMechanicalPropagatePositionAndVelocityVisitorMT<TTask>::fwdMechanicalMapping(sofa::simulation::Node* /*node*/, sofa::core::BaseMapping* mapping)
{        
    SOFA_ASSERT(mapping);

    const sofa::core::VecId positionId = m_visitorInfo.positionId;
    if(!positionId.isNull())
    {
        const bool isRestPosition = (positionId == sofa::core::VecCoordId::restPosition());
        if (!isRestPosition || (isRestPosition && mapping->applyRestPosition()))
        {
            TTask* pTask = findOrCreate<TTask>(mapping, m_propagationTasks, m_taskBaseName);
            pTask->enable(&m_taskStatus, m_visitorInfo);
            m_taskDependencyGraph.addTask(pTask, mapping->getFrom(), mapping->getTo());
        }
    }

    return Visitor::RESULT_CONTINUE;
}

} // namespace simulation
} // namespace sofa

#endif 

