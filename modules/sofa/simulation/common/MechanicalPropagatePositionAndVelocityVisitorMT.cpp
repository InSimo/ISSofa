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
#include "MechanicalPropagatePositionAndVelocityVisitorMT.inl"
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

UpdateMappingVisitorMT::UpdateMappingVisitorMT(
    const VisitorInfo& visitorInfo,
    MapPropagationTask& propagationTasks,
    TaskDependencyGraph& taskDependencyGraph,
    sofa::simulation::Task::Status& taskStatus
)
    : sofa::simulation::Visitor(visitorInfo.pExecParams)
    , m_visitorInfo(visitorInfo)
    , m_propagationTasks(propagationTasks)
    , m_taskDependencyGraph(taskDependencyGraph)
    , m_taskStatus(taskStatus)
    , m_taskBaseName("UpdateVisualMapping")
{
}

void UpdateMappingVisitorMT::processMapping(sofa::simulation::Node* /*node*/, sofa::core::BaseMapping* mapping)
{
    if (mapping)
    {
        MappingPropagateTask* pTask = findOrCreate<MappingPropagateTask>(mapping, m_propagationTasks, m_taskBaseName);
        pTask->enable(&m_taskStatus, m_visitorInfo);

        m_taskDependencyGraph.addTask(pTask, mapping->getFrom(), mapping->getTo());
    }
}

void UpdateMappingVisitorMT::processMechanicalMapping(sofa::simulation::Node* /*n*/, core::BaseMapping* /*obj*/)
{
    // mechanical mappings with isMechanical flag not set are now processed by the MechanicalPropagatePositionVisitor visitor
}

sofa::simulation::Visitor::Result UpdateMappingVisitorMT::processNodeTopDown(sofa::simulation::Node* node)
{
    for_each(this, node, node->mapping, &UpdateMappingVisitorMT::processMapping);
    for_each(this, node, node->mechanicalMapping, &UpdateMappingVisitorMT::processMechanicalMapping);

    return RESULT_CONTINUE;
}



} // namespace simulation
} // namespace sofa
