/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
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
