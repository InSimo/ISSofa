/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_MECHANICALPROPAGATEPOSITIONANDVELOCITYVISITORMT_INL
#define ISPHYSICS_BASE_MECHANICALPROPAGATEPOSITIONANDVELOCITYVISITORMT_INL

#include "MechanicalPropagatePositionAndVelocityVisitorMT.h"
#ifndef ISSOFA_VERSION 
#include <sofa/simulation/Node.h>
#else
#include <sofa/simulation/common/Node.h>
#endif // !ISSOFA_VERSION

#include "TaskDependencyGraph.inl"

#include <utility>

ISPHYSICS_INTERNAL

namespace isphysics
{
namespace base
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
    ISASSERT(mapping);

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

} // namespace base
} // namespace isphysics

#endif 

