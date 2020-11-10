/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#include "MechanicalAccumulateMatrixDerivMT.h"

#include "AccumulateTasks.h"
#include "TaskDependencyGraph.inl"

namespace isphysics
{
namespace base
{

namespace
{

AccumulateTask* findOrCreate(sofa::core::BaseMapping* mapping,
                             MechanicalAccumulateMatrixDerivMT::MapAccumulateTasks& accumulateTasks)
{
    AccumulateTask* task = NULL;

    MechanicalAccumulateMatrixDerivMT::MapAccumulateTasks::iterator itFind = accumulateTasks.find(mapping->getExecUID());
    if (itFind != accumulateTasks.end()) {
        task = itFind->second;
    }
    else
    {
        task = new AccumulateTask(mapping);
        accumulateTasks.insert(itFind, std::make_pair(mapping->getExecUID(), task));
    }

    return task;
}

} // namespace

void MechanicalAccumulateMatrixDerivMT::bwdMechanicalMapping(sofa::simulation::Node* /*node*/, sofa::core::BaseMapping* mapping)
{
    if (mapping)
    {
        AccumulateTask* task = findOrCreate(mapping, m_accumulateTasks);
        task->enable(&m_taskStatus, m_accumulateTaskInfo);
        m_taskDependencyGraph.addTask(task, mapping->getTo(), mapping->getFrom());
    }
}



} // namespace base
} // namespace isphysics
