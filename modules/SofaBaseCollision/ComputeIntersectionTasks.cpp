/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include "ComputeIntersectionTasks.h"

#include <sofa/simulation/common/TaskScheduler.h>
#ifndef ISSOFA_VERSION
#include <SofaBase/config.h>
#else
#include <sofa/SofaBase.h>
#endif // !ISSOFA_VERSION



namespace sofa
{
namespace collision
{


ComputeIntersectionTasks::ComputeIntersectionTasks()
    : m_outputThreadIndex(0)
    , m_outputBeginIndex(0)
    , m_outputEndIndex(0)
{
}

void ComputeIntersectionTasks::enable(
        const sofa::simulation::Task::Status* pStatus,
        const ComputeIntersectionTaskInfo& taskInfo)
{
    sofa::simulation::Task::enable(pStatus);
    m_taskInfo = taskInfo;

    sofa::core::CollisionModel* firstModel  = m_taskInfo.elems[0].first.model;
    sofa::core::CollisionModel* secondModel = m_taskInfo.elems[0].second.model;
    m_name = firstModel->getName() + "-" + secondModel->getName() + "_" + std::to_string(m_taskInfo.index);
}

bool ComputeIntersectionTasks::run (sofa::simulation::WorkerThread* thread)
{
    int threadIndex = thread->getThreadIndex();
    sofa::core::collision::ElementIntersector*  currentIntersector  = m_taskInfo.intersector;
    DetectionOutputContainer*  currentOutput  = m_taskInfo.outputPerThread[threadIndex];

    m_outputThreadIndex = threadIndex;
    m_outputBeginIndex = currentOutput->size();
    for (const auto& elemsPair : m_taskInfo.elems)
    {
        currentIntersector->intersect(elemsPair.first, elemsPair.second, currentOutput);
    }
    m_outputEndIndex = currentOutput->size();
    
    return true;
}


} // namespace base
} // namespace isphysics
