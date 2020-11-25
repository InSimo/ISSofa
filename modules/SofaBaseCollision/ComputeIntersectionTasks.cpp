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
