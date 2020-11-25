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
#include "FilteredBruteForceDetectionMT.h"
#include <sofa/simulation/common/TaskScheduler.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/assert.h>

namespace sofa
{
namespace collision
{


SOFA_DECL_CLASS(FilteredBruteForceDetectionMT)

int FilteredBruteForceDetectionMTClass = 
    sofa::core::RegisterObject("BruteForceDetection with multithreaded narrow phase.").add< FilteredBruteForceDetectionMT>();

FilteredBruteForceDetectionMT::FilteredBruteForceDetectionMT()
: d_nbIntersectionTestsPerSubTask(initData(&d_nbIntersectionTestsPerSubTask, 0u, "nbIntersectionTestsPerSubTask", "Number of intersection tests to give to each narrow phase sub task. 0 means no subtask is created"))
, d_nbMaxSubTask(initData(&d_nbMaxSubTask, std::numeric_limits< unsigned int >::max(), "nbMaxSubTask", "Number max of subtask created by narrow phase task."))
{
}

FilteredBruteForceDetectionMT::~FilteredBruteForceDetectionMT()
{
    clearTasks();
}

void FilteredBruteForceDetectionMT::clearTasks()
{
    for (MapNarrowPhaseDetectionTask::const_iterator it = m_tasks.begin(),
        itend = m_tasks.end();
        it != itend; ++it)
    {
        delete it->second;
    }

    m_tasks.clear();
}

void FilteredBruteForceDetectionMT::addCollisionPair (const std::pair<sofa::core::CollisionModel*, sofa::core::CollisionModel*>& cmPair)
{
    sofa::core::CollisionModel *cm1 = cmPair.first; //->getNext();
    sofa::core::CollisionModel *cm2 = cmPair.second; //->getNext();

    if (!cm1->isSimulated() && !cm2->isSimulated())
        return;

    if (cm1->empty() || cm2->empty())
        return;

    sofa::core::CollisionModel *finalcm1 = cm1->getLast();//get the finnest CollisionModel which is not a CubeModel
    sofa::core::CollisionModel *finalcm2 = cm2->getLast();
    
    bool swapModels = false;
    sofa::core::collision::ElementIntersector* finalintersector = intersectionMethod->findIntersector(finalcm1, finalcm2, swapModels);//find the method for the finnest CollisionModels
    
    if (finalintersector == NULL)
        return;

    if (swapModels)
    {
        sofa::core::CollisionModel* tmp;
        tmp = cm1; cm1 = cm2; cm2 = tmp;
        tmp = finalcm1; finalcm1 = finalcm2; finalcm2 = tmp;
    }

    auto& outputs = this->getDetectionOutputs(finalcm1, finalcm2);
    finalintersector->beginIntersect(finalcm1, finalcm2, outputs);//creates outputs if null

    NarrowPhaseDetectionTaskInfo taskInfo;
    taskInfo.cm1                = cm1;
    taskInfo.finalcm1           = finalcm1;
    taskInfo.cm2                = cm2;
    taskInfo.finalcm2           = finalcm2;
    taskInfo.finalIntersector   = finalintersector;
    taskInfo.intersectionMethod = intersectionMethod;
    taskInfo.swapModels         = swapModels;
    taskInfo.outputVector       = outputs.first;

    NarrowPhaseDetectionTask*& task = m_tasks[std::make_pair(finalcm1, finalcm2)];
    if (task == NULL)
    {
        task = new NarrowPhaseDetectionTask(finalcm1, finalcm2, m_tasks.size()-1);
    }
    const bool   createIntersectionSubTasks = d_nbIntersectionTestsPerSubTask.getValue() > 0;
    task->enable(&m_taskStatus, taskInfo, createIntersectionSubTasks, d_nbIntersectionTestsPerSubTask.getValue(), d_nbMaxSubTask.getValue());
}


void FilteredBruteForceDetectionMT::beginNarrowPhase()
{
    for (const auto& pair : m_tasks)
    {
        NarrowPhaseDetectionTask* const task = pair.second;
        task->disable();
    }
    SOFA_ASSERT_FAST(m_taskStatus.IsBusy() == false);
    Inherit::beginNarrowPhase();
}


void FilteredBruteForceDetectionMT::endNarrowPhase() 
{
    sofa::simulation::WorkerThread* const thread = sofa::simulation::WorkerThread::GetCurrent();

    for (const auto& pair : m_tasks)
    {
        NarrowPhaseDetectionTask* const task = pair.second;
        if (task->isEnabled())
        {
           thread->addStealableTask(task);
        }
    }

    thread->workUntilDone(&m_taskStatus);

    Inherit::endNarrowPhase();
}


} // namespace collision
} // namespace sofa
