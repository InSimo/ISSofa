/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#include "FilteredBruteForceDetectionMT.h"
#include <sofa/simulation/common/TaskScheduler.h>
#include <sofa/core/ObjectFactory.h>
#include <ISSystem/ISAssert.h>

namespace isphysics
{
namespace base
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
    ISASSERT_FAST(m_taskStatus.IsBusy() == false);
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


} // namespace base
} // namespace isphysics
