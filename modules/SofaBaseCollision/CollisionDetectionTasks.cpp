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
#include "CollisionDetectionTasks.h"

#include "ComputeIntersectionTasks.h"

#include <sofa/simulation/common/TaskScheduler.h>
#include <SofaBaseCollision/BruteForceDetection.h>

#include <queue>
#include <stack>

namespace sofa
{
namespace collision
{

unsigned char NarrowPhaseDetectionTask::taskColors[][3] = {
    {166,50,139},
    {235,110,33},
    {185,125,111},
    {226,47,94},
    {228,54,201},
    {154,67,43},
    {155,65,89},
    {235,61,57},
    {227,52,135},
    {225,108,171},
    {228,99,96},
    {199,120,147},
    {174,49,20},
    {224,88,126},
    {224,51,165},
    {205,127,87},
    {177,88,28},
    {211,91,189},
    {128,76,50},
    {236,60,25},
    {225,100,68},
    {179,39,47},
    {146,54,113},
    {169,38,91},
    {151,53,59},
    {232,132,62},
    {207,115,115},
    {128,71,89},
    {201,76,16}};

NarrowPhaseDetectionTask::NarrowPhaseDetectionTask(sofa::core::CollisionModel* finalcm1, sofa::core::CollisionModel* finalcm2, int index)
:m_index(index)
,m_createIntersectionSubTasks(false)
{
    m_name = finalcm1->getName() + std::string("-") + finalcm2->getName();
    m_color = Color(taskColors[m_index%NCOLORS][0]/255.0f,
                    taskColors[m_index%NCOLORS][1]/255.0f,
                    taskColors[m_index%NCOLORS][2]/255.0f,
                    1.0f);
}

NarrowPhaseDetectionTask::~NarrowPhaseDetectionTask()
{
    for (const auto& outputVector : m_vecOutputPerThread)
    {
        delete outputVector;
    }

    for (const auto& task : m_vecComputeIntersectionTasks)
    {
        delete task;
    }
}

void NarrowPhaseDetectionTask::enable(const sofa::simulation::Task::Status* pStatus, const NarrowPhaseDetectionTaskInfo& taskInfo, bool createIntersectionSubTasks, unsigned int nbIntersectionTestsPerSubTask, unsigned int nbMaxSubTask)
{
    sofa::simulation::Task::enable(pStatus);
    this->m_taskInfo = taskInfo;
    this->m_createIntersectionSubTasks = createIntersectionSubTasks;

    if (m_lastNbOfSubTask > nbMaxSubTask)
    {
        unsigned int lastNbIntersectionTests = std::max(m_correctedNbIntersectionTestsPerSubTask, nbIntersectionTestsPerSubTask)*m_lastNbOfSubTask;
        // Force number of intersection per subtask avoiding max subtask number overflow
        m_correctedNbIntersectionTestsPerSubTask = lastNbIntersectionTests / nbMaxSubTask + (lastNbIntersectionTests % nbMaxSubTask == 0u ? 0u : 1u);

        sofa::core::objectmodel::BaseObject* cm1 = (taskInfo.cm1->getMaster() != nullptr) ? taskInfo.cm1->getMaster() : taskInfo.cm1;
        sofa::core::objectmodel::BaseObject* cm2 = (taskInfo.cm2->getMaster() != nullptr) ? taskInfo.cm2->getMaster() : taskInfo.cm2;
        sofa::core::collision::Intersection* intersectionMethod = m_taskInfo.intersectionMethod;
        std::cout << "BruteForceDetection: " << m_lastNbOfSubTask << " subtasks created (max is " << nbMaxSubTask << ") for " << intersectionMethod->getName() << " between " << cm1->getName() << " - " << cm2->getName()
            << ", nbIntersectionTestsPerSubTask forced to " << m_correctedNbIntersectionTestsPerSubTask << std::endl;
    }
    this->m_nbIntersectionTestsPerSubTask = std::max(m_correctedNbIntersectionTestsPerSubTask, nbIntersectionTestsPerSubTask);
}


bool NarrowPhaseDetectionTask::run( sofa::simulation::WorkerThread* thread)
{
    sofa::core::CollisionModel*                   cm1                = m_taskInfo.cm1;
    sofa::core::CollisionModel*                   cm2                = m_taskInfo.cm2;
    sofa::core::CollisionModel*                   finalcm1           = m_taskInfo.finalcm1;
    sofa::core::CollisionModel*                   finalcm2           = m_taskInfo.finalcm2;
    sofa::core::collision::ElementIntersector*    finalintersector   = m_taskInfo.finalIntersector;
    sofa::core::collision::DetectionOutputContainer* outputs         = m_taskInfo.outputVector;
    sofa::core::collision::Intersection*          intersectionMethod = m_taskInfo.intersectionMethod;
    bool                                          swapModels         = m_taskInfo.swapModels;
    const bool                                    self               = (finalcm1->getContext() == finalcm2->getContext());
    
    
    sofa::simulation::Task::Status taskStatus;
    std::size_t taskIndex = 0;
    sofa::helper::vector<CollisionElementsPair> intersectionElems;
    intersectionElems.reserve(m_nbIntersectionTestsPerSubTask);
    
    if (m_createIntersectionSubTasks)
    {
        for (auto& task : m_vecComputeIntersectionTasks)
        {
            task->disable();
        }

        // Initialize a DetectionOutputContainer per thread
        m_vecOutputPerThread.resize(thread->getTaskScheduler()->getThreadCount(), nullptr);
        for (auto& outputVectorPtr : m_vecOutputPerThread)
        {
            if (!outputVectorPtr)
            {
                outputVectorPtr = outputs->create();
            }
            else
            {
                outputVectorPtr->clear();
            }
        }
    }
    

    if (finalcm1 == cm1 || finalcm2 == cm2)
    {
        // The last model also contains the root element -> it does not only contains the final level of the tree
        finalcm1 = nullptr;
        finalcm2 = nullptr;
        finalintersector = nullptr;
    }

    std::queue< TestPair > externalCells;

    CollisionElementsPair internalChildren1 = cm1->begin().getInternalChildren();
    CollisionElementsPair internalChildren2 = cm2->begin().getInternalChildren();
    CollisionElementsPair externalChildren1 = cm1->begin().getExternalChildren();
    CollisionElementsPair externalChildren2 = cm2->begin().getExternalChildren();
    if (internalChildren1.first != internalChildren1.second)
    {
        if (internalChildren2.first != internalChildren2.second)
            externalCells.emplace(internalChildren1,internalChildren2);
        if (externalChildren2.first != externalChildren2.second)
            externalCells.emplace(internalChildren1,externalChildren2);
    }
    if (externalChildren1.first != externalChildren1.second)
    {
        if (internalChildren2.first != internalChildren2.second)
            externalCells.emplace(externalChildren1,internalChildren2);
        if (externalChildren2.first != externalChildren2.second)
            externalCells.emplace(externalChildren1,externalChildren2);
    }
    //externalCells.push(std::make_pair(std::make_pair(cm1->begin(),cm1->end()),std::make_pair(cm2->begin(),cm2->end())));

    //core::collision::ElementIntersector* intersector = intersectionMethod->findIntersector(cm1, cm2);
    sofa::core::collision::ElementIntersector* intersector = nullptr;
    sofa::component::collision::MirrorIntersector mirror;
    cm1 = nullptr; // force later init of intersector
    cm2 = nullptr;

    while (!externalCells.empty())
    {
        TestPair root = externalCells.front();
        externalCells.pop();

        if (cm1 != root.first.first.getCollisionModel() || cm2 != root.second.first.getCollisionModel())//if the CollisionElements do not belong to cm1 and cm2, update cm1 and cm2
        {
            cm1 = root.first.first.getCollisionModel();
            cm2 = root.second.first.getCollisionModel();
            if (!cm1 || !cm2) continue;
            intersector = intersectionMethod->findIntersector(cm1, cm2, swapModels);

            if (intersector == nullptr)
            {
                std::cout << "BruteForceDetection: Error finding intersector " << intersectionMethod->getName() << " for "<<cm1->getClassName()<<" - "<<cm2->getClassName()<<std::endl;
            }
            //else std::cout << "BruteForceDetection: intersector " << intersector->name() << " for " << intersectionMethod->getName() << " for "<<gettypename(typeid(*cm1))<<" - "<<gettypename(typeid(*cm2))<<std::endl;
            if (swapModels)
            {
                mirror.intersector = intersector; intersector = &mirror;
            }
        }
        
        if (intersector == nullptr)
            continue;
        
        std::stack< TestPair > internalCells;
        internalCells.push(root);

        while (!internalCells.empty())
        {
            const TestPair& current = internalCells.top();
            
            sofa::core::CollisionElementIterator begin1 = current.first.first;
            sofa::core::CollisionElementIterator end1 = current.first.second;
            sofa::core::CollisionElementIterator begin2 = current.second.first;
            sofa::core::CollisionElementIterator end2 = current.second.second;
            
            internalCells.pop();

            if (begin1.getCollisionModel() == finalcm1 && begin2.getCollisionModel() == finalcm2)
            {
                // Final collision pairs
                for (sofa::core::CollisionElementIterator it1 = begin1; it1 != end1; ++it1)
                {
                    for (sofa::core::CollisionElementIterator it2 = begin2; it2 != end2; ++it2)
                    {
                        if (!self || it1.canCollideWith(it2))
                            intersector->intersect(it1,it2,outputs);
                    }
                }
            }
            else
            {
                for (sofa::core::CollisionElementIterator it1 = begin1; it1 != end1; ++it1)
                {
                    for (sofa::core::CollisionElementIterator it2 = begin2; it2 != end2; ++it2)
                    {
                        if (intersector->canIntersect(it1,it2))
                        {
                            // Need to test recursively
                            // Note that an element cannot have both internal and external children
                            TestPair newInternalTests(it1.getInternalChildren(),it2.getInternalChildren());
                            
                            if (newInternalTests.first.first != newInternalTests.first.second)
                            {
                                if (newInternalTests.second.first != newInternalTests.second.second)
                                {
                                    internalCells.push(newInternalTests);
                                }
                                else
                                {
                                    newInternalTests.second.first = it2;
                                    newInternalTests.second.second = it2;
                                    ++newInternalTests.second.second;
                                    internalCells.push(newInternalTests);
                                }
                            }
                            else
                            {
                                if (newInternalTests.second.first != newInternalTests.second.second)
                                {
                                    newInternalTests.first.first = it1;
                                    newInternalTests.first.second = it1;
                                    ++newInternalTests.first.second;
                                    internalCells.push(newInternalTests);
                                }
                                else
                                {
                                    // end of both internal tree of elements.
                                    // need to test external children
                                    TestPair newExternalTests(it1.getExternalChildren(),it2.getExternalChildren());
                                    
                                    if (newExternalTests.first.first != newExternalTests.first.second)
                                    {
                                        if (newExternalTests.second.first != newExternalTests.second.second)
                                        {
                                            if (newExternalTests.first.first.getCollisionModel() == finalcm1 && newExternalTests.second.first.getCollisionModel() == finalcm2)
                                            {
                                                sofa::core::CollisionElementIterator begin1 = newExternalTests.first.first;
                                                sofa::core::CollisionElementIterator end1 = newExternalTests.first.second;
                                                sofa::core::CollisionElementIterator begin2 = newExternalTests.second.first;
                                                sofa::core::CollisionElementIterator end2 = newExternalTests.second.second;

                                                for (sofa::core::CollisionElementIterator it1 = begin1; it1 != end1; ++it1)
                                                {
                                                    for (sofa::core::CollisionElementIterator it2 = begin2; it2 != end2; ++it2)
                                                    {
                                                        // Final collision pair
                                                        if (!self || it1.canCollideWith(it2))
                                                        {
                                                            if (m_createIntersectionSubTasks)
                                                            {
                                                                intersectionElems.emplace_back(it1, it2);
                                                                
                                                                if (intersectionElems.size() >= m_nbIntersectionTestsPerSubTask)
                                                                {
                                                                    if (taskIndex >= m_vecComputeIntersectionTasks.size())
                                                                    {
                                                                        m_vecComputeIntersectionTasks.push_back(new ComputeIntersectionTasks());
                                                                        // use nearly same colors as parent NarrowPhaseDetectionTask, but a bit darker
                                                                        Color taskColor = getColor();
                                                                        for (std::size_t ci = 0; ci < 3; ++ci)
                                                                        {
                                                                            taskColor[ci] *= 0.85f;
                                                                        }
                                                                        m_vecComputeIntersectionTasks[taskIndex]->setColor(taskColor);
                                                                    }

                                                                    ComputeIntersectionTaskInfo taskInfo{taskIndex, intersectionElems, finalintersector, m_vecOutputPerThread};
                                                                    m_vecComputeIntersectionTasks[taskIndex]->enable(&taskStatus, taskInfo);
                                                                    thread->addStealableTask(m_vecComputeIntersectionTasks[taskIndex]);
                                                                    ++taskIndex;
                                                                    intersectionElems.clear();
                                                                }
                                                            }
                                                            else
                                                            {
                                                                finalintersector->intersect(it1,it2,outputs);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                            else
                                            {
                                                externalCells.push(newExternalTests);
                                            }
                                        }
                                        else
                                        {
                                            // only first element has external children
                                            // test them against the second element
                                            newExternalTests.second.first = it2;
                                            newExternalTests.second.second = it2;
                                            ++newExternalTests.second.second;
                                            externalCells.push(newExternalTests);
                                        }
                                    }
                                    else if (newExternalTests.second.first != newExternalTests.second.second)
                                    {
                                        // only second element has external children
                                        // test them against the first element
                                        newExternalTests.first.first = it1;
                                        newExternalTests.first.second = it1;
                                        ++newExternalTests.first.second;
                                        externalCells.push(newExternalTests);
                                    }
                                    else
                                    {
                                        // No child -> final collision pair
                                        if (!self || it1.canCollideWith(it2))
                                            intersector->intersect(it1,it2, outputs);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    

    if (m_createIntersectionSubTasks)
    {
        if (!intersectionElems.empty())
        {
            if (taskIndex >= m_vecComputeIntersectionTasks.size())
            {
                m_vecComputeIntersectionTasks.push_back(new ComputeIntersectionTasks());
                // use nearly same colors as parent NarrowPhaseDetectionTask, but a bit darker
                Color taskColor = getColor();
                for (std::size_t ci = 0; ci < 3; ++ci)
                {
                    taskColor[ci] *= 0.85f;
                }
                m_vecComputeIntersectionTasks[taskIndex]->setColor(taskColor);
            }

            ComputeIntersectionTaskInfo taskInfo{taskIndex, intersectionElems, finalintersector, m_vecOutputPerThread};

            m_vecComputeIntersectionTasks[taskIndex]->enable(&taskStatus, taskInfo);
            thread->addStealableTask(m_vecComputeIntersectionTasks[taskIndex]);
            ++taskIndex;
            // no need to clear intersectionElems here
        }

        thread->workUntilDone(&taskStatus);

        std::size_t sizeToReserve = outputs->size();
        for (const DetectionOutputContainer* const vec : m_vecOutputPerThread)
        {
            sizeToReserve += vec->size();
        }
        outputs->reserve(sizeToReserve);
        
        // Insert the DetectionOutputs in the output vector in the order of creation of the tasks, so that their ordering is determinist
        for (const ComputeIntersectionTasks* const task : m_vecComputeIntersectionTasks)
        {
            if (!task->isEnabled()) continue; // /!\ skip if task wasn't used this step
            const DetectionOutputContainer& vecDetectionOutput = *m_vecOutputPerThread[task->getOutputThreadIndex()];
            const int start = task->getOutputBeginIndex();
            const int len   = task->getOutputEndIndex() - task->getOutputBeginIndex();
            outputs->insertFrom(vecDetectionOutput, start, len);
        }
    }

    m_lastNbOfSubTask = taskIndex;

    return true;
}


} // namespace base
} // namespace isphysics
