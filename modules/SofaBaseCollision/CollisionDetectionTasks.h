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
#ifndef SOFA_COLLISION_COLLISIONDETECTIONTASKS_H
#define SOFA_COLLISION_COLLISIONDETECTIONTASKS_H

#include <sofa/simulation/common/Tasks.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/core/collision/Intersection.h>

namespace sofa
{
namespace collision
{


struct NarrowPhaseDetectionTaskInfo 
{
    sofa::core::CollisionModel*                   cm1      = nullptr;///< collision model 1 
    sofa::core::CollisionModel*                   finalcm1 = nullptr;///< collision model 1 final element involved in the narrow phase
    sofa::core::CollisionModel*                   cm2      = nullptr;///< collision model 2
    sofa::core::CollisionModel*                   finalcm2 = nullptr;///< collision model 2 final element involved in the narrow phase
    sofa::core::collision::ElementIntersector*    finalIntersector   = nullptr;///< intersector for finalcm1 and finalcm2
    sofa::core::collision::Intersection*          intersectionMethod = nullptr;///< 
    bool                                          swapModels         = false;///< true if models should be swapped.
    sofa::core::collision::DetectionOutputContainer* outputVector    = nullptr;///< the output vector the intersector can write into.
};

class ComputeIntersectionTasks;

class NarrowPhaseDetectionTask : public sofa::simulation::Task
{
public:
    using DetectionOutputContainer = sofa::core::collision::DetectionOutputContainer;
    using CollisionElementsPair = std::pair<sofa::core::CollisionElementIterator,sofa::core::CollisionElementIterator>;
    using TestPair = std::pair< CollisionElementsPair, CollisionElementsPair>;

    NarrowPhaseDetectionTask(sofa::core::CollisionModel* finalcm1, sofa::core::CollisionModel* finalcm2, int index = 0);

    ~NarrowPhaseDetectionTask();

    void enable(const sofa::simulation::Task::Status* pStatus, const NarrowPhaseDetectionTaskInfo& taskInfo, bool createIntersectionSubTasks = false, unsigned int nbIntersectionTestsPerSubTask = 0u, unsigned int nbMaxSubTask = std::numeric_limits< unsigned int >::max());
   
    bool run(sofa::simulation::WorkerThread* thread);
    
    const char* getName() const { return m_name.c_str(); }
    
    Color getColor() const { return m_color; }
        
private:
    std::string                  m_name;
    int                          m_index;
    Color                        m_color;
    NarrowPhaseDetectionTaskInfo m_taskInfo;

    bool m_createIntersectionSubTasks;
    unsigned int m_nbIntersectionTestsPerSubTask = 0u;
    unsigned int m_correctedNbIntersectionTestsPerSubTask = 0u;
    unsigned int m_lastNbOfSubTask = 0u;

    sofa::helper::vector<DetectionOutputContainer*> m_vecOutputPerThread;
    sofa::helper::vector<ComputeIntersectionTasks*> m_vecComputeIntersectionTasks;

    enum { NCOLORS = 29 };
    static unsigned char taskColors[NCOLORS][3];
};


} // namespace base
} // namespace isphysics

#endif // SOFA_COLLISION_COLLISIONDETECTIONTASKS_H
