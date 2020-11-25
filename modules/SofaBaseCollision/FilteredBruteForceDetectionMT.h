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
#ifndef SOFA_COLLISION_FILTEREDBRUTEFORCEDETECTIONMT_H
#define SOFA_COLLISION_FILTEREDBRUTEFORCEDETECTIONMT_H

#include "FilteredBruteForceDetection.h"
#include <sofa/core/CollisionModel.h>
#include "CollisionDetectionTasks.h"
#include <unordered_map>

namespace sofa
{
namespace collision
{

class FilteredBruteForceDetectionMT : public FilteredBruteForceDetection
{
public:
    SOFA_CLASS(FilteredBruteForceDetectionMT, FilteredBruteForceDetection);
    typedef Inherit1 Inherit;

    /// Add a new task for the potentially colliding pairs of models
    void addCollisionPair (const std::pair<sofa::core::CollisionModel*, sofa::core::CollisionModel*>& cmPair);

    /// Clears the task list from the previous step, and call inherited method.
    void beginNarrowPhase();

    /// Wait for completion of the tasks, and call the inherited method.
    void endNarrowPhase();

protected:
    FilteredBruteForceDetectionMT();
    virtual ~FilteredBruteForceDetectionMT();

    void clearTasks();

private:
    // could be changed to unordered map since the order does not matter here, but std::hash has no default implementation when it comes to a pair. 
    typedef std::map<std::pair<sofa::core::CollisionModel*, sofa::core::CollisionModel*>, NarrowPhaseDetectionTask*> MapNarrowPhaseDetectionTask;
    
    sofa::Data< unsigned int>       d_nbIntersectionTestsPerSubTask;
    sofa::Data< unsigned int>       d_nbMaxSubTask;
    sofa::simulation::Task::Status  m_taskStatus;
    MapNarrowPhaseDetectionTask     m_tasks;
    
};



} // namespace collision
} // namespace sofa

#endif // SOFA_COLLISION_FILTEREDBRUTEFORCEDETECTIONMT_H
