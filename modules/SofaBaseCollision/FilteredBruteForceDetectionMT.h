/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#ifndef ISPHYSICS_BASE_FILTEREDBRUTEFORCEDETECTIONMT_H
#define ISPHYSICS_BASE_FILTEREDBRUTEFORCEDETECTIONMT_H

#include "FilteredBruteForceDetection.h"
#include <sofa/core/CollisionModel.h>
#include "CollisionDetectionTasks.h"
#include <unordered_map>

ISPHYSICS_INTERNAL

namespace isphysics
{
namespace base
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



} // namespace base
} // namespace isphysics

#endif // ISPHYSICS_BASE_FILTEREDBRUTEFORCEDETECTIONMT_H
