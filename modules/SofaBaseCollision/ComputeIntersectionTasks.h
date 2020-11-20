/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_COMPUTEINTERSECTIONTASKS_H
#define ISPHYSICS_BASE_COMPUTEINTERSECTIONTASKS_H

#include <sofa/simulation/common/Tasks.h>
#include <sofa/core/collision/Intersection.h>
#include <sofa/core/collision/DetectionOutput.h>

#include <isphysics/ISPhysics.h>

ISPHYSICS_INTERNAL

namespace isphysics
{
namespace base
{

struct ComputeIntersectionTaskInfo
{
    using DetectionOutputContainer = sofa::core::collision::DetectionOutputContainer;
    using CollisionElementsPair    = std::pair<sofa::core::CollisionElementIterator,sofa::core::CollisionElementIterator>;
    std::size_t                                                              index;
    sofa::helper::vector<CollisionElementsPair>                              elems;
    sofa::core::collision::ElementIntersector*                               intersector;
    sofa::helper::vector<DetectionOutputContainer*>                          outputPerThread;
};

class ComputeIntersectionTasks : public sofa::simulation::Task
{
public:
    using DetectionOutputContainer = sofa::core::collision::DetectionOutputContainer;

    ComputeIntersectionTasks();

    void enable(const sofa::simulation::Task::Status* pStatus, const ComputeIntersectionTaskInfo& taskInfo);

    bool run(sofa::simulation::WorkerThread* thread);
    
    const char* getName() const { return m_name.c_str(); }

    void setColor(const Color& color) { m_color = color; };

    Color getColor() const override { return m_color; }

    int getOutputThreadIndex() const { return m_outputThreadIndex; }

    int getOutputBeginIndex() const { return m_outputBeginIndex; }

    int getOutputEndIndex() const { return m_outputEndIndex; }

private:
    std::string                  m_name;
    Color                        m_color;
    ComputeIntersectionTaskInfo  m_taskInfo;

    int                          m_outputThreadIndex;
    int                          m_outputBeginIndex;
    int                          m_outputEndIndex;
};


} // namespace base
} // namespace isphysics

#endif // ISPHYSICS_BASE_COMPUTEINTERSECTIONTASKS_H
