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
#ifndef SOFA_COLLISION_COMPUTEINTERSECTIONTASKS_H
#define SOFA_COLLISION_COMPUTEINTERSECTIONTASKS_H

#include <sofa/simulation/common/Tasks.h>
#include <sofa/core/collision/Intersection.h>
#include <sofa/core/collision/DetectionOutput.h>

namespace sofa
{
namespace collision
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

#endif // SOFA_COLLISION_COMPUTEINTERSECTIONTASKS_H
