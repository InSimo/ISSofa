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
#ifndef SOFA_COMPONENT_COLLISION_BASETRIANGLEMODEL_H
#define SOFA_COMPONENT_COLLISION_BASETRIANGLEMODEL_H

#include "initMeshCollision.h"
#include <sofa/core/CollisionModel.h>

namespace sofa
{

namespace component
{

namespace collision
{

class SOFA_MESH_COLLISION_API BaseTriangleModel : public sofa::core::CollisionModel
{
public:
    SOFA_ABSTRACT_CLASS_EXTERNAL((BaseTriangleModel), ((sofa::core::CollisionModel)));

    enum TriangleFlag
    {
        FLAG_P1 = 1 << 0,  ///< Point 1  is attached to this triangle
        FLAG_P2 = 1 << 1,  ///< Point 2  is attached to this triangle
        FLAG_P3 = 1 << 2,  ///< Point 3  is attached to this triangle
        FLAG_BP1 = 1 << 3,  ///< Point 1  is attached to this triangle and is a boundary
        FLAG_BP2 = 1 << 4,  ///< Point 2  is attached to this triangle and is a boundary
        FLAG_BP3 = 1 << 5,  ///< Point 3  is attached to this triangle and is a boundary
        FLAG_E23 = 1 << 6,  ///< Edge 2-3 is attached to this triangle
        FLAG_E31 = 1 << 7,  ///< Edge 3-1 is attached to this triangle
        FLAG_E12 = 1 << 8,  ///< Edge 1-2 is attached to this triangle
        FLAG_BE23 = 1 << 9,  ///< Edge 2-3 is attached to this triangle and is a boundary
        FLAG_BE31 = 1 << 10, ///< Edge 3-1 is attached to this triangle and is a boundary
        FLAG_BE12 = 1 << 11, ///< Edge 1-2 is attached to this triangle and is a boundary
        FLAG_BADSHAPE = 1 << 12, ///< Triangle has a bad shape and should be ignored
        FLAG_FIRST_CUSTOM = 1 << 13,
        FLAG_POINTS = FLAG_P1 | FLAG_P2 | FLAG_P3,
        FLAG_EDGES = FLAG_E12 | FLAG_E23 | FLAG_E31,
        FLAG_BPOINTS = FLAG_BP1 | FLAG_BP2 | FLAG_BP3,
        FLAG_BEDGES = FLAG_BE12 | FLAG_BE23 | FLAG_BE31,
    };

    void init() override;

    void resize(int size) override;

    bool canCollideWithElement(int index, core::CollisionModel* model2, int index2) override;

    void handleTopologyChange(sofa::core::topology::Topology* t) override;

    sofa::core::topology::BaseMeshTopology* getTopology() const final { return this->m_topology; }

    int getTriangleFlags(int index) const { return m_triangleFlags[index]; }

    bool intersectOnlyBoundaryEdges() const { return d_intersectOnlyBoundaryEdges.getValue(); }

protected:
    BaseTriangleModel();

    virtual ~BaseTriangleModel();

    /// \brief Update TriangleFlags computed only from topology.
    virtual void updateTopologicalTriangleFlags();

    sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

    bool m_hasTopologicalChange = true;

    sofa::helper::vector<int> m_triangleFlags;

    Data<bool> d_intersectOnlyBoundaryEdges;
};

} // namespace collision

} // namespace component

} // namespace sofa 

#endif
