/*******************************************************************************
*          Private SOFA components, (c) 2018 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

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
        FLAG_FIRST_CUSTOM = 1 << 12,
        FLAG_POINTS = FLAG_P1 | FLAG_P2 | FLAG_P3,
        FLAG_EDGES = FLAG_E12 | FLAG_E23 | FLAG_E31,
        FLAG_BPOINTS = FLAG_BP1 | FLAG_BP2 | FLAG_BP3,
        FLAG_BEDGES = FLAG_BE12 | FLAG_BE23 | FLAG_BE31,
    };

    void init() override;

    void resize(int size) override;

    void handleTopologyChange(sofa::core::topology::Topology* t) override;

    sofa::core::topology::BaseMeshTopology* getTopology() const { return this->m_topology; }

    int getTriangleFlags(int index) const { return m_triangleFlags[index]; }

protected:
    BaseTriangleModel();

    virtual ~BaseTriangleModel();

    /// \brief Update TriangleFlags computed only from topology.
    virtual void updateTopologicalTriangleFlags();

    sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

    bool m_hasTopologicalChange = true;

    sofa::helper::vector<int> m_triangleFlags;
};

} // namespace collision

} // namespace component

} // namespace sofa 

#endif
