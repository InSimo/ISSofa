/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_BASELINEMODEL_H
#define SOFA_COMPONENT_COLLISION_BASELINEMODEL_H

#include "initMeshCollision.h"
#include <sofa/core/CollisionModel.h>

namespace sofa
{

namespace component
{

namespace collision
{

class SOFA_MESH_COLLISION_API BaseLineModel : public sofa::core::CollisionModel
{
public:
    SOFA_ABSTRACT_CLASS_EXTERNAL((BaseLineModel), ((sofa::core::CollisionModel)));

    enum LineFlag
    {
        FLAG_P1  = 1<<0, ///< Point 1  is attached to this line
        FLAG_P2  = 1<<1, ///< Point 2  is attached to this line
        FLAG_BP1 = 1<<2, ///< Point 1  is attached to this line and is a boundary
        FLAG_BP2 = 1<<3, ///< Point 2  is attached to this line and is a boundary
        FLAG_POINTS  = FLAG_P1|FLAG_P2,
        FLAG_BPOINTS = FLAG_BP1|FLAG_BP2
    };

    void init() override;

    void resize(int size) override;

    void handleTopologyChange(sofa::core::topology::Topology* t) override;
    sofa::core::topology::BaseMeshTopology* getTopology() const final { return this->m_topology; }

    int getLineFlags(int index) const { return m_lineFlags[index]; }

protected:
    BaseLineModel();

    virtual ~BaseLineModel();

    /// \brief Update LineFlags computed only from topology.
    virtual void updateTopologicalLineFlags();

    sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

    bool m_hasTopologicalChange = true;

    sofa::helper::vector<int> m_lineFlags;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
