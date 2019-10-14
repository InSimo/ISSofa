/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_BASEPOINTMODEL_H
#define SOFA_COMPONENT_COLLISION_BASEPOINTMODEL_H

#include "initMeshCollision.h"
#include <sofa/core/CollisionModel.h>

namespace sofa
{

namespace component
{

namespace collision
{

class SOFA_MESH_COLLISION_API BasePointModel : public sofa::core::CollisionModel
{
public:
    SOFA_ABSTRACT_CLASS_EXTERNAL((BasePointModel), ((sofa::core::CollisionModel)));

    void init() override;

    bool canCollideWithElement(int index, core::CollisionModel* model2, int index2) override;

    void handleTopologyChange(sofa::core::topology::Topology* t) override;
    sofa::core::topology::BaseMeshTopology* getTopology() const final { return this->m_topology; }

protected:
    BasePointModel();

    virtual ~BasePointModel();

    sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

    bool m_hasTopologicalChange = true;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
