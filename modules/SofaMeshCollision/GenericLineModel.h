/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_H
#define SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_H

#include "initMeshCollision.h"
#include <sofa/core/CollisionModel.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace core
{
namespace behavior
{
template<class DataTypes>
class MechanicalState;
}
}

namespace component
{

namespace collision
{

template<class TDataTypes>
class GenericLineModel : public sofa::core::CollisionModel
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((GenericLineModel<TDataTypes>), ((sofa::core::CollisionModel)));

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

    virtual void computeBoundingTree(int maxDepth = 0) override;
    virtual void computeContinuousBoundingTree(double dt, int maxDepth = 0) override;

    /// \brief Returns the bounding box of an element.
    virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance) = 0;
    /// \brief Returns the bounding box of an element, accounting for motions within the given timestep.
    virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance, double dt) = 0;

    sofa::core::behavior::MechanicalState<TDataTypes>* getMechanicalState() const { return this->m_mstate; }
    sofa::core::topology::BaseMeshTopology* getTopology() const { return this->m_topology; }

    int getLineFlags(int index) const { return m_lineFlags[index]; }

protected:
    GenericLineModel();

    virtual ~GenericLineModel() {};

    /// \brief Update LineFlags computed only from topology.
    virtual void updateTopologicalLineFlags();

    core::behavior::MechanicalState<TDataTypes>* m_mstate = nullptr;
    sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

    bool m_hasTopologicalChange = true;

    sofa::helper::vector<int> m_lineFlags;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_MESH_COLLISION)
#ifndef SOFA_FLOAT
extern template class SOFA_MESH_COLLISION_API GenericLineModel<defaulttype::Vec3dTypes>;
extern template class SOFA_MESH_COLLISION_API GenericLineModel<defaulttype::Rigid3dTypes>;
#endif

#ifndef SOFA_DOUBLE
extern template class SOFA_MESH_COLLISION_API GenericLineModel<defaulttype::Vec3fTypes>;
extern template class SOFA_MESH_COLLISION_API GenericLineModel<defaulttype::Rigid3fTypes>;
#endif
#endif

} // namespace collision

} // namespace component

} // namespace sofa

#endif
