/*******************************************************************************
*          Private SOFA components, (c) 2018 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICTRIANGLEMODEL_H
#define SOFA_COMPONENT_COLLISION_GENERICTRIANGLEMODEL_H

#include "initMeshCollision.h"
#include <sofa/core/CollisionModel.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/BoundingBox.h>

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
class GenericTriangleModel : public sofa::core::CollisionModel
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((GenericTriangleModel<TDataTypes>), ((sofa::core::CollisionModel)));

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

    virtual void computeBoundingTree(int maxDepth = 0) override;
    virtual void computeContinuousBoundingTree(double dt, int maxDepth = 0) override;

    /// \brief Returns the bounding box of an element.
    virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance) = 0;
    /// \brief Returns the bounding box of an element, accounting for motions within the given timestep.
    virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance, double dt) = 0;

    sofa::core::behavior::MechanicalState<TDataTypes>* getMechanicalState() const { return this->m_mstate; }
    sofa::core::topology::BaseMeshTopology* getTopology() const { return this->m_topology; }

    int getTriangleFlags(int index) const { return m_triangleFlags[index]; }

    Data<Real> d_boundaryAngleThreshold;

protected:
    GenericTriangleModel();

    virtual ~GenericTriangleModel() {};

    /// \brief Update TriangleFlags computed only from topology.
    virtual void updateTopologicalTriangleFlags();
    /// \brief Update TriangleFlags computed from mesh positions.
    virtual void updateMechanicalTriangleFlags();

    core::behavior::MechanicalState<TDataTypes>* m_mstate = nullptr;
    sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

    bool m_hasTopologicalChange = true;

    sofa::helper::vector<int> m_triangleFlags;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_MESH_COLLISION)
#ifndef SOFA_FLOAT
extern template class SOFA_MESH_COLLISION_API GenericTriangleModel<defaulttype::Vec3dTypes>;
extern template class SOFA_MESH_COLLISION_API GenericTriangleModel<defaulttype::Rigid3dTypes>;
#endif

#ifndef SOFA_DOUBLE
extern template class SOFA_MESH_COLLISION_API GenericTriangleModel<defaulttype::Vec3fTypes>;
extern template class SOFA_MESH_COLLISION_API GenericTriangleModel<defaulttype::Rigid3fTypes>;
#endif
#endif

} // namespace collision

} // namespace component

} // namespace sofa 

#endif
