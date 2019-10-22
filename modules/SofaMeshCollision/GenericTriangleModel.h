/*******************************************************************************
*          Private SOFA components, (c) 2018 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICTRIANGLEMODEL_H
#define SOFA_COMPONENT_COLLISION_GENERICTRIANGLEMODEL_H

#include "initMeshCollision.h"
#include "BaseTriangleModel.h"
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

template<class TCollisionModel, class TDataTypes = typename TCollisionModel::DataTypes>
class GenericTriangleModel : public BaseTriangleModel
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((GenericTriangleModel<TCollisionModel, TDataTypes>), ((BaseTriangleModel)));
    typedef TDataTypes DataTypes;
    typedef TCollisionModel FinalCollisionModel;

    void init() override;

    virtual void computeBoundingTree(int maxDepth = 0) override;
    virtual void computeContinuousBoundingTree(double dt, int maxDepth = 0) override;

    /// \brief Returns the bounding box of an element.
    //virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance) = 0;
    /// \brief Returns the bounding box of an element, accounting for motions within the given timestep.
    //virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance, double dt) = 0;

    sofa::core::behavior::MechanicalState<DataTypes>* getMechanicalState() const { return this->m_mstate; }

    Data<Real> d_boundaryAngleThreshold;
    Data<Real> d_minTriangleArea;
    Data<Real> d_minEdgeLength;

protected:
    GenericTriangleModel();

    virtual ~GenericTriangleModel();

    /// \brief Update TriangleFlags computed from mesh positions.
    virtual void updateMechanicalTriangleFlags();

    core::behavior::MechanicalState<DataTypes>* m_mstate = nullptr;

private:
    void updateBoundaryFlags();
    void updateBadShapeFlag();
};

} // namespace collision

} // namespace component

} // namespace sofa 

#endif
