/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICPOINTMODEL_H
#define SOFA_COMPONENT_COLLISION_GENERICPOINTMODEL_H

#include "initMeshCollision.h"
#include "BasePointModel.h"
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
class GenericPointModel : public BasePointModel
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((GenericPointModel<TCollisionModel, TDataTypes>), ((BasePointModel)));

    typedef TCollisionModel FinalCollisionModel;
    typedef TDataTypes DataTypes;

    void init() override;

    virtual void computeBoundingTree(int maxDepth = 0) override;
    virtual void computeContinuousBoundingTree(double dt, int maxDepth = 0) override;

    /// \brief Returns the bounding box of an element.
    defaulttype::BoundingBox computeElementBBox(int index, SReal distance) const;
    /// \brief Returns the bounding box of an element, accounting for motions within the given timestep.
    defaulttype::BoundingBox computeElementBBox(int index, SReal distance, double dt) const;

    sofa::core::behavior::MechanicalState<TDataTypes>* getMechanicalState() const { return this->m_mstate; }

protected:
    GenericPointModel();

    virtual ~GenericPointModel();

    core::behavior::MechanicalState<TDataTypes>* m_mstate = nullptr;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
