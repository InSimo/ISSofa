/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_H
#define SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_H

#include "initMeshCollision.h"
#include "BaseLineModel.h"
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
class GenericLineModel : public BaseLineModel
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((GenericLineModel<TCollisionModel, TDataTypes>), ((BaseLineModel)));

    typedef TCollisionModel FinalCollisionModel;
    typedef TDataTypes DataTypes;

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

    virtual void computeBoundingTree(int maxDepth = 0) override;
    virtual void computeContinuousBoundingTree(double dt, int maxDepth = 0) override;

    /// \brief Returns the bounding box of an element.
    //virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance) = 0;
    /// \brief Returns the bounding box of an element, accounting for motions within the given timestep.
    //virtual defaulttype::BoundingBox computeElementBBox(int index, SReal distance, double dt) = 0;

    sofa::core::behavior::MechanicalState<TDataTypes>* getMechanicalState() const { return this->m_mstate; }

protected:
    GenericLineModel();

    virtual ~GenericLineModel();

    core::behavior::MechanicalState<TDataTypes>* m_mstate = nullptr;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
