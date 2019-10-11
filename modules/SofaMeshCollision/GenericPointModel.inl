/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICPOINTMODEL_INL
#define SOFA_COMPONENT_COLLISION_GENERICPOINTMODEL_INL

#include "GenericPointModel.h"
#include "helpers.h"

namespace sofa
{

namespace component
{

namespace collision
{

template<class TCollisionModel, class TDataTypes>
GenericPointModel<TCollisionModel,TDataTypes>::GenericPointModel()
{
    enum_type = POINT_TYPE;
}

template<class TCollisionModel, class TDataTypes>
GenericPointModel<TCollisionModel,TDataTypes>::~GenericPointModel()
{
}

template<class TCollisionModel, class TDataTypes>
void GenericPointModel<TCollisionModel,TDataTypes>::init()
{
    Inherit1::init();

    m_mstate = sofa::core::behavior::MechanicalState<DataTypes>::DynamicCast(this->getContext()->getMechanicalState());

    if (!m_mstate)
    {
        serr << "No MechanicalState found" << sendl;
    }
}

template<class TCollisionModel, class TDataTypes>
void GenericPointModel<TCollisionModel,TDataTypes>::computeBoundingTree(int maxDepth)
{
    FinalCollisionModel* cm = static_cast<FinalCollisionModel*>(this);
    helpers::computeBoundingTree(cm, maxDepth, m_hasTopologicalChange);
    m_hasTopologicalChange = false;
}

template<class TCollisionModel, class TDataTypes>
void GenericPointModel<TCollisionModel,TDataTypes>::computeContinuousBoundingTree(double dt, int maxDepth)
{
    FinalCollisionModel* cm = static_cast<FinalCollisionModel*>(this);
    helpers::computeContinuousBoundingTree(cm, maxDepth, dt, m_hasTopologicalChange);
    m_hasTopologicalChange = false;
}

template<class TCollisionModel, class TDataTypes>
defaulttype::BoundingBox GenericPointModel<TCollisionModel,TDataTypes>::computeElementBBox(int index, SReal distance) const
{
    auto x = this->m_mstate->readPositions();

    sofa::defaulttype::BoundingBox bbox;
    bbox.include(DataTypes::getCPos(x[index]));
    bbox.inflate(distance);

    return bbox;
}

template<class TCollisionModel, class TDataTypes>
defaulttype::BoundingBox GenericPointModel<TCollisionModel,TDataTypes>::computeElementBBox(int index, SReal distance, double dt) const
{
    auto x = this->m_mstate->readPositions();
    auto v = this->m_mstate->readVelocities();

    sofa::defaulttype::BoundingBox bbox;
    typename DataTypes::CPos xi = DataTypes::getCPos(x[index]);
    bbox.include(xi);
    bbox.include(xi + DataTypes::getDPos(v[index]) * dt);
    bbox.inflate(distance);

    return bbox;
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
