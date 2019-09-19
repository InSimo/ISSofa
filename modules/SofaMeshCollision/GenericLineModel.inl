/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_INL
#define SOFA_COMPONENT_COLLISION_GENERICLINEMODEL_INL

#include "GenericLineModel.h"
#include "helpers.h"
#include <cassert>

namespace sofa
{

namespace component
{

namespace collision
{

template<class TCollisionModel, class TDataTypes>
GenericLineModel<TCollisionModel,TDataTypes>::GenericLineModel()
{
    enum_type = LINE_TYPE;
}

template<class TCollisionModel, class TDataTypes>
GenericLineModel<TCollisionModel,TDataTypes>::~GenericLineModel()
{
}

template<class TCollisionModel, class TDataTypes>
void GenericLineModel<TCollisionModel,TDataTypes>::init()
{
    Inherit1::init();

    m_mstate = sofa::core::behavior::MechanicalState<DataTypes>::DynamicCast(this->getContext()->getMechanicalState());

    if (!m_mstate)
    {
        serr << "No MechanicalState found" << sendl;
    }
}

template<class TCollisionModel, class TDataTypes>
void GenericLineModel<TCollisionModel,TDataTypes>::computeBoundingTree(int maxDepth)
{
    FinalCollisionModel* cm = static_cast<FinalCollisionModel*>(this);
    helpers::computeBoundingTree(cm, maxDepth, m_hasTopologicalChange);
    m_hasTopologicalChange = false;
}

template<class TCollisionModel, class TDataTypes>
void GenericLineModel<TCollisionModel,TDataTypes>::computeContinuousBoundingTree(double dt, int maxDepth)
{
    FinalCollisionModel* cm = static_cast<FinalCollisionModel*>(this);
    helpers::computeContinuousBoundingTree(cm, maxDepth, dt, m_hasTopologicalChange);
    m_hasTopologicalChange = false;
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
