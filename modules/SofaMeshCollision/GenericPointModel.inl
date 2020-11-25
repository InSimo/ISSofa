/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 MGH, INRIA, USTL, UJF, CNRS, InSimo                *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
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
    if (this->getSize() != this->getMechanicalState()->getSize())
    {
        if (m_hasTopologicalChange)
        {
            serr << "Something went wrong during topology changes, the size of the collision model does not match the size of the mechanical state" << sendl;
        }
        else
        {
            serr << "No complete set of topology changes detected, but the size of the collision model does not match the size of the mechanical state. Is something resizing the mechanical state directly or is the ending event missing ?" << sendl;
        }
        serr << "Collision model size = " << this->getSize() << ", mechanical state size = " << this->getMechanicalState()->getSize() << sendl;
    }

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
