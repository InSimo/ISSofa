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
#ifndef SOFA_COMPONENT_COLLISION_GENERICPOINTITERATOR_H
#define SOFA_COMPONENT_COLLISION_GENERICPOINTITERATOR_H

#include <sofa/core/CollisionModel.h>
#include <sofa/core/VecId.h>

namespace sofa
{
namespace component
{
namespace collision
{

template<class PointCollisionModel>
class GenericPointIterator : public sofa::core::TCollisionElementIterator<PointCollisionModel >
{
public:
    typedef typename PointCollisionModel::DataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef PointCollisionModel ParentModel;

    GenericPointIterator(ParentModel* model, int index);
    GenericPointIterator() {}

    explicit GenericPointIterator(const sofa::core::CollisionElementIterator& i);

    const Coord& p() const;
    const Coord& pFree() const;
    const Deriv& v() const;
    Deriv n() const;

    /// Return true if the element stores a free position vector
    bool hasFreePosition() const;
};

template<class DataTypes>
inline GenericPointIterator<DataTypes>::GenericPointIterator(ParentModel* model, int index)
    : sofa::core::TCollisionElementIterator<ParentModel>(model, index)
{
}

template<class DataTypes>
inline GenericPointIterator<DataTypes>::GenericPointIterator(const sofa::core::CollisionElementIterator& i)
    : sofa::core::TCollisionElementIterator<ParentModel>(static_cast<ParentModel*>(i.getCollisionModel()), i.getIndex())
{
}

template<class PointCollisionModel>
inline const typename GenericPointIterator<PointCollisionModel>::DataTypes::Coord& GenericPointIterator<PointCollisionModel>::p() const { return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[this->index]; }

template<class PointCollisionModel>
inline const typename GenericPointIterator<PointCollisionModel>::DataTypes::Coord& GenericPointIterator<PointCollisionModel>::pFree() const
{
    if (hasFreePosition())
        return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[this->index];
    else
        return p();
}

template<class PointCollisionModel>
inline const typename GenericPointIterator<PointCollisionModel>::DataTypes::Deriv& GenericPointIterator<PointCollisionModel>::v() const { return this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[this->index]; }

template<class PointCollisionModel>
inline typename GenericPointIterator<PointCollisionModel>::DataTypes::Deriv GenericPointIterator<PointCollisionModel>::n() const { return this->model->getNormal(this->index); }

template<class DataTypes>
inline bool GenericPointIterator<DataTypes>::hasFreePosition() const { return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->isSet(); }

} // namespace collision

} // namespace component

} // namespace sofa

#endif
