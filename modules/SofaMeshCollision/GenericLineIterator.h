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
#ifndef SOFA_COMPONENT_COLLISION_GENERICLINEITERATOR_H
#define SOFA_COMPONENT_COLLISION_GENERICLINEITERATOR_H

#include <sofa/core/CollisionElement.h>
#include <sofa/core/VecId.h>

namespace sofa
{
namespace component
{
namespace collision
{

template<class LineCollisionModel>
class GenericLineIterator : public sofa::core::TCollisionElementIterator<LineCollisionModel >
{
public:
    typedef typename LineCollisionModel::DataTypes DataTypes;
    typedef typename DataTypes::CPos CPos;
    typedef typename DataTypes::DPos DPos;
    typedef LineCollisionModel ParentModel;

    GenericLineIterator(ParentModel* model, int index);
    GenericLineIterator() {}

    explicit GenericLineIterator(const sofa::core::CollisionElementIterator& i);

    unsigned i1() const;
    unsigned i2() const;
    int flags() const;

    const CPos& p1() const;
    const CPos& p2() const;
    const CPos& p(int i) const;
    const CPos& p0(int i) const;

    const CPos& p1Free() const;
    const CPos& p2Free() const;

    const DPos& v1() const;
    const DPos& v2() const;

    DPos n() const { return DPos(); } // TODO: remove this once no code uses it anymore

    /// Return true if the element stores a free position vector
    bool hasFreePosition() const;

    unsigned int getClassificationSampling() const;

protected:
    unsigned int getLinePointIndex(int i) const;
};


template<class DataTypes>
inline GenericLineIterator<DataTypes>::GenericLineIterator(ParentModel* model, int index)
    : sofa::core::TCollisionElementIterator<ParentModel>(model, index)
{
}

template<class DataTypes>
inline GenericLineIterator<DataTypes>::GenericLineIterator(const sofa::core::CollisionElementIterator& i)
    : sofa::core::TCollisionElementIterator<ParentModel>(static_cast<ParentModel*>(i.getCollisionModel()), i.getIndex())
{
}

template<class DataTypes>
inline unsigned GenericLineIterator<DataTypes>::i1() const { return getLinePointIndex(0); }

template<class DataTypes>
inline unsigned GenericLineIterator<DataTypes>::i2() const { return getLinePointIndex(1); }

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::CPos& GenericLineIterator<LineCollisionModel>::p1() const { return p(0); }

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::CPos& GenericLineIterator<LineCollisionModel>::p2() const { return p(1); }

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::CPos& GenericLineIterator<LineCollisionModel>::p(int i) const
{
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[getLinePointIndex(i)]);
}

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::CPos& GenericLineIterator<LineCollisionModel>::p0(int i) const
{
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::restPosition())->getValue()[getLinePointIndex(i)]);
}

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::CPos& GenericLineIterator<LineCollisionModel>::p1Free() const
{
    if (hasFreePosition())
        return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[getLinePointIndex(0)]);
    else
        return p1();
}

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::CPos& GenericLineIterator<LineCollisionModel>::p2Free() const
{
    if (hasFreePosition())
        return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[getLinePointIndex(1)]);
    else
        return p2();
}

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DPos& GenericLineIterator<LineCollisionModel>::v1() const { return DataTypes::getDPos(this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[getLinePointIndex(0)]); }

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DPos& GenericLineIterator<LineCollisionModel>::v2() const { return DataTypes::getDPos(this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[getLinePointIndex(1)]); }


template<class DataTypes>
inline int GenericLineIterator<DataTypes>::flags() const { return this->model->getLineFlags(this->index); }

template<class DataTypes>
inline bool GenericLineIterator<DataTypes>::hasFreePosition() const { return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->isSet(); }

template<class DataTypes>
inline unsigned int GenericLineIterator<DataTypes>::getClassificationSampling() const
{
    return this->model->getClassificationSampling(this->index);
}

template<class DataTypes>
inline unsigned int GenericLineIterator<DataTypes>::getLinePointIndex(int i) const
{
    return this->model->getTopology()->getEdge(this->index)[i];
}

} // namespace collision 
} // namespace component
} // namespace sofa

#endif
