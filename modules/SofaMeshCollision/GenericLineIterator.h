/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICLINEITERATOR_H
#define SOFA_COMPONENT_COLLISION_GENERICLINEITERATOR_H

#include <sofa/core/CollisionModel.h>
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
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef LineCollisionModel ParentModel;

    GenericLineIterator(ParentModel* model, int index);
    GenericLineIterator() {}

    explicit GenericLineIterator(const sofa::core::CollisionElementIterator& i);

    unsigned i1() const;
    unsigned i2() const;
    int flags() const;

    const Coord& p1() const;
    const Coord& p2() const;
    const Coord& p(int i) const;
    const Coord& p0(int i) const;

    const Coord& p1Free() const;
    const Coord& p2Free() const;

    const Deriv& v1() const;
    const Deriv& v2() const;

    Deriv n() const { return Deriv(); } // TODO: remove this once no code uses it anymore

    /// Return true if the element stores a free position vector
    bool hasFreePosition() const;

    bool activated(sofa::core::CollisionModel *cm = 0) const;

    unsigned int getClassificationSampling() const;
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
inline unsigned GenericLineIterator<DataTypes>::i1() const { return this->model->elems[this->index].p[0]; }

template<class DataTypes>
inline unsigned GenericLineIterator<DataTypes>::i2() const { return this->model->elems[this->index].p[1]; }

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DataTypes::Coord& GenericLineIterator<LineCollisionModel>::p1() const { return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[this->model->elems[this->index].p[0]]; }

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DataTypes::Coord& GenericLineIterator<LineCollisionModel>::p2() const { return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[this->model->elems[this->index].p[1]]; }

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DataTypes::Coord& GenericLineIterator<LineCollisionModel>::p(int i) const {
    return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[this->model->elems[this->index].p[i]];
}

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DataTypes::Coord& GenericLineIterator<LineCollisionModel>::p0(int i) const {
    return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::restPosition())->getValue()[this->model->elems[this->index].p[i]];
}

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DataTypes::Coord& GenericLineIterator<LineCollisionModel>::p1Free() const
{
    if (hasFreePosition())
        return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[this->model->elems[this->index].p[0]];
    else
        return p1();
}

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DataTypes::Coord& GenericLineIterator<LineCollisionModel>::p2Free() const
{
    if (hasFreePosition())
        return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[this->model->elems[this->index].p[1]];
    else
        return p2();
}

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DataTypes::Deriv& GenericLineIterator<LineCollisionModel>::v1() const { return this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[this->model->elems[this->index].p[0]]; }

template<class LineCollisionModel>
inline const typename GenericLineIterator<LineCollisionModel>::DataTypes::Deriv& GenericLineIterator<LineCollisionModel>::v2() const { return this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[this->model->elems[this->index].p[1]]; }


template<class DataTypes>
inline int GenericLineIterator<DataTypes>::flags() const { return this->model->getLineFlags(this->index); }

template<class DataTypes>
inline bool GenericLineIterator<DataTypes>::hasFreePosition() const { return this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->isSet(); }

template<class DataTypes>
inline bool GenericLineIterator<DataTypes>::activated(sofa::core::CollisionModel *cm) const
{
    return this->model->myActiver->activeLine(this->index, cm);
}

template<class DataTypes>
inline unsigned int GenericLineIterator<DataTypes>::getClassificationSampling() const
{
    return this->model->getClassificationSampling(this->index);
}

} // namespace collision 
} // namespace component
} // namespace sofa

#endif