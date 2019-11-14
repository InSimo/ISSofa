/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICTRIANGLEITERATOR_H
#define SOFA_COMPONENT_COLLISION_GENERICTRIANGLEITERATOR_H

#include <sofa/core/CollisionElement.h>
#include <sofa/core/VecId.h>

namespace sofa
{
namespace component
{
namespace collision
{

template<class TriangleCollisionModel>
class GenericTriangleIterator : public sofa::core::TCollisionElementIterator<TriangleCollisionModel >
{
public:
    typedef typename TriangleCollisionModel::DataTypes DataTypes;
    typedef typename DataTypes::CPos CPos;
    typedef typename DataTypes::DPos DPos;
    typedef TriangleCollisionModel ParentModel;
    typedef typename DataTypes::Real Real;

    GenericTriangleIterator() {}
    GenericTriangleIterator(ParentModel* model, int index);
    explicit GenericTriangleIterator(const sofa::core::CollisionElementIterator& i);
    GenericTriangleIterator(ParentModel* model, int index, sofa::helper::ReadAccessor<typename DataTypes::VecCoord>& /*x*/);

    const CPos& p1() const;
    const CPos& p2() const;
    const CPos& p3() const;

    const CPos& p(int i)const;
    const CPos& p0(int i)const;

    int p1Index() const;
    int p2Index() const;
    int p3Index() const;

    const CPos& p1Free() const;
    const CPos& p2Free() const;
    const CPos& p3Free() const;

    CPos operator[](int i) const;

    const DPos& v1() const;
    const DPos& v2() const;
    const DPos& v3() const;
    const DPos& v(int i) const;

    const DPos n() const;

    /// Return true if the element stores a free position vector
    bool hasFreePosition() const;

    int flags() const;
    bool intersectOnlyBoundaryPoints() const;
    bool intersectOnlyBoundaryEdges() const;
    bool getTrianglePointProjectionRule() const;

    GenericTriangleIterator& shape() { return *this; }
    const GenericTriangleIterator& shape() const { return *this; }

protected:
    unsigned int getTrianglePointIndex(int i) const;
};


template<class DataTypes>
inline GenericTriangleIterator<DataTypes>::GenericTriangleIterator(ParentModel* model, int index)
    : sofa::core::TCollisionElementIterator<ParentModel>(model, index)
{}

template<class DataTypes>
inline GenericTriangleIterator<DataTypes>::GenericTriangleIterator(const sofa::core::CollisionElementIterator& i)
    : sofa::core::TCollisionElementIterator<ParentModel>(static_cast<ParentModel*>(i.getCollisionModel()), i.getIndex())
{}

template<class TriangleCollisionModel>
inline GenericTriangleIterator<TriangleCollisionModel>::GenericTriangleIterator(ParentModel* model, int index, sofa::helper::ReadAccessor<typename DataTypes::VecCoord>& /*x*/)
    : sofa::core::TCollisionElementIterator<ParentModel>(model, index)
{}


template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::CPos& GenericTriangleIterator<TriangleCollisionModel>::p1() const { return p(0); }

template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::CPos& GenericTriangleIterator<TriangleCollisionModel>::p2() const { return p(1); }

template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::CPos& GenericTriangleIterator<TriangleCollisionModel>::p3() const { return p(2); }

template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::CPos& GenericTriangleIterator<TriangleCollisionModel>::p(int i) const
{
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[getTrianglePointIndex(i)]);
}

template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::CPos& GenericTriangleIterator<TriangleCollisionModel>::p0(int i) const
{
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::restPosition())->getValue()[getTrianglePointIndex(i)]);
}


template<class TriangleCollisionModel>
inline typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::CPos GenericTriangleIterator<TriangleCollisionModel>::operator[](int i) const { return p(i); }


template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::CPos& GenericTriangleIterator<TriangleCollisionModel>::p1Free() const
{
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[getTrianglePointIndex(0)]);
}

template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::CPos& GenericTriangleIterator<TriangleCollisionModel>::p2Free() const
{
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[getTrianglePointIndex(1)]);
}

template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::CPos& GenericTriangleIterator<TriangleCollisionModel>::p3Free() const
{
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[getTrianglePointIndex(2)]);
}


template<class DataTypes>
inline int GenericTriangleIterator<DataTypes>::p1Index() const { return getTrianglePointIndex(0); }

template<class DataTypes>
inline int GenericTriangleIterator<DataTypes>::p2Index() const { return getTrianglePointIndex(1); }

template<class DataTypes>
inline int GenericTriangleIterator<DataTypes>::p3Index() const { return getTrianglePointIndex(2); }


template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::DPos& GenericTriangleIterator<TriangleCollisionModel>::v1() const { return v(0); }

template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::DPos& GenericTriangleIterator<TriangleCollisionModel>::v2() const { return v(1); }

template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::DPos& GenericTriangleIterator<TriangleCollisionModel>::v3() const { return v(2); }

template<class TriangleCollisionModel>
inline const typename GenericTriangleIterator<TriangleCollisionModel>::DataTypes::DPos& GenericTriangleIterator<TriangleCollisionModel>::v(int i) const
{
    return DataTypes::getDPos(this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[getTrianglePointIndex(i)]);
}

template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::n() const -> DPos
{
    return this->index < (int)this->model->getNormals().size() ? DataTypes::getDPos(this->model->getNormals()[this->index]) : DPos();
}

template<class DataTypes>
inline bool GenericTriangleIterator<DataTypes>::hasFreePosition() const
{
    return this->model->getMechanicalState()->read(core::ConstVecCoordId::freePosition())->isSet();
}

template<class DataTypes>
inline int GenericTriangleIterator<DataTypes>::flags() const { return this->model->getTriangleFlags(this->index); }

template<class DataTypes>
inline bool GenericTriangleIterator<DataTypes>::intersectOnlyBoundaryPoints() const { return false; }

template<class DataTypes>
inline bool GenericTriangleIterator<DataTypes>::intersectOnlyBoundaryEdges() const { return this->model->intersectOnlyBoundaryEdges(); }

template<class DataTypes>
inline bool GenericTriangleIterator<DataTypes>::getTrianglePointProjectionRule() const { return false; }

template<class DataTypes>
inline unsigned int GenericTriangleIterator<DataTypes>::getTrianglePointIndex(int i) const
{
    return this->model->getTriangles()[this->index][i];
}


} // namespace collision
} // namespace component
} // namespace sofa
 
#endif
