/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICTRIANGLEITERATOR_H
#define SOFA_COMPONENT_COLLISION_GENERICTRIANGLEITERATOR_H

#include <sofa/core/CollisionModel.h>
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
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::CPos CPos;
    typedef typename DataTypes::DPos DPos;
    typedef TriangleCollisionModel ParentModel;
    typedef typename DataTypes::Real Real;

    GenericTriangleIterator(ParentModel* model, int index);
    GenericTriangleIterator() {}
    explicit GenericTriangleIterator(const sofa::core::CollisionElementIterator& i);
    GenericTriangleIterator(ParentModel* model, int index, sofa::helper::ReadAccessor<typename DataTypes::VecCoord>& /*x*/);

    CPos p1() const;
    CPos p2() const;
    CPos p3() const;

    CPos p(int i)const;
    CPos p0(int i)const;

    int p1Index() const;
    int p2Index() const;
    int p3Index() const;

    CPos p1Free() const;
    CPos p2Free() const;
    CPos p3Free() const;

    CPos operator[](int i) const;

    DPos v1() const;
    DPos v2() const;
    DPos v3() const;
    DPos v(int i) const;


    DPos n() const;

    /// Return true if the element stores a free position vector
    bool hasFreePosition() const;

    int flags() const;
    bool intersectOnlyBoundaryPoints() const;
    bool intersectOnlyBoundaryEdges() const;
    bool getTrianglePointProjectionRule() const;

    GenericTriangleIterator& shape() { return *this; }
    const GenericTriangleIterator& shape() const { return *this; }

    CPos interpX(sofa::defaulttype::Vec<2, Real> bary) const
    {
        return (p1()*(1 - bary[0] - bary[1])) + (p2()*bary[0]) + (p3()*bary[1]);
    }

    CPos interpX(Real bary0, Real bary1) const
    {
        return (p1()*(1 - bary0 - bary1)) + (p2()*bary0) + (p3()*bary1);
    }

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
inline auto GenericTriangleIterator<TriangleCollisionModel>::p1() const -> CPos { return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[this->model->getTriangles()[this->index][0]]); }
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::p2() const -> CPos { return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[this->model->getTriangles()[this->index][1]]); }
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::p3() const -> CPos { return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[this->model->getTriangles()[this->index][2]]); }
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::p(int i) const -> CPos {
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[this->model->getTriangles()[this->index][i]]);
}
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::p0(int i) const -> CPos {
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::restPosition())->getValue()[this->model->getTriangles()[this->index][i]]);
}
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::operator[](int i) const -> CPos {
    return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue()[this->model->getTriangles()[this->index][i]]);
}

template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::p1Free() const -> CPos { return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[this->model->getTriangles()[this->index][0]]); }
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::p2Free() const -> CPos { return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[this->model->getTriangles()[this->index][1]]); }
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::p3Free() const -> CPos { return DataTypes::getCPos(this->model->getMechanicalState()->read(sofa::core::ConstVecCoordId::freePosition())->getValue()[this->model->getTriangles()[this->index][2]]); }

template<class DataTypes>
inline int GenericTriangleIterator<DataTypes>::p1Index() const { return this->model->getTriangles()[this->index][0]; }
template<class DataTypes>
inline int GenericTriangleIterator<DataTypes>::p2Index() const { return this->model->getTriangles()[this->index][1]; }
template<class DataTypes>
inline int GenericTriangleIterator<DataTypes>::p3Index() const { return this->model->getTriangles()[this->index][2]; }

template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::v1() const -> DPos { return DataTypes::getDPos(this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[this->model->getTriangles()[this->index][0]]); }
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::v2() const -> DPos { return DataTypes::getDPos(this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[this->model->getTriangles()[this->index][1]]); }
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::v3() const -> DPos { return DataTypes::getDPos(this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[this->model->getTriangles()[this->index][2]]); }
template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::v(int i) const -> DPos { return DataTypes::getDPos(this->model->getMechanicalState()->read(sofa::core::ConstVecDerivId::velocity())->getValue()[this->model->getTriangles()[this->index][i]]); }

template<class TriangleCollisionModel>
inline auto GenericTriangleIterator<TriangleCollisionModel>::n() const -> DPos { return this->index < (int)this->model->getNormals().size() ? DataTypes::getDPos(this->model->getNormals()[this->index]) : DPos(); }

template<class DataTypes>
inline int GenericTriangleIterator<DataTypes>::flags() const { return this->model->getTriangleFlags(this->index); }

template<class DataTypes>
inline bool GenericTriangleIterator<DataTypes>::intersectOnlyBoundaryPoints() const { return false; }

template<class DataTypes>
inline bool GenericTriangleIterator<DataTypes>::intersectOnlyBoundaryEdges() const { return false; }

template<class DataTypes>
inline bool GenericTriangleIterator<DataTypes>::getTrianglePointProjectionRule() const { return false; }

template<class DataTypes>
inline bool GenericTriangleIterator<DataTypes>::hasFreePosition() const { return this->model->getMechanicalState()->read(core::ConstVecCoordId::freePosition())->isSet(); }

} // namespace collision
} // namespace component
} // namespace sofa
 
#endif
