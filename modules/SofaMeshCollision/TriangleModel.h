/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
#ifndef SOFA_COMPONENT_COLLISION_TRIANGLEMODEL_H
#define SOFA_COMPONENT_COLLISION_TRIANGLEMODEL_H

#include <sofa/core/CollisionModel.h>
#include <SofaMeshCollision/GenericTriangleModel.h>
#include <SofaMeshCollision/GenericTriangleIterator.h>
#include <SofaMeshCollision/LocalMinDistanceFilter.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaBaseTopology/TopologyData.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaMeshCollision/PointModel.h>
#include <map>

namespace sofa
{

namespace component
{

namespace collision
{

template<class DataTypes>
class TTriangleModel;

class TriangleLocalMinDistanceFilter;

template<class TDataTypes>
class TTriangle : public sofa::component::collision::GenericTriangleIterator< TTriangleModel<TDataTypes> >
{
public:
    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef TTriangleModel<DataTypes> ParentModel;
	typedef typename DataTypes::Real Real;

    TTriangle(ParentModel* model, int index);
    TTriangle() {}
    explicit TTriangle(const core::CollisionElementIterator& i);
	TTriangle(ParentModel* model, int index, helper::ReadAccessor<typename DataTypes::VecCoord>& /*x*/);

    const Coord& p1() const;
    const Coord& p2() const;
    const Coord& p3() const;

    const Coord& p(int i)const;
	const Coord& p0(int i)const;

    int p1Index() const;
    int p2Index() const;
    int p3Index() const;

    const Coord& p1Free() const;
    const Coord& p2Free() const;
    const Coord& p3Free() const;

    const Coord& operator[](int i) const;

    const Deriv& v1() const;
    const Deriv& v2() const;
    const Deriv& v3() const;
    const Deriv& v(int i) const;


    const Deriv& n() const;
    Deriv& n();

    /// Return true if the element stores a free position vector
    bool hasFreePosition() const;

    int flags() const;

	TTriangle& shape() { return *this; }
    const TTriangle& shape() const { return *this; }

    Coord interpX(defaulttype::Vec<2,Real> bary) const
    {
		return (p1()*(1-bary[0]-bary[1])) + (p2()*bary[0]) + (p3()*bary[1]);
	}
};

template<class TDataTypes>
class TTriangleModel : public component::collision::GenericTriangleModel<TDataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TTriangleModel, TDataTypes), SOFA_TEMPLATE(component::collision::GenericTriangleModel, TDataTypes));

    typedef TDataTypes DataTypes;
    typedef DataTypes InDataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef TTriangle<DataTypes> Element;
    friend class TTriangle<DataTypes>;
    friend class GenericTriangleIterator<TTriangleModel<DataTypes>>;

    enum TriangleFlag
    {
        FLAG_P1 = 1 << 0,  ///< Point 1  is attached to this triangle
        FLAG_P2 = 1 << 1,  ///< Point 2  is attached to this triangle
        FLAG_P3 = 1 << 2,  ///< Point 3  is attached to this triangle
        FLAG_BP1 = 1 << 3,  ///< Point 1  is attached to this triangle and is a boundary
        FLAG_BP2 = 1 << 4,  ///< Point 2  is attached to this triangle and is a boundary
        FLAG_BP3 = 1 << 5,  ///< Point 3  is attached to this triangle and is a boundary
        FLAG_E23 = 1 << 6,  ///< Edge 2-3 is attached to this triangle
        FLAG_E31 = 1 << 7,  ///< Edge 3-1 is attached to this triangle
        FLAG_E12 = 1 << 8,  ///< Edge 1-2 is attached to this triangle
        FLAG_BE23 = 1 << 9,  ///< Edge 2-3 is attached to this triangle and is a boundary
        FLAG_BE31 = 1 << 10, ///< Edge 3-1 is attached to this triangle and is a boundary
        FLAG_BE12 = 1 << 11, ///< Edge 1-2 is attached to this triangle and is a boundary
        FLAG_BADSHAPE = 1 << 12, ///< Triangle has a bad shape and should be ignored
        FLAG_POINTS = FLAG_P1 | FLAG_P2 | FLAG_P3,
        FLAG_EDGES = FLAG_E12 | FLAG_E23 | FLAG_E31,
        FLAG_BPOINTS = FLAG_BP1 | FLAG_BP2 | FLAG_BP3,
        FLAG_BEDGES = FLAG_BE12 | FLAG_BE23 | FLAG_BE31,
    };

	enum { NBARY = 2 };

protected:
#if 0
        /// Output stream
        inline friend std::ostream& operator<< (std::ostream& os, const TriangleInfo& ti)
        {
            return os << ti.normal;
        }

        /// Input stream
        inline friend std::istream& operator >> (std::istream& in, TriangleInfo& ti)
        {
            return in >> ti.normal;
        }
    };
#endif

    //topology::TriangleData<TriangleInfo> elems;
    VecDeriv normals;

    const sofa::core::topology::BaseMeshTopology::SeqTriangles* triangles = nullptr;

    sofa::core::topology::BaseMeshTopology::SeqTriangles mytriangles;

    bool needsUpdate = false;
    virtual void updateFromTopology();
    virtual void updateNormals();
    virtual void updateFlags();
    int getTriangleFlags(int i) const;

    core::behavior::MechanicalState<DataTypes>* mstate = nullptr;
    Data<bool> computeNormals;
    Data<Real> d_boundaryAngleThreshold;
    Data<Real> d_minTriangleArea;
    Data<bool> d_drawBoundaryPoints;
    Data<bool> d_drawBoundaryEdges;
    int meshRevision = -1;
    sofa::helper::vector<int> triangleFlags;

    sofa::core::topology::BaseMeshTopology* _topology = nullptr;

    PointModel* mpoints = nullptr;

    TriangleLocalMinDistanceFilter *m_lmdFilter = nullptr;

    int m_countBadShape = 0; ///< how many triangles were below the minTriangleArea threshold


protected:

    TTriangleModel();
public:
    virtual void init() override;

    virtual void reinit() override;

    // -- CollisionModel interface

    virtual void resize(int size);

    virtual void computeBoundingTree(int maxDepth=0) override;

    virtual void computeContinuousBoundingTree(double dt, int maxDepth=0) override;

    void draw(const core::visual::VisualParams*, int index);

    void draw(const core::visual::VisualParams* vparams) override;
     
    virtual bool canCollideWithElement(int index, core::CollisionModel* model2, int index2) override;

    virtual void handleTopologyChange() override;

    core::behavior::MechanicalState<DataTypes>* getMechanicalState() { return mstate; }
    const core::behavior::MechanicalState<DataTypes>* getMechanicalState() const { return mstate; }

    const VecCoord& getX() const { return(getMechanicalState()->read(core::ConstVecCoordId::position())->getValue()); }
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& getTriangles() const { return *triangles; }
    const VecDeriv& getNormals() const { return normals; }
    const Deriv& getNormal(unsigned int i) const { return normals[i]; }

    TriangleLocalMinDistanceFilter *getFilter() const;

    //template< class TFilter >
    //TFilter *getFilter() const
    //{
    //	if (m_lmdFilter != 0)
    //		return m_lmdFilter;
    //	else
    //		return &m_emptyFilter;
    //}

    void setFilter(TriangleLocalMinDistanceFilter * /*lmdFilter*/);

    Deriv velocity(int index)const;


    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        if (core::behavior::MechanicalState<DataTypes>::DynamicCast(context->getMechanicalState()) == NULL)
            return false;
        return core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const TTriangleModel<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }
};

template<class DataTypes>
inline TTriangle<DataTypes>::TTriangle(ParentModel* model, int index)
    : sofa::component::collision::GenericTriangleIterator<ParentModel>(model, index)
{}

template<class DataTypes>
inline TTriangle<DataTypes>::TTriangle(const core::CollisionElementIterator& i)
    : sofa::component::collision::GenericTriangleIterator<ParentModel>(static_cast<ParentModel*>(i.getCollisionModel()), i.getIndex())
{}

template<class DataTypes>
inline TTriangle<DataTypes>::TTriangle(ParentModel* model, int index, helper::ReadAccessor<typename DataTypes::VecCoord>& /*x*/)
    : sofa::component::collision::GenericTriangleIterator<ParentModel>(model, index)
{}

template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p1() const { return this->model->mstate->read(core::ConstVecCoordId::position())->getValue()[(*(this->model->triangles))[this->index][0]]; }
template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p2() const { return this->model->mstate->read(core::ConstVecCoordId::position())->getValue()[(*(this->model->triangles))[this->index][1]]; }
template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p3() const { return this->model->mstate->read(core::ConstVecCoordId::position())->getValue()[(*(this->model->triangles))[this->index][2]]; }
template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p(int i) const {
    return this->model->mstate->read(core::ConstVecCoordId::position())->getValue()[(*(this->model->triangles))[this->index][i]];
}
template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p0(int i) const {
    return this->model->mstate->read(core::ConstVecCoordId::restPosition())->getValue()[(*(this->model->triangles))[this->index][i]];
}
template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::operator[](int i) const {
    return this->model->mstate->read(core::ConstVecCoordId::position())->getValue()[(*(this->model->triangles))[this->index][i]];
}

template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p1Free() const { return (this->model->mstate->read(core::ConstVecCoordId::freePosition())->getValue())[(*(this->model->triangles))[this->index][0]]; }
template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p2Free() const { return (this->model->mstate->read(core::ConstVecCoordId::freePosition())->getValue())[(*(this->model->triangles))[this->index][1]]; }
template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p3Free() const { return (this->model->mstate->read(core::ConstVecCoordId::freePosition())->getValue())[(*(this->model->triangles))[this->index][2]]; }

template<class DataTypes>
inline int TTriangle<DataTypes>::p1Index() const { return (*(this->model->triangles))[this->index][0]; }
template<class DataTypes>
inline int TTriangle<DataTypes>::p2Index() const { return (*(this->model->triangles))[this->index][1]; }
template<class DataTypes>
inline int TTriangle<DataTypes>::p3Index() const { return (*(this->model->triangles))[this->index][2]; }

template<class DataTypes>
inline const typename DataTypes::Deriv& TTriangle<DataTypes>::v1() const { return (this->model->mstate->read(core::ConstVecDerivId::velocity())->getValue())[(*(this->model->triangles))[this->index][0]]; }
template<class DataTypes>
inline const typename DataTypes::Deriv& TTriangle<DataTypes>::v2() const { return this->model->mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(this->model->triangles))[this->index][1]]; }
template<class DataTypes>
inline const typename DataTypes::Deriv& TTriangle<DataTypes>::v3() const { return this->model->mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(this->model->triangles))[this->index][2]]; }
template<class DataTypes>
inline const typename DataTypes::Deriv& TTriangle<DataTypes>::v(int i) const { return this->model->mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(this->model->triangles))[this->index][i]]; }

template<class DataTypes>
inline const typename DataTypes::Deriv& TTriangle<DataTypes>::n() const { return this->model->normals[this->index]; }
template<class DataTypes>
inline       typename DataTypes::Deriv& TTriangle<DataTypes>::n()       { return this->model->normals[this->index]; }

template<class DataTypes>
inline int TTriangle<DataTypes>::flags() const { return this->model->getTriangleFlags(this->index); }

template<class DataTypes>
inline bool TTriangle<DataTypes>::hasFreePosition() const { return this->model->mstate->read(core::ConstVecCoordId::freePosition())->isSet(); }

template<class DataTypes>
inline typename DataTypes::Deriv TTriangleModel<DataTypes>::velocity(int index) const
{
    Deriv velocity;
    DataTypes::setDPos(velocity, DataTypes::getDPos( mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(triangles))[index][0]] + mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(triangles))[index][1]] +
            mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(triangles))[index][2]] ) / 3.0);
    return velocity;
}
typedef TTriangleModel<sofa::defaulttype::Vec3Types> TriangleModel;
typedef TTriangle<sofa::defaulttype::Vec3Types> Triangle;

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_MESH_COLLISION)
#ifndef SOFA_FLOAT
extern template class SOFA_MESH_COLLISION_API TTriangleModel<defaulttype::Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_MESH_COLLISION_API TTriangleModel<defaulttype::Vec3fTypes>;
#endif
#endif

} // namespace collision

} // namespace component

} // namespace sofa


template< class DataTypes>
struct sofa::core::collision::CollisionModelTraits< sofa::component::collision::TTriangleModel<DataTypes> >
    : public CollisionTriangleModelTraits<DataTypes>
{
};


template < class DataTypes >
struct sofa::core::collision::InterpX< sofa::component::collision::TTriangleModel< DataTypes> >
{
    typedef sofa::component::collision::TTriangleModel< DataTypes>   CollisionModel;
    typedef typename CollisionModelTraits<CollisionModel>::Coord     Coord;
    typedef typename CollisionModelTraits<CollisionModel>::BaryCoord BaryCoord;
    typedef typename CollisionModel::Element                         Element;

    static Coord eval(const Element& e, const BaryCoord& baryCoords)
    {
        Coord interp = e.p1()*(1 - baryCoords[0] - baryCoords[1]) + e.p2()*baryCoords[0] + e.p3()*baryCoords[1];
        return interp;
    }

};

template < class DataTypes >
struct sofa::core::collision::InterpV< sofa::component::collision::TTriangleModel< DataTypes > >
{
    typedef sofa::component::collision::TTriangleModel< DataTypes>   CollisionModel;
    typedef typename CollisionModelTraits<CollisionModel>::Deriv     Deriv;
    typedef typename CollisionModelTraits<CollisionModel>::BaryCoord BaryCoord;
    typedef typename CollisionModel::Element                         Element;

    static Deriv eval(const Element& e, const BaryCoord& baryCoords)
    {
        Deriv interp = e.v1()*(1 - baryCoords[0] - baryCoords[1]) + e.v2()*baryCoords[0] + e.v3()*baryCoords[1];
        return interp;
    }
};
#endif
