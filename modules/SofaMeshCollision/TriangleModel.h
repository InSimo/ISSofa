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

    using GenericTriangleIterator<TTriangleModel<TDataTypes>>::GenericTriangleIterator;

    const Coord& p1() const;
    const Coord& p2() const;
    const Coord& p3() const;

    const Deriv& v1() const;
    const Deriv& v2() const;
    const Deriv& v3() const;

    const Deriv& n() const;
    Deriv& n();

	TTriangle& shape() { return *this; }
    const TTriangle& shape() const { return *this; }

    Coord interpX(defaulttype::Vec<2,Real> bary) const
    {
        return (this->p1()*(1-bary[0]-bary[1])) + (this->p2()*bary[0]) + (this->p3()*bary[1]);
	}
};

template<class TDataTypes>
class TTriangleModel : public component::collision::GenericTriangleModel<TTriangleModel<TDataTypes>, TDataTypes>
{
public:
    SOFA_CLASS_UNIQUE((TTriangleModel<TDataTypes>), ((component::collision::GenericTriangleModel<TTriangleModel<TDataTypes>, TDataTypes>)));

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
        FLAG_BADSHAPE = Inherit1::FLAG_FIRST_CUSTOM << 0, ///< Triangle has a bad shape and should be ignored
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

    Data<bool> computeNormals;
    Data<Real> d_minTriangleArea;
    Data<bool> d_drawBoundaryPoints;
    Data<bool> d_drawBoundaryEdges;
    int meshRevision = -1;

    TriangleLocalMinDistanceFilter *m_lmdFilter = nullptr;

    int m_countBadShape = 0; ///< how many triangles were below the minTriangleArea threshold


protected:

    TTriangleModel();
    ~TTriangleModel();

public:
    virtual void init() override;

    virtual void reinit() override;

    // -- CollisionModel interface

    virtual void resize(int size);

    defaulttype::BoundingBox computeElementBBox(int index, SReal distance);
    defaulttype::BoundingBox computeElementBBox(int index, SReal distance, double dt);

    virtual void computeBoundingTree(int maxDepth=0) override;
    virtual void computeContinuousBoundingTree(double dt, int maxDepth=0) override;

    void draw(const core::visual::VisualParams*, int index);

    void draw(const core::visual::VisualParams* vparams) override;
     
    virtual void handleTopologyChange(sofa::core::topology::Topology* t) override;

    const VecCoord& getX() const { return this->getMechanicalState()->read(core::ConstVecCoordId::position())->getValue(); }
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& getTriangles() const { return *triangles; }
    const VecDeriv& getNormals() const { return normals; }
    const Deriv& getNormal(unsigned int i) const { return normals[i]; }

    TriangleLocalMinDistanceFilter *getFilter() const;

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
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p1() const { return this->model->m_mstate->read(core::ConstVecCoordId::position())->getValue()[(*(this->model->triangles))[this->index][0]]; }
template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p2() const { return this->model->m_mstate->read(core::ConstVecCoordId::position())->getValue()[(*(this->model->triangles))[this->index][1]]; }
template<class DataTypes>
inline const typename DataTypes::Coord& TTriangle<DataTypes>::p3() const { return this->model->m_mstate->read(core::ConstVecCoordId::position())->getValue()[(*(this->model->triangles))[this->index][2]]; }

template<class DataTypes>
inline const typename DataTypes::Deriv& TTriangle<DataTypes>::v1() const { return (this->model->m_mstate->read(core::ConstVecDerivId::velocity())->getValue())[(*(this->model->triangles))[this->index][0]]; }
template<class DataTypes>
inline const typename DataTypes::Deriv& TTriangle<DataTypes>::v2() const { return this->model->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(this->model->triangles))[this->index][1]]; }
template<class DataTypes>
inline const typename DataTypes::Deriv& TTriangle<DataTypes>::v3() const { return this->model->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(this->model->triangles))[this->index][2]]; }

template<class DataTypes>
inline const typename DataTypes::Deriv& TTriangle<DataTypes>::n() const { return this->model->normals[this->index]; }
template<class DataTypes>
inline typename DataTypes::Deriv& TTriangle<DataTypes>::n() { return this->model->normals[this->index]; }

template<class DataTypes>
inline typename DataTypes::Deriv TTriangleModel<DataTypes>::velocity(int index) const
{
    Deriv velocity;
    DataTypes::setDPos(velocity, DataTypes::getDPos(
            this->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(triangles))[index][0]] +
            this->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(triangles))[index][1]] +
            this->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[(*(triangles))[index][2]] ) / 3.0);
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
