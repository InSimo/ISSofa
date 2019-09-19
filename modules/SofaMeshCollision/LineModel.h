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
#ifndef SOFA_COMPONENT_COLLISION_LINEMODEL_H
#define SOFA_COMPONENT_COLLISION_LINEMODEL_H

#include <sofa/core/CollisionModel.h>
#include <SofaMeshCollision/GenericLineModel.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <SofaMeshCollision/LocalMinDistanceFilter.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <SofaMeshCollision/PointModel.h>

namespace sofa
{

namespace component
{

namespace collision
{

template<class DataTypes>
class TLineModel;

class LineLocalMinDistanceFilter;

template<class TDataTypes>
class TLine : public core::TCollisionElementIterator<TLineModel<TDataTypes> >
{
public:
    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef TLineModel<DataTypes> ParentModel;

    TLine(ParentModel* model, int index);
    TLine() {}

    explicit TLine(const core::CollisionElementIterator& i);

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

    /// Return true if the element stores a free position vector
    bool hasFreePosition() const;

    bool activated(core::CollisionModel *cm = 0) const;

    unsigned int getClassificationSampling() const { return this->model->getClassificationSampling(this->index); }

    // Return respectively the Vertex composing the neighbor Rigt and Left Triangle
//	const Vector3* tRight() const;
//	const Vector3* tLeft() const;
};

class LineActiver
{
public:
    LineActiver() {}
    virtual ~LineActiver() {}
    virtual bool activeLine(int /*index*/, core::CollisionModel * /*cm*/ = 0) {return true;}
	static LineActiver* getDefaultActiver() { static LineActiver defaultActiver; return &defaultActiver; }
};

template<class TDataTypes>
class TLineModel : public GenericLineModel<TLineModel<TDataTypes>, TDataTypes>
{
public :
    SOFA_CLASS_UNIQUE((TLineModel<TDataTypes>), ((GenericLineModel<TLineModel<TDataTypes>, TDataTypes>)));
    
protected:
    struct LineData
    {
        int p[2];
        // Triangles neighborhood
//		int tRight, tLeft;
    };

    sofa::helper::vector<LineData> elems;
    TLineModel();

public:
    typedef TDataTypes DataTypes;
    typedef DataTypes InDataTypes;
    typedef TLineModel<DataTypes> ParentModel;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef TLine<DataTypes> Element;
    friend class TLine<DataTypes>;

    virtual void init() override;

    // -- CollisionModel interface

    virtual void resize(int size) override;

    defaulttype::BoundingBox computeElementBBox(int index, SReal distance);
    defaulttype::BoundingBox computeElementBBox(int index, SReal distance, double dt);

    virtual void computeBoundingTree(int maxDepth=0) override;

    void draw(const core::visual::VisualParams*,int index) override;

    void draw(const core::visual::VisualParams* vparams) override;

    bool canCollideWithElement(int index, core::CollisionModel* model2, int index2) override;

    void updateTopologicalLineFlags() override;

    Deriv velocity(int index) const;

    LineLocalMinDistanceFilter *getFilter() const;

    virtual int getElemEdgeIndex(int index) const { return index; }
    
    void setFilter(LineLocalMinDistanceFilter * /*lmdFilter*/);


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

    static std::string templateName(const TLineModel<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

    unsigned int getClassificationSampling(const unsigned int index) const;

protected:

    PointModel* mpoints;
    LineLocalMinDistanceFilter *m_lmdFilter;

    Data< std::string  > LineActiverPath;
    Data< bool > m_displayFreePosition;
    Data< sofa::helper::vector<unsigned int> > d_classificationSampling;

    LineActiver *myActiver;
};

template<class DataTypes>
inline TLine<DataTypes>::TLine(ParentModel* model, int index)
    : core::TCollisionElementIterator<ParentModel>(model, index)
{
//	activated = model->myActiver->activeLine(index);
}

template<class DataTypes>
inline TLine<DataTypes>::TLine(const core::CollisionElementIterator& i)
    : core::TCollisionElementIterator<ParentModel>(static_cast<ParentModel*>(i.getCollisionModel()), i.getIndex())
{
//	LineModel* CM = static_cast<LineModel*>(i.getCollisionModel());
//	activated = CM->myActiver->activeLine(i.getIndex());
}

template<class DataTypes>
inline unsigned TLine<DataTypes>::i1() const { return this->model->elems[this->index].p[0]; }

template<class DataTypes>
inline unsigned TLine<DataTypes>::i2() const { return this->model->elems[this->index].p[1]; }

template<class DataTypes>
inline const typename DataTypes::Coord& TLine<DataTypes>::p1() const { return this->model->m_mstate->read(core::ConstVecCoordId::position())->getValue()[this->model->elems[this->index].p[0]]; }

template<class DataTypes>
inline const typename DataTypes::Coord& TLine<DataTypes>::p2() const { return this->model->m_mstate->read(core::ConstVecCoordId::position())->getValue()[this->model->elems[this->index].p[1]]; }

template<class DataTypes>
inline const typename DataTypes::Coord& TLine<DataTypes>::p(int i) const {
    return this->model->m_mstate->read(core::ConstVecCoordId::position())->getValue()[this->model->elems[this->index].p[i]];
}

template<class DataTypes>
inline const typename DataTypes::Coord& TLine<DataTypes>::p0(int i) const {
    return this->model->m_mstate->read(core::ConstVecCoordId::restPosition())->getValue()[this->model->elems[this->index].p[i]];
}

template<class DataTypes>
inline const typename DataTypes::Coord& TLine<DataTypes>::p1Free() const
{
    if (hasFreePosition())
        return this->model->m_mstate->read(core::ConstVecCoordId::freePosition())->getValue()[this->model->elems[this->index].p[0]];
    else
        return p1();
}

template<class DataTypes>
inline const typename DataTypes::Coord& TLine<DataTypes>::p2Free() const
{
    if (hasFreePosition())
        return this->model->m_mstate->read(core::ConstVecCoordId::freePosition())->getValue()[this->model->elems[this->index].p[1]];
    else
        return p2();
}

template<class DataTypes>
inline const typename DataTypes::Deriv& TLine<DataTypes>::v1() const { return this->model->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[this->model->elems[this->index].p[0]]; }

template<class DataTypes>
inline const typename DataTypes::Deriv& TLine<DataTypes>::v2() const { return this->model->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[this->model->elems[this->index].p[1]]; }


template<class DataTypes>
inline typename TLineModel<DataTypes>::Deriv TLineModel<DataTypes>::velocity(int index) const { return (this->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[elems[index].p[0]] + this->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[elems[index].p[1]]) /((core::CollisionModel::Real)(2.0)); }

template<class DataTypes>
inline int TLine<DataTypes>::flags() const { return this->model->getLineFlags(this->index); }

template<class DataTypes>
inline bool TLine<DataTypes>::hasFreePosition() const { return this->model->m_mstate->read(core::ConstVecCoordId::freePosition())->isSet(); }

template<class DataTypes>
inline bool TLine<DataTypes>::activated(core::CollisionModel *cm) const
{
    return this->model->myActiver->activeLine(this->index, cm);
}

typedef TLineModel<sofa::defaulttype::Vec3Types> LineModel;
typedef TLine<sofa::defaulttype::Vec3Types> Line;

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_MESH_COLLISION)
#ifndef SOFA_FLOAT
extern template class SOFA_MESH_COLLISION_API TLineModel<defaulttype::Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_MESH_COLLISION_API TLineModel<defaulttype::Vec3fTypes>;
#endif
#endif

} // namespace collision

} // namespace component

} // namespace sofa

template < class DataTypes >
struct sofa::core::collision::CollisionModelTraits< sofa::component::collision::TLineModel<DataTypes> >
    : public CollisionLineModelTraits<DataTypes>
{
};

template < class DataTypes >
struct sofa::core::collision::InterpX< sofa::component::collision::TLineModel< DataTypes> >
{
    typedef sofa::component::collision::TLineModel< DataTypes>         CollisionModel;
    typedef typename CollisionModelTraits<CollisionModel>::Coord       Coord;
    typedef typename CollisionModelTraits<CollisionModel>::BaryCoord   BaryCoord;
    typedef typename CollisionModel::Element                           Element;

    static Coord eval(const Element& e, const BaryCoord& baryCoords)
    {
        Coord interp = e.p1()*(1 - baryCoords[0]) + e.p2()*baryCoords[0];
        return interp;
    }

};

template < class DataTypes >
struct sofa::core::collision::InterpV< sofa::component::collision::TLineModel< DataTypes > >
{
    typedef sofa::component::collision::TLineModel< DataTypes>       CollisionModel;
    typedef typename CollisionModelTraits<CollisionModel>::Deriv     Deriv;
    typedef typename CollisionModelTraits<CollisionModel>::BaryCoord BaryCoord;
    typedef typename CollisionModel::Element Element;

    static Deriv eval(const Element& e, const BaryCoord& baryCoords)
    {
        Deriv interp = e.v1()*(1 - baryCoords[0]) + e.v2()*baryCoords[0];
        return interp;
    }
};

#endif
