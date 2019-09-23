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
#include <SofaMeshCollision/GenericLineIterator.h>
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
class TLine : public GenericLineIterator<TLineModel<TDataTypes>>
{
public:
    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef TLineModel<DataTypes> ParentModel;

    using GenericLineIterator<TLineModel<TDataTypes>>::GenericLineIterator;
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
    ~TLineModel();

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
    friend class GenericLineIterator<TLineModel<DataTypes>>;

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
inline typename TLineModel<DataTypes>::Deriv TLineModel<DataTypes>::velocity(int index) const { return (this->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[elems[index].p[0]] + this->m_mstate->read(core::ConstVecDerivId::velocity())->getValue()[elems[index].p[1]]) /((core::CollisionModel::Real)(2.0)); }

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
