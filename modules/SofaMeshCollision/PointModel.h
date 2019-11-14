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
#ifndef SOFA_COMPONENT_COLLISION_POINTMODEL_H
#define SOFA_COMPONENT_COLLISION_POINTMODEL_H

#include <SofaMeshCollision/GenericPointModel.h>
#include <SofaMeshCollision/GenericPointIterator.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/collision/CollisionModelTraits.h>

namespace sofa
{

namespace component
{

namespace collision
{

template<class DataTypes>
class TPointModel;

class PointLocalMinDistanceFilter;

template<class TDataTypes>
class TPoint : public GenericPointIterator<TPointModel<TDataTypes>>
{
public:
    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef TPointModel<DataTypes> ParentModel;

    using GenericPointIterator<TPointModel<TDataTypes>>::GenericPointIterator;

    bool activated(sofa::core::CollisionModel *cm = nullptr) const { return this->model->m_myActiver->activePoint(this->index, cm); }
};

class PointActiver
{
public:
    PointActiver() {}
    virtual ~PointActiver() {}
    virtual bool activePoint(int /*index*/, core::CollisionModel* = nullptr) { return true; }
    static PointActiver* getDefaultActiver() { static PointActiver defaultActiver; return &defaultActiver; }
};

template<class TDataTypes>
class TPointModel : public GenericPointModel<TPointModel<TDataTypes>, TDataTypes>
{
public:
    SOFA_CLASS_UNIQUE((TPointModel<TDataTypes>), ((GenericPointModel<TPointModel<TDataTypes>, TDataTypes>)));

    typedef TDataTypes DataTypes;
    typedef DataTypes InDataTypes;
    typedef TPointModel<DataTypes> ParentModel;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::CPos CPos;
    typedef typename DataTypes::DPos DPos;
    typedef typename DataTypes::Deriv Deriv;
    typedef TPoint<DataTypes> Element;
    typedef helper::vector<unsigned int> VecIndex;

    friend class TPoint<DataTypes>;
    friend class GenericPointIterator<TPointModel<DataTypes>>;

protected:
    TPointModel();
public:
    void init() override;

    void draw(const core::visual::VisualParams*, int index);
    void draw(const core::visual::VisualParams* vparams) override;

    DPos getNormal(int index) const { return !m_normals.empty() ? m_normals[index] : DPos(); }

    PointLocalMinDistanceFilter *getFilter() const { return m_lmdFilter; }
    void setFilter(PointLocalMinDistanceFilter *lmdFilter) { m_lmdFilter = lmdFilter; }


    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        if (core::behavior::MechanicalState<DataTypes>::DynamicCast(context->getMechanicalState()) == nullptr)
            return false;
        return core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const TPointModel<DataTypes>* = nullptr)
    {
        return DataTypes::Name();
    }

protected:
    void updateNormals();

    Data<bool> d_computeNormals;
    Data<bool> d_displayFreePosition;
    Data<std::string> d_pointActiverPath;

    sofa::helper::vector<DPos> m_normals;
    PointLocalMinDistanceFilter *m_lmdFilter = nullptr;
    PointActiver *m_myActiver = nullptr;
};

typedef TPointModel<sofa::defaulttype::Vec3Types> PointModel;
typedef TPoint<sofa::defaulttype::Vec3Types> Point;

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_MESH_COLLISION)
#ifndef SOFA_FLOAT
extern template class SOFA_MESH_COLLISION_API TPointModel<defaulttype::Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_MESH_COLLISION_API TPointModel<defaulttype::Vec3fTypes>;
#endif
#endif

} // namespace collision

} // namespace component

} // namespace sofa
template < class DataTypes >
struct sofa::core::collision::CollisionModelTraits< sofa::component::collision::TPointModel<DataTypes> >
    : public CollisionPointModelTraits<DataTypes>
{
};

template < class DataTypes >
struct sofa::core::collision::InterpX< sofa::component::collision::TPointModel< DataTypes > >
{
    typedef sofa::component::collision::TPointModel< DataTypes>        CollisionModel;
    typedef typename CollisionModelTraits<CollisionModel>::Coord       Coord;
    typedef typename CollisionModelTraits<CollisionModel>::BaryCoord   BaryCoord;
    typedef typename CollisionModel::Element                           Element;

    static Coord eval(const Element& e, const BaryCoord&)
    {
        return e.p();
    }
};

template < class DataTypes >
struct sofa::core::collision::InterpV< sofa::component::collision::TPointModel< DataTypes > >
{
    typedef sofa::component::collision::TPointModel< DataTypes>        CollisionModel;
    typedef typename CollisionModelTraits<CollisionModel>::Deriv       Deriv;
    typedef typename CollisionModelTraits<CollisionModel>::BaryCoord   BaryCoord;
    typedef typename CollisionModel::Element                           Element;

    static Deriv eval(const Element& e, const BaryCoord&)
    {
        return e.v();
    }
};
#endif
