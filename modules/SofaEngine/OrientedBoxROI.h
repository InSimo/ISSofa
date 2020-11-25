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
#ifndef SOFA_COMPONENT_ENGINE_ORIENTEDBOXROI_H
#define SOFA_COMPONENT_ENGINE_ORIENTEDBOXROI_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif


#include <sofa/core/objectmodel/BaseObject.h>

#include "BaseBoxROI.h"


namespace sofa
{

namespace component
{

namespace engine
{

struct OrientedBoxROIType
{
    typedef sofa::defaulttype::Vector3 Vec3;
    typedef sofa::helper::Quater<double> Quater;

    OrientedBoxROIType()
    {
        center = sofa::defaulttype::Vector3(0, 0, 0);
        dimensions = sofa::defaulttype::Vector3(1, 1, 1);
        quat = sofa::helper::Quater<double>(0, 0, 0, 1);
    }

    OrientedBoxROIType(Vec3 pos, Vec3 dims, Quater quater)
        : center(pos), dimensions(dims), quat(quater)
    {}

    Vec3 center;
    Vec3 dimensions;
    Quater quat;

    inline friend std::ostream& operator<< (std::ostream& os, const OrientedBoxROIType& b)
    {
        os << b.center << " ";
        os << b.dimensions << " ";
        os << b.quat << " ";
        return os;
    }

    inline friend std::istream& operator>> (std::istream& in, OrientedBoxROIType& b)
    {
        in >> b.center;
        in >> b.dimensions;
        in >> b.quat;
        return in;
    }
};

/**
 * This class find all the points/edges/triangles/tetrahedra located inside given boxes
 * given by 3 packed parameters : a center, 3 width and a quaternion
 */
template <class DataTypes>
class OrientedBoxROI : public BaseBoxROI<DataTypes, OrientedBoxROIType>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(OrientedBoxROI,DataTypes), SOFA_TEMPLATE2(BaseBoxROI,DataTypes,OrientedBoxROIType));

    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::CPos CPos;
    typedef sofa::defaulttype::Vector3 Vec3;
    typedef typename sofa::helper::Quater<Real> Quat;

    typedef typename Inherit1::Boxes Boxes;


protected:
    OrientedBoxROI();

    ~OrientedBoxROI() {}

public:
    virtual void init() override;

    virtual void updateBoxes() override;

    virtual void computeBBox(const core::ExecParams*  params ) override;

    virtual void drawBoxes(const core::visual::VisualParams* vparams) override;

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        if (!arg->getAttribute("template"))
        {
            // only check if this template is correct if no template was given
            if (context->getMechanicalState() && sofa::core::behavior::MechanicalState<DataTypes>::DynamicCast(context->getMechanicalState()) == NULL)
                return false; // this template is not the same as the existing MechanicalState
        }
        return core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

    /// Construction method called by ObjectFactory.
    template<class T>
    static typename T::SPtr create(T* tObj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        return core::objectmodel::BaseObject::create(tObj, context, arg);
    }

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const OrientedBoxROI<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

protected:
    virtual bool isPointInBox(const CPos& p, const OrientedBoxROIType& b) override;

public:
    sofa::Data< sofa::helper::vector<Vec3> > d_centers;
    sofa::Data< sofa::helper::vector<Vec3>> d_dimensions;
    sofa::Data< sofa::helper::vector<Quat>> d_quaternions;
};


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_ORIENTEDBOXROI_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_ENGINE_API OrientedBoxROI<defaulttype::Vec3dTypes>;
extern template class SOFA_ENGINE_API OrientedBoxROI<defaulttype::Rigid3dTypes>;
extern template class SOFA_ENGINE_API OrientedBoxROI<defaulttype::Vec6dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_ENGINE_API OrientedBoxROI<defaulttype::Vec3fTypes>;
extern template class SOFA_ENGINE_API OrientedBoxROI<defaulttype::Rigid3fTypes>;
extern template class SOFA_ENGINE_API OrientedBoxROI<defaulttype::Vec6fTypes>;
#endif //SOFA_DOUBLE
#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
