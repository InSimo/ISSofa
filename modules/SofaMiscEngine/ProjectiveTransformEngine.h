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
#ifndef SOFA_COMPONENT_ENGINE_PROJECTIVETRANSFORMENGINE_H
#define SOFA_COMPONENT_ENGINE_PROJECTIVETRANSFORMENGINE_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/SofaMisc.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include <sofa/defaulttype/Quat.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace engine
{

/**
 * This class transforms the positions of one DataFields into new positions after applying a transformation
This transformation can be either translation, rotation, scale
 */
template <class DataTypes>
class ProjectiveTransformEngine : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ProjectiveTransformEngine,DataTypes),core::DataEngine);
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef sofa::defaulttype::Mat<3,4,Real> ProjMat;
    typedef sofa::defaulttype::Vec<4,Real> Vec4;
    typedef sofa::defaulttype::Vec<3,Real> Vec3;

protected:
    ProjectiveTransformEngine();

    ~ProjectiveTransformEngine() {}
public:
    void init();

    void reinit();

    void update();

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const ProjectiveTransformEngine<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

protected:
    Data<VecCoord> f_inputX;   // input position
    Data<VecCoord> f_outputX;  // output position: Z=focal_distance
    Data<ProjMat> proj_mat;        // 3x4 projection matrix
    Data<Real> focal_distance; // focal distance i.e. distance between the optical center and the image plane
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_PROJECTIVETRANSFORMENGINE_CPP)

#ifndef SOFA_FLOAT
extern template class SOFA_MISC_ENGINE_API ProjectiveTransformEngine<defaulttype::Vec3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_MISC_ENGINE_API ProjectiveTransformEngine<defaulttype::Vec3fTypes>;
#endif //SOFA_DOUBLE
extern template class SOFA_MISC_ENGINE_API ProjectiveTransformEngine<defaulttype::ExtVec3fTypes>;

#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
