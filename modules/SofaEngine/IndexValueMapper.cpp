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
#define INDEXVALUEMAPPER_CPP_

#include "IndexValueMapper.inl"
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace engine
{

using namespace sofa;
using namespace sofa::defaulttype;

SOFA_DECL_CLASS(IndexValueMapper)

int IndexValueMapperClass = core::RegisterObject("?")
#if defined(SOFA_DOUBLE)
    .add< IndexValueMapper< helper::vector<double> > >(true)
#elif defined(SOFA_FLOAT)
    .add< IndexValueMapper< helper::vector<float> > >(true)
#else
    .add< IndexValueMapper< helper::vector<double> > >(true)
    .add< IndexValueMapper< helper::vector<float> > >()
#endif
    .add< IndexValueMapper< helper::vector<int> > >()
    .add< IndexValueMapper< helper::vector<bool> > >()
    //.add< IndexValueMapper< helper::vector<std::string> > >()
#ifndef SOFA_FLOAT
    .add< IndexValueMapper< helper::vector<defaulttype::Vec2d> > >()
    .add< IndexValueMapper< helper::vector<defaulttype::Vec3d> > >()
    .add< IndexValueMapper< defaulttype::Rigid2dTypes::VecCoord > >()
    .add< IndexValueMapper< defaulttype::Rigid2dTypes::VecDeriv > >()
    .add< IndexValueMapper< defaulttype::Rigid3dTypes::VecCoord > >()
    .add< IndexValueMapper< defaulttype::Rigid3dTypes::VecDeriv > >()
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
    .add< IndexValueMapper< helper::vector<defaulttype::Vec2f> > >()
    .add< IndexValueMapper< helper::vector<defaulttype::Vec3f> > >()
    .add< IndexValueMapper< defaulttype::Rigid2fTypes::VecCoord > >()
    .add< IndexValueMapper< defaulttype::Rigid2fTypes::VecDeriv > >()
    .add< IndexValueMapper< defaulttype::Rigid3fTypes::VecCoord > >()
    .add< IndexValueMapper< defaulttype::Rigid3fTypes::VecDeriv > >()
#endif //SOFA_DOUBLE
        ;

template class SOFA_ENGINE_API IndexValueMapper< helper::vector<int> >;
template class SOFA_ENGINE_API IndexValueMapper< helper::vector<bool> >;
//template class SOFA_ENGINE_API IndexValueMapper< helper::vector<std::string> >;

#ifndef SOFA_FLOAT
template class SOFA_ENGINE_API IndexValueMapper< helper::vector<double> >;
template class SOFA_ENGINE_API IndexValueMapper< helper::vector<defaulttype::Vec2d> >;
template class SOFA_ENGINE_API IndexValueMapper< helper::vector<defaulttype::Vec3d> >;
template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid2dTypes::VecCoord >;
template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid2dTypes::VecDeriv >;
template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid3dTypes::VecCoord >;
template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid3dTypes::VecDeriv >;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
template class SOFA_ENGINE_API IndexValueMapper< helper::vector<float> >;
template class SOFA_ENGINE_API IndexValueMapper< helper::vector<defaulttype::Vec2f> >;
template class SOFA_ENGINE_API IndexValueMapper< helper::vector<defaulttype::Vec3f> >;
template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid2fTypes::VecCoord >;
template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid2fTypes::VecDeriv >;
template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid3fTypes::VecCoord >;
template class SOFA_ENGINE_API IndexValueMapper< defaulttype::Rigid3fTypes::VecDeriv >;
#endif //SOFA_DOUBLE


} // namespace engine

} // namespace component

} // namespace sofa
