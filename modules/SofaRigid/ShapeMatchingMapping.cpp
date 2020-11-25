/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
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
#define SOFA_COMPONENT_MAPPING_SHAPEMATCHINGMAPPING_CPP

#include <SofaRigid/ShapeMatchingMapping.inl>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace mapping
{

SOFA_DECL_CLASS(ShapeMatchingMapping)

using namespace sofa::defaulttype;

int ShapeMatchingMappingClass = core::RegisterObject("Compute target frames using shape matching deformation method by Muller et al.")
#ifndef SOFA_FLOAT
        .add< ShapeMatchingMapping<Vec3dTypes, Rigid3dTypes> >()
#endif //SOFA_FLOAT
        ;

#ifndef SOFA_FLOAT
template class SOFA_RIGID_API ShapeMatchingMapping<Vec3dTypes, Rigid3dTypes>;
#endif //SOFA_FLOAT


} // namespace mapping

} // namespace component

} // namespace sofa

