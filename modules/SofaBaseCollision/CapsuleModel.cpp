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


#include <SofaBaseCollision/CapsuleModel.inl>
#include <sofa/core/ObjectFactory.h>


namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
using namespace sofa::core::collision;
using namespace helper;

SOFA_DECL_CLASS(Capsule)

int CapsuleModelClass = core::RegisterObject("Collision model which represents a set of Capsules")
#ifndef SOFA_FLOAT
        .add<  TCapsuleModel<Vec3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
        .add < TCapsuleModel<Vec3fTypes> >()
#endif
        .addAlias("Capsule")
        .addAlias("CapsuleModel")
//.addAlias("CapsuleMesh")
//.addAlias("CapsuleSet")
        ;

#ifndef SOFA_FLOAT
template class SOFA_BASE_COLLISION_API TCapsule<defaulttype::Vec3dTypes>;
template class SOFA_BASE_COLLISION_API TCapsuleModel<defaulttype::Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_BASE_COLLISION_API TCapsule<defaulttype::Vec3fTypes>;
template class SOFA_BASE_COLLISION_API TCapsuleModel<defaulttype::Vec3fTypes>;
#endif


} // namespace collision

} // namespace component

} // namespace sofa

