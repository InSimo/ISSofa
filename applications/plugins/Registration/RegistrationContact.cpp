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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "RegistrationContact.inl"
#include <SofaMeshCollision/BarycentricContactMapper.inl>

#include <SofaMeshCollision/IdentityContactMapper.inl>
#include <SofaMeshCollision/RigidContactMapper.inl>
#include "RegistrationContactForceField.inl"

namespace sofa
{

namespace component
{

namespace collision
{

using namespace defaulttype;
using simulation::Node;

SOFA_DECL_CLASS(RegistrationContact)

sofa::core::collision::ContactCreator< RegistrationContact<SphereModel, SphereModel> > SphereSphereRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<SphereModel, PointModel> > SpherePointRegistrationContactClass("registration",true);
//sofa::core::collision::ContactCreator< RegistrationContact<SphereTreeModel, SphereTreeModel> > SphereTreeSphereTreeRegistrationContactClass("registration", true);
//sofa::core::collision::ContactCreator< RegistrationContact<SphereTreeModel, TriangleModel> > SphereTreeTriangleContactClass("registration", true);
sofa::core::collision::ContactCreator< RegistrationContact<PointModel, PointModel> > PointPointRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<LineModel, PointModel> > LinePointRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<LineModel, LineModel> > LineLineRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<LineModel, SphereModel> > LineSphereRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<TriangleModel, SphereModel> > TriangleSphereRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<TriangleModel, PointModel> > TrianglePointRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<TriangleModel, LineModel> > TriangleLineRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<TriangleModel, TriangleModel> > TriangleTriangleRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<TetrahedronModel, SphereModel> > TetrahedronSphereRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<TetrahedronModel, PointModel> > TetrahedronPointRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<TetrahedronModel, LineModel> > TetrahedronLineRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<TetrahedronModel, TriangleModel> > TetrahedronTriangleRegistrationContactClass("registration",true);
sofa::core::collision::ContactCreator< RegistrationContact<TetrahedronModel, TetrahedronModel> > TetrahedronTetrahedronRegistrationContactClass("registration",true);

sofa::core::collision::ContactCreator< RegistrationContact<RigidDistanceGridCollisionModel, RigidDistanceGridCollisionModel> > DistanceGridDistanceGridRegistrationContactClass("registration", true);
sofa::core::collision::ContactCreator< RegistrationContact<RigidDistanceGridCollisionModel, PointModel> > DistanceGridPointRegistrationContactClass("registration", true);
sofa::core::collision::ContactCreator< RegistrationContact<RigidDistanceGridCollisionModel, SphereModel> > DistanceGridSphereRegistrationContactClass("registration", true);
sofa::core::collision::ContactCreator< RegistrationContact<RigidDistanceGridCollisionModel, TriangleModel> > DistanceGridTriangleRegistrationContactClass("registration", true);

sofa::core::collision::ContactCreator< RegistrationContact<FFDDistanceGridCollisionModel, FFDDistanceGridCollisionModel> > FFDDistanceGridRegistrationContactClass("registration", true);
sofa::core::collision::ContactCreator< RegistrationContact<FFDDistanceGridCollisionModel, RigidDistanceGridCollisionModel> > FFDDistanceGridRigidDistanceGridRegistrationContactClass("registration", true);
sofa::core::collision::ContactCreator< RegistrationContact<FFDDistanceGridCollisionModel, PointModel> > FFDDistanceGridPoinRegistrationtContactClass("registration", true);
sofa::core::collision::ContactCreator< RegistrationContact<FFDDistanceGridCollisionModel, SphereModel> > FFDDistanceGridSphereRegistrationContactClass("registration", true);
sofa::core::collision::ContactCreator< RegistrationContact<FFDDistanceGridCollisionModel, TriangleModel> > FFDDistanceGridTriangleRegistrationContactClass("registration", true);


} // namespace collision

} // namespace component

} // namespace sofa

