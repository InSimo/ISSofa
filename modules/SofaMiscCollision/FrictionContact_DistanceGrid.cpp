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
#include <SofaConstraint/FrictionContact.inl>
#include <SofaMeshCollision/RigidContactMapper.inl>
#include <SofaMeshCollision/BarycentricContactMapper.inl>
#include <SofaVolumetricData/DistanceGridCollisionModel.h>


using namespace sofa::core::collision;

namespace sofa
{

namespace component
{

namespace collision
{


sofa::core::collision::ContactCreator< FrictionContact<RigidDistanceGridCollisionModel, RigidDistanceGridCollisionModel> > DistanceGridDistanceGridFrictionContactClass("FrictionContact", true);
sofa::core::collision::ContactCreator< FrictionContact<RigidDistanceGridCollisionModel, PointModel> > DistanceGridPointFrictionContactClass("FrictionContact", true);
sofa::core::collision::ContactCreator< FrictionContact<RigidDistanceGridCollisionModel, SphereModel> > DistanceGridSphereFrictionContactClass("FrictionContact", true);
sofa::core::collision::ContactCreator< FrictionContact<RigidDistanceGridCollisionModel, TriangleModel> > DistanceGridTriangleFrictionContactClass("FrictionContact", true);

sofa::core::collision::ContactCreator< FrictionContact<FFDDistanceGridCollisionModel, FFDDistanceGridCollisionModel> > FFDDistanceGridFrictionContactClass("FrictionContact", true);
sofa::core::collision::ContactCreator< FrictionContact<FFDDistanceGridCollisionModel, RigidDistanceGridCollisionModel> > FFDDistanceGridRigidDistanceGridFrictionContactClass("FrictionContact", true);
sofa::core::collision::ContactCreator< FrictionContact<FFDDistanceGridCollisionModel, PointModel> > FFDDistanceGridPointFrictionContactClass("FrictionContact", true);
sofa::core::collision::ContactCreator< FrictionContact<FFDDistanceGridCollisionModel, SphereModel> > FFDDistanceGridSphereFrictionContactClass("FrictionContact", true);
sofa::core::collision::ContactCreator< FrictionContact<FFDDistanceGridCollisionModel, TriangleModel> > FFDDistanceGridTriangleFrictionContactClass("FrictionContact", true);


} // namespace collision

} // namespace component

} // namespace sofa
