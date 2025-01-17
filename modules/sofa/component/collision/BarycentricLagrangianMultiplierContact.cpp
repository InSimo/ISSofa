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
#include <sofa/component/collision/BarycentricLagrangianMultiplierContact.inl>

namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
using namespace collision;

SOFA_DECL_CLASS(BarycentricLagrangianMultiplierContact)

sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<PointModel, PointModel> > PointPointLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<LineModel, PointModel> > LinePointLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<LineModel, LineModel> > LineLineLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<TriangleModel, PointModel> > TrianglePointLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<TriangleModel, LineModel> > TriangleLineLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<TriangleModel, TriangleModel> > TriangleTriangleLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<SphereModel, SphereModel> > SphereSphereLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<SphereModel, PointModel> > SpherePointLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<LineModel, SphereModel> > LineSphereLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<TriangleModel, SphereModel> > TriangleSphereLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<SphereTreeModel,SphereTreeModel> > SphereTreeSphereTreeLagrangianMultiplierContactClass("LagrangianMultiplier",true);
sofa::core::collision::ContactCreator< BarycentricLagrangianMultiplierContact<SphereTreeModel,TriangleModel> > SphereTreeTriangleLagrangianMultiplierContactClass("LagrangianMultiplier",true);

} // namespace collision

} // namespace component

} // namespace sofa

