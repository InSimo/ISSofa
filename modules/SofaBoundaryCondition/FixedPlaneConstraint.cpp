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
#define SOFA_COMPONENT_PROJECTIVECONSTRAINTSET_FIXEDPLANECONSTRAINT_CPP

#include <SofaBoundaryCondition/FixedPlaneConstraint.inl>
#include <sofa/core/behavior/ProjectiveConstraintSet.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace projectiveconstraintset
{

using namespace sofa::defaulttype;
using namespace sofa::helper;

SOFA_DECL_CLASS(FixedPlaneConstraint)

int FixedPlaneConstraintClass = core::RegisterObject("Project particles on a given plane")
#ifndef SOFA_FLOAT
        .add< FixedPlaneConstraint<Vec3dTypes> >()
        .add< FixedPlaneConstraint<Vec6dTypes> >()
        .add< FixedPlaneConstraint<Rigid3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
        .add< FixedPlaneConstraint<Vec3fTypes> >()
        .add< FixedPlaneConstraint<Vec6fTypes> >()
        .add< FixedPlaneConstraint<Rigid3fTypes> >()
#endif
        ;


#ifndef SOFA_FLOAT
template class SOFA_BOUNDARY_CONDITION_API FixedPlaneConstraint<Rigid3dTypes>;
template class SOFA_BOUNDARY_CONDITION_API FixedPlaneConstraint<Vec3dTypes>;
template class SOFA_BOUNDARY_CONDITION_API FixedPlaneConstraint<Vec6dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_BOUNDARY_CONDITION_API FixedPlaneConstraint<Rigid3fTypes>;
template class SOFA_BOUNDARY_CONDITION_API FixedPlaneConstraint<Vec3fTypes>;
template class SOFA_BOUNDARY_CONDITION_API FixedPlaneConstraint<Vec6fTypes>;
#endif

} // namespace projectiveconstraintset

} // namespace component

} // namespace sofa

