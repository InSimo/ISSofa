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



// File automatically generated by "generateTypedef"


#ifndef SOFA_TYPEDEF_MechanicalMapping_combination_H
#define SOFA_TYPEDEF_MechanicalMapping_combination_H

//Default files containing the declaration of the vector type
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>

//Default files needed to create a Mechanical Mapping
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/MechanicalMapping.h>


#include <SofaBaseMechanics/BarycentricMapping.h>
#include <SofaMiscMapping/BeamLinearMapping.h>
#include <SofaMiscMapping/CenterOfMassMapping.h>
#include <SofaMiscMapping/CenterPointMechanicalMapping.h>
#include <SofaMiscMapping/CurveMapping.h>
#include <SofaMiscMapping/ExternalInterpolationMapping.h>
#include <SofaBaseMechanics/IdentityMapping.h>
#include <SofaRigid/LineSetSkinningMapping.h>
#include <SofaTopologyMapping/Mesh2PointMechanicalMapping.h>
#include <SofaRigid/RigidMapping.h>
#include <SofaRigid/RigidRigidMapping.h>
#include <SofaTopologyMapping/SimpleTesselatedTetraMechanicalMapping.h>
#include <SofaRigid/SkinningMapping.h>
#include <SofaBaseMechanics/SubsetMapping.h>
#include <SofaMiscMapping/TubularMapping.h>



//---------------------------------------------------------------------------------------------
//Typedef for BarycentricMapping
typedef sofa::component::mapping::BarycentricMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> > > > BarycentricMechanicalMapping3f_to_Rigid3d;
typedef sofa::component::mapping::BarycentricMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> > > > BarycentricMechanicalMapping3d_to_Rigid3f;
typedef sofa::component::mapping::BarycentricMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > BarycentricMechanicalMapping3f_to_3d;
typedef sofa::component::mapping::BarycentricMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > BarycentricMechanicalMapping3d_to_3f;



//---------------------------------------------------------------------------------------------
//Typedef for BeamLinearMapping
typedef sofa::component::mapping::BeamLinearMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > BeamLinearMechanicalMappingRigid3d_to_3f;
typedef sofa::component::mapping::BeamLinearMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > BeamLinearMechanicalMappingRigid3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for CenterOfMassMapping
typedef sofa::component::mapping::CenterOfMassMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > CenterOfMassMechanicalMappingRigid3d_to_3f;
typedef sofa::component::mapping::CenterOfMassMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > CenterOfMassMechanicalMappingRigid3f_to_3d;
typedef sofa::component::mapping::CenterOfMassMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<2, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > > > CenterOfMassMechanicalMappingRigid2d_to_2f;
typedef sofa::component::mapping::CenterOfMassMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<2, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> > > > CenterOfMassMechanicalMappingRigid2f_to_2d;



//---------------------------------------------------------------------------------------------
//Typedef for CenterPointMechanicalMapping
typedef sofa::component::mapping::CenterPointMechanicalMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > CenterPointMechanicalMechanicalMapping3f_to_3d;
typedef sofa::component::mapping::CenterPointMechanicalMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > CenterPointMechanicalMechanicalMapping3d_to_3f;



//---------------------------------------------------------------------------------------------
//Typedef for CurveMapping
typedef sofa::component::mapping::CurveMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> > > > CurveMechanicalMapping3d_to_Rigid3f;
typedef sofa::component::mapping::CurveMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> > > > CurveMechanicalMapping3f_to_Rigid3d;



//---------------------------------------------------------------------------------------------
//Typedef for ExternalInterpolationMapping
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > ExternalInterpolationMechanicalMapping3f_to_3d;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > ExternalInterpolationMechanicalMapping3d_to_3f;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> > > > ExternalInterpolationMechanicalMapping2f_to_2d;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > > > ExternalInterpolationMechanicalMapping2d_to_2f;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double> > > > ExternalInterpolationMechanicalMapping1f_to_1d;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > > > ExternalInterpolationMechanicalMapping1d_to_1f;



//---------------------------------------------------------------------------------------------
//Typedef for IdentityMapping
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > IdentityMechanicalMapping3f_to_3d;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > IdentityMechanicalMapping3d_to_3f;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> > > > IdentityMechanicalMapping2f_to_2d;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > > > IdentityMechanicalMapping2d_to_2f;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double> > > > IdentityMechanicalMapping1f_to_1d;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > > > IdentityMechanicalMapping1d_to_1f;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<6, float>, sofa::defaulttype::Vec<6, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<6, double>, sofa::defaulttype::Vec<6, double>, double> > > > IdentityMechanicalMapping6f_to_6d;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<6, double>, sofa::defaulttype::Vec<6, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<6, float>, sofa::defaulttype::Vec<6, float>, float> > > > IdentityMechanicalMapping6d_to_6f;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> > > > IdentityMechanicalMappingRigid3d_to_Rigid3f;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> > > > IdentityMechanicalMappingRigid3f_to_Rigid3d;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<2, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<2, float> > > > IdentityMechanicalMappingRigid2d_to_Rigid2f;
typedef sofa::component::mapping::IdentityMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<2, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<2, double> > > > IdentityMechanicalMappingRigid2f_to_Rigid2d;



//---------------------------------------------------------------------------------------------
//Typedef for LineSetSkinningMapping
typedef sofa::component::mapping::LineSetSkinningMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > LineSetSkinningMechanicalMappingRigid3d_to_3f;
typedef sofa::component::mapping::LineSetSkinningMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > LineSetSkinningMechanicalMappingRigid3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for Mesh2PointMechanicalMapping
typedef sofa::component::mapping::Mesh2PointMechanicalMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > Mesh2PointMechanicalMechanicalMapping3f_to_3d;
typedef sofa::component::mapping::Mesh2PointMechanicalMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > Mesh2PointMechanicalMechanicalMapping3d_to_3f;



//---------------------------------------------------------------------------------------------
//Typedef for RigidMapping
typedef sofa::component::mapping::RigidMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > RigidMechanicalMappingRigid3d_to_3f;
typedef sofa::component::mapping::RigidMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > RigidMechanicalMappingRigid3f_to_3d;
typedef sofa::component::mapping::RigidMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<2, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > > > RigidMechanicalMappingRigid2d_to_2f;
typedef sofa::component::mapping::RigidMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<2, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> > > > RigidMechanicalMappingRigid2f_to_2d;



//---------------------------------------------------------------------------------------------
//Typedef for RigidRigidMapping
typedef sofa::component::mapping::RigidRigidMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> > > > RigidRigidMechanicalMappingRigid3d_to_Rigid3f;
typedef sofa::component::mapping::RigidRigidMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> > > > RigidRigidMechanicalMappingRigid3f_to_Rigid3d;



//---------------------------------------------------------------------------------------------
//Typedef for SimpleTesselatedTetraMechanicalMapping
typedef sofa::component::mapping::SimpleTesselatedTetraMechanicalMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > SimpleTesselatedTetraMechanicalMechanicalMapping3f_to_3d;
typedef sofa::component::mapping::SimpleTesselatedTetraMechanicalMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > SimpleTesselatedTetraMechanicalMechanicalMapping3d_to_3f;



//---------------------------------------------------------------------------------------------
//Typedef for SkinningMapping
typedef sofa::component::mapping::SkinningMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > SkinningMechanicalMappingRigid3d_to_3f;
typedef sofa::component::mapping::SkinningMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > SkinningMechanicalMappingRigid3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for SubsetMapping
typedef sofa::component::mapping::SubsetMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > SubsetMechanicalMapping3f_to_3d;
typedef sofa::component::mapping::SubsetMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > SubsetMechanicalMapping3d_to_3f;
typedef sofa::component::mapping::SubsetMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double> > > > SubsetMechanicalMapping1f_to_1d;
typedef sofa::component::mapping::SubsetMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > > > SubsetMechanicalMapping1d_to_1f;
typedef sofa::component::mapping::SubsetMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> > > > SubsetMechanicalMappingRigid3f_to_Rigid3d;
typedef sofa::component::mapping::SubsetMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> > > > SubsetMechanicalMappingRigid3d_to_Rigid3f;



//---------------------------------------------------------------------------------------------
//Typedef for TubularMapping
typedef sofa::component::mapping::TubularMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, double> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > > TubularMechanicalMappingRigid3d_to_3f;
typedef sofa::component::mapping::TubularMapping<sofa::core::behavior::MechanicalMapping<sofa::core::behavior::MechanicalState<sofa::defaulttype::StdRigidTypes<3, float> >, sofa::core::behavior::MechanicalState<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > > TubularMechanicalMappingRigid3f_to_3d;





#endif
