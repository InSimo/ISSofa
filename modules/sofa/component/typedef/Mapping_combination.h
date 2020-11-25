/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
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


#ifndef SOFA_TYPEDEF_Mapping_combination_H
#define SOFA_TYPEDEF_Mapping_combination_H

//Default files containing the declaration of the vector type
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>


//Default files needed to create a Mapping
#include <sofa/core/State.h>
#include <sofa/core/Mapping.h>


#include <SofaBaseMechanics/BarycentricMapping.h>
#include <SofaMiscMapping/BeamLinearMapping.h>
#include <SofaMiscMapping/CenterOfMassMapping.h>
#include <SofaMiscMapping/CenterPointMechanicalMapping.h>
#include <SofaMiscMapping/CurveMapping.h>
#include <SofaMiscMapping/ExternalInterpolationMapping.h>
#include <SofaNonUniformFem/HexahedronCompositeFEMMapping.h>
#include <SofaBaseMechanics/IdentityMapping.h>
#include <SofaVolumetricData/ImplicitSurfaceMapping.h>
#include <SofaRigid/LineSetSkinningMapping.h>
#include <SofaTopologyMapping/Mesh2PointMechanicalMapping.h>
#include <SofaRigid/RigidMapping.h>
#include <SofaRigid/RigidRigidMapping.h>
#include <SofaSphFluid/SPHFluidSurfaceMapping.h>
#include <SofaTopologyMapping/SimpleTesselatedTetraMechanicalMapping.h>
#include <SofaRigid/SkinningMapping.h>
#include <SofaBaseMechanics/SubsetMapping.h>
#include <SofaMiscMapping/TubularMapping.h>



//---------------------------------------------------------------------------------------------
//Typedef for BarycentricMapping
typedef sofa::component::mapping::BarycentricMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > BarycentricMapping3d_to_3f;
typedef sofa::component::mapping::BarycentricMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > BarycentricMapping3f_to_3d;
typedef sofa::component::mapping::BarycentricMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdRigidTypes<3, float> > BarycentricMapping3d_to_Rigid3f;
typedef sofa::component::mapping::BarycentricMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdRigidTypes<3, double> > BarycentricMapping3f_to_Rigid3d;



//---------------------------------------------------------------------------------------------
//Typedef for BeamLinearMapping
typedef sofa::component::mapping::BeamLinearMapping<sofa::defaulttype::StdRigidTypes<3, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > BeamLinearMappingRigid3d_to_3f;
typedef sofa::component::mapping::BeamLinearMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > BeamLinearMappingRigid3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for CenterOfMassMapping
typedef sofa::component::mapping::CenterOfMassMapping<sofa::defaulttype::StdRigidTypes<3, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > CenterOfMassMappingRigid3d_to_3f;
typedef sofa::component::mapping::CenterOfMassMapping<sofa::defaulttype::StdRigidTypes<2, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > CenterOfMassMappingRigid2d_to_2f;
typedef sofa::component::mapping::CenterOfMassMapping<sofa::defaulttype::StdRigidTypes<2, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> > CenterOfMassMappingRigid2f_to_2d;
typedef sofa::component::mapping::CenterOfMassMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::ExtVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > CenterOfMassMappingRigid3f_to_Ext3d;
typedef sofa::component::mapping::CenterOfMassMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > CenterOfMassMappingRigid3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for CenterPointMechanicalMapping
typedef sofa::component::mapping::CenterPointMechanicalMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > CenterPointMechanicalMapping3d_to_3f;
typedef sofa::component::mapping::CenterPointMechanicalMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > CenterPointMechanicalMapping3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for CurveMapping
typedef sofa::component::mapping::CurveMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdRigidTypes<3, float> > CurveMapping3d_to_Rigid3f;
typedef sofa::component::mapping::CurveMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdRigidTypes<3, double> > CurveMapping3f_to_Rigid3d;



//---------------------------------------------------------------------------------------------
//Typedef for ExternalInterpolationMapping
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > ExternalInterpolationMapping1d_to_1f;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double> > ExternalInterpolationMapping1f_to_1d;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > ExternalInterpolationMapping2d_to_2f;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> > ExternalInterpolationMapping2f_to_2d;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > ExternalInterpolationMapping3d_to_3f;
typedef sofa::component::mapping::ExternalInterpolationMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > ExternalInterpolationMapping3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for HexahedronCompositeFEMMapping
typedef sofa::component::mapping::HexahedronCompositeFEMMapping<sofa::core::Mapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > > HexahedronCompositeFEMMapping3d_to_3f;
typedef sofa::component::mapping::HexahedronCompositeFEMMapping<sofa::core::Mapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > > HexahedronCompositeFEMMapping3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for IdentityMapping
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdRigidTypes<3, double>, sofa::defaulttype::StdRigidTypes<3, float> > IdentityMappingRigid3d_to_Rigid3f;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdRigidTypes<2, double>, sofa::defaulttype::StdRigidTypes<2, float> > IdentityMappingRigid2d_to_Rigid2f;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdRigidTypes<2, float>, sofa::defaulttype::StdRigidTypes<2, double> > IdentityMappingRigid2f_to_Rigid2d;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::StdRigidTypes<3, double> > IdentityMappingRigid3f_to_Rigid3d;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > IdentityMapping1d_to_1f;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double> > IdentityMapping1f_to_1d;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > IdentityMapping2d_to_2f;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> > IdentityMapping2f_to_2d;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > IdentityMapping3d_to_3f;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > IdentityMapping3f_to_3d;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<6, double>, sofa::defaulttype::Vec<6, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<6, float>, sofa::defaulttype::Vec<6, float>, float> > IdentityMapping6d_to_6f;
typedef sofa::component::mapping::IdentityMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<6, float>, sofa::defaulttype::Vec<6, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<6, double>, sofa::defaulttype::Vec<6, double>, double> > IdentityMapping6f_to_6d;



//---------------------------------------------------------------------------------------------
//Typedef for ImplicitSurfaceMapping
typedef sofa::component::mapping::ImplicitSurfaceMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > ImplicitSurfaceMapping3d_to_3f;
typedef sofa::component::mapping::ImplicitSurfaceMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > ImplicitSurfaceMapping3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for LineSetSkinningMapping
typedef sofa::component::mapping::LineSetSkinningMapping<sofa::defaulttype::StdRigidTypes<3, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > LineSetSkinningMappingRigid3d_to_3f;
typedef sofa::component::mapping::LineSetSkinningMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > LineSetSkinningMappingRigid3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for Mesh2PointMechanicalMapping
typedef sofa::component::mapping::Mesh2PointMechanicalMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > Mesh2PointMechanicalMapping3d_to_3f;
typedef sofa::component::mapping::Mesh2PointMechanicalMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > Mesh2PointMechanicalMapping3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for RigidMapping
typedef sofa::component::mapping::RigidMapping<sofa::defaulttype::StdRigidTypes<3, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > RigidMappingRigid3d_to_3f;
typedef sofa::component::mapping::RigidMapping<sofa::defaulttype::StdRigidTypes<2, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > RigidMappingRigid2d_to_2f;
typedef sofa::component::mapping::RigidMapping<sofa::defaulttype::StdRigidTypes<2, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, double>, sofa::defaulttype::Vec<2, double>, double> > RigidMappingRigid2f_to_2d;
typedef sofa::component::mapping::RigidMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > RigidMappingRigid3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for RigidRigidMapping
typedef sofa::component::mapping::RigidRigidMapping<sofa::defaulttype::StdRigidTypes<3, double>, sofa::defaulttype::StdRigidTypes<3, float> > RigidRigidMappingRigid3d_to_Rigid3f;
typedef sofa::component::mapping::RigidRigidMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::StdRigidTypes<3, double> > RigidRigidMappingRigid3f_to_Rigid3d;



//---------------------------------------------------------------------------------------------
//Typedef for SPHFluidSurfaceMapping
typedef sofa::component::mapping::SPHFluidSurfaceMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > SPHFluidSurfaceMapping3d_to_3f;
typedef sofa::component::mapping::SPHFluidSurfaceMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > SPHFluidSurfaceMapping3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for SimpleTesselatedTetraMechanicalMapping
typedef sofa::component::mapping::SimpleTesselatedTetraMechanicalMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > SimpleTesselatedTetraMechanicalMapping3d_to_3f;
typedef sofa::component::mapping::SimpleTesselatedTetraMechanicalMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > SimpleTesselatedTetraMechanicalMapping3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for SkinningMapping
typedef sofa::component::mapping::SkinningMapping<sofa::defaulttype::StdRigidTypes<3, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > SkinningMappingRigid3d_to_3f;
typedef sofa::component::mapping::SkinningMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > SkinningMappingRigid3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for SubsetMapping
typedef sofa::component::mapping::SubsetMapping<sofa::defaulttype::StdRigidTypes<3, double>, sofa::defaulttype::StdRigidTypes<3, float> > SubsetMappingRigid3d_to_Rigid3f;
typedef sofa::component::mapping::SubsetMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::StdRigidTypes<3, double> > SubsetMappingRigid3f_to_Rigid3d;
typedef sofa::component::mapping::SubsetMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > SubsetMapping1d_to_1f;
typedef sofa::component::mapping::SubsetMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, double>, sofa::defaulttype::Vec<1, double>, double> > SubsetMapping1f_to_1d;
typedef sofa::component::mapping::SubsetMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > SubsetMapping3d_to_3f;
typedef sofa::component::mapping::SubsetMapping<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > SubsetMapping3f_to_3d;



//---------------------------------------------------------------------------------------------
//Typedef for TubularMapping
typedef sofa::component::mapping::TubularMapping<sofa::defaulttype::StdRigidTypes<3, double>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > TubularMappingRigid3d_to_3f;
typedef sofa::component::mapping::TubularMapping<sofa::defaulttype::StdRigidTypes<3, float>, sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, double>, sofa::defaulttype::Vec<3, double>, double> > TubularMappingRigid3f_to_3d;





#endif
