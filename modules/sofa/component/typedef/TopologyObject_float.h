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


#ifndef SOFA_TYPEDEF_TopologyObject_float_H
#define SOFA_TYPEDEF_TopologyObject_float_H

//Default files containing the declaration of the vector type
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>




#include <SofaNonUniformFem/DynamicSparseGridGeometryAlgorithms.h>
#include <SofaNonUniformFem/DynamicSparseGridTopologyAlgorithms.h>
#include <SofaBaseTopology/EdgeSetGeometryAlgorithms.h>
#include <SofaBaseTopology/EdgeSetTopologyAlgorithms.h>
#include <SofaBaseTopology/HexahedronSetGeometryAlgorithms.h>
#include <SofaBaseTopology/HexahedronSetTopologyAlgorithms.h>
#include <SofaBaseTopology/PointSetGeometryAlgorithms.h>
#include <SofaBaseTopology/PointSetTopologyAlgorithms.h>
#include <SofaBaseTopology/QuadSetGeometryAlgorithms.h>
#include <SofaBaseTopology/QuadSetTopologyAlgorithms.h>
#include <SofaBaseTopology/TetrahedronSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TetrahedronSetTopologyAlgorithms.h>
#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TriangleSetTopologyAlgorithms.h>



//---------------------------------------------------------------------------------------------
//Typedef for DynamicSparseGridGeometryAlgorithms
typedef sofa::component::topology::DynamicSparseGridGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > DynamicSparseGridGeometryAlgorithms1f;
typedef sofa::component::topology::DynamicSparseGridGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > DynamicSparseGridGeometryAlgorithms2f;
typedef sofa::component::topology::DynamicSparseGridGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > DynamicSparseGridGeometryAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for DynamicSparseGridTopologyAlgorithms
typedef sofa::component::topology::DynamicSparseGridTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > DynamicSparseGridTopologyAlgorithms1f;
typedef sofa::component::topology::DynamicSparseGridTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > DynamicSparseGridTopologyAlgorithms2f;
typedef sofa::component::topology::DynamicSparseGridTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > DynamicSparseGridTopologyAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for EdgeSetGeometryAlgorithms
typedef sofa::component::topology::EdgeSetGeometryAlgorithms<sofa::defaulttype::StdRigidTypes<2, float> > EdgeSetGeometryAlgorithmsRigid2f;
typedef sofa::component::topology::EdgeSetGeometryAlgorithms<sofa::defaulttype::StdRigidTypes<3, float> > EdgeSetGeometryAlgorithmsRigid3f;
typedef sofa::component::topology::EdgeSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > EdgeSetGeometryAlgorithms1f;
typedef sofa::component::topology::EdgeSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > EdgeSetGeometryAlgorithms2f;
typedef sofa::component::topology::EdgeSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > EdgeSetGeometryAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for EdgeSetTopologyAlgorithms
typedef sofa::component::topology::EdgeSetTopologyAlgorithms<sofa::defaulttype::StdRigidTypes<2, float> > EdgeSetTopologyAlgorithmsRigid2f;
typedef sofa::component::topology::EdgeSetTopologyAlgorithms<sofa::defaulttype::StdRigidTypes<3, float> > EdgeSetTopologyAlgorithmsRigid3f;
typedef sofa::component::topology::EdgeSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > EdgeSetTopologyAlgorithms1f;
typedef sofa::component::topology::EdgeSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > EdgeSetTopologyAlgorithms2f;
typedef sofa::component::topology::EdgeSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > EdgeSetTopologyAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for HexahedronSetGeometryAlgorithms
typedef sofa::component::topology::HexahedronSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > HexahedronSetGeometryAlgorithms1f;
typedef sofa::component::topology::HexahedronSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > HexahedronSetGeometryAlgorithms2f;
typedef sofa::component::topology::HexahedronSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > HexahedronSetGeometryAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for HexahedronSetTopologyAlgorithms
typedef sofa::component::topology::HexahedronSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > HexahedronSetTopologyAlgorithms1f;
typedef sofa::component::topology::HexahedronSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > HexahedronSetTopologyAlgorithms2f;
typedef sofa::component::topology::HexahedronSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > HexahedronSetTopologyAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for PointSetGeometryAlgorithms
typedef sofa::component::topology::PointSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > PointSetGeometryAlgorithms1f;
typedef sofa::component::topology::PointSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > PointSetGeometryAlgorithms2f;
typedef sofa::component::topology::PointSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > PointSetGeometryAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for PointSetTopologyAlgorithms
typedef sofa::component::topology::PointSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > PointSetTopologyAlgorithms1f;
typedef sofa::component::topology::PointSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > PointSetTopologyAlgorithms2f;
typedef sofa::component::topology::PointSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > PointSetTopologyAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for QuadSetGeometryAlgorithms
typedef sofa::component::topology::QuadSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > QuadSetGeometryAlgorithms1f;
typedef sofa::component::topology::QuadSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > QuadSetGeometryAlgorithms2f;
typedef sofa::component::topology::QuadSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > QuadSetGeometryAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for QuadSetTopologyAlgorithms
typedef sofa::component::topology::QuadSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > QuadSetTopologyAlgorithms1f;
typedef sofa::component::topology::QuadSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > QuadSetTopologyAlgorithms2f;
typedef sofa::component::topology::QuadSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > QuadSetTopologyAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for TetrahedronSetGeometryAlgorithms
typedef sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > TetrahedronSetGeometryAlgorithms1f;
typedef sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > TetrahedronSetGeometryAlgorithms2f;
typedef sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > TetrahedronSetGeometryAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for TetrahedronSetTopologyAlgorithms
typedef sofa::component::topology::TetrahedronSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > TetrahedronSetTopologyAlgorithms1f;
typedef sofa::component::topology::TetrahedronSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > TetrahedronSetTopologyAlgorithms2f;
typedef sofa::component::topology::TetrahedronSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > TetrahedronSetTopologyAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for TriangleSetGeometryAlgorithms
typedef sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > TriangleSetGeometryAlgorithms1f;
typedef sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > TriangleSetGeometryAlgorithms2f;
typedef sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > TriangleSetGeometryAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for TriangleSetTopologyAlgorithms
typedef sofa::component::topology::TriangleSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > TriangleSetTopologyAlgorithms1f;
typedef sofa::component::topology::TriangleSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > TriangleSetTopologyAlgorithms2f;
typedef sofa::component::topology::TriangleSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > TriangleSetTopologyAlgorithms3f;





#ifdef SOFA_FLOAT
typedef DynamicSparseGridGeometryAlgorithms1f DynamicSparseGridGeometryAlgorithms1;
typedef DynamicSparseGridGeometryAlgorithms2f DynamicSparseGridGeometryAlgorithms2;
typedef DynamicSparseGridGeometryAlgorithms3f DynamicSparseGridGeometryAlgorithms3;
typedef DynamicSparseGridTopologyAlgorithms1f DynamicSparseGridTopologyAlgorithms1;
typedef DynamicSparseGridTopologyAlgorithms2f DynamicSparseGridTopologyAlgorithms2;
typedef DynamicSparseGridTopologyAlgorithms3f DynamicSparseGridTopologyAlgorithms3;
typedef EdgeSetGeometryAlgorithmsRigid2f EdgeSetGeometryAlgorithmsRigid2;
typedef EdgeSetGeometryAlgorithmsRigid3f EdgeSetGeometryAlgorithmsRigid3;
typedef EdgeSetGeometryAlgorithms1f EdgeSetGeometryAlgorithms1;
typedef EdgeSetGeometryAlgorithms2f EdgeSetGeometryAlgorithms2;
typedef EdgeSetGeometryAlgorithms3f EdgeSetGeometryAlgorithms3;
typedef EdgeSetTopologyAlgorithmsRigid2f EdgeSetTopologyAlgorithmsRigid2;
typedef EdgeSetTopologyAlgorithmsRigid3f EdgeSetTopologyAlgorithmsRigid3;
typedef EdgeSetTopologyAlgorithms1f EdgeSetTopologyAlgorithms1;
typedef EdgeSetTopologyAlgorithms2f EdgeSetTopologyAlgorithms2;
typedef EdgeSetTopologyAlgorithms3f EdgeSetTopologyAlgorithms3;
typedef HexahedronSetGeometryAlgorithms1f HexahedronSetGeometryAlgorithms1;
typedef HexahedronSetGeometryAlgorithms2f HexahedronSetGeometryAlgorithms2;
typedef HexahedronSetGeometryAlgorithms3f HexahedronSetGeometryAlgorithms3;
typedef HexahedronSetTopologyAlgorithms1f HexahedronSetTopologyAlgorithms1;
typedef HexahedronSetTopologyAlgorithms2f HexahedronSetTopologyAlgorithms2;
typedef HexahedronSetTopologyAlgorithms3f HexahedronSetTopologyAlgorithms3;
typedef PointSetGeometryAlgorithms1f PointSetGeometryAlgorithms1;
typedef PointSetGeometryAlgorithms2f PointSetGeometryAlgorithms2;
typedef PointSetGeometryAlgorithms3f PointSetGeometryAlgorithms3;
typedef PointSetTopologyAlgorithms1f PointSetTopologyAlgorithms1;
typedef PointSetTopologyAlgorithms2f PointSetTopologyAlgorithms2;
typedef PointSetTopologyAlgorithms3f PointSetTopologyAlgorithms3;
typedef QuadSetGeometryAlgorithms1f QuadSetGeometryAlgorithms1;
typedef QuadSetGeometryAlgorithms2f QuadSetGeometryAlgorithms2;
typedef QuadSetGeometryAlgorithms3f QuadSetGeometryAlgorithms3;
typedef QuadSetTopologyAlgorithms1f QuadSetTopologyAlgorithms1;
typedef QuadSetTopologyAlgorithms2f QuadSetTopologyAlgorithms2;
typedef QuadSetTopologyAlgorithms3f QuadSetTopologyAlgorithms3;
typedef TetrahedronSetGeometryAlgorithms1f TetrahedronSetGeometryAlgorithms1;
typedef TetrahedronSetGeometryAlgorithms2f TetrahedronSetGeometryAlgorithms2;
typedef TetrahedronSetGeometryAlgorithms3f TetrahedronSetGeometryAlgorithms3;
typedef TetrahedronSetTopologyAlgorithms1f TetrahedronSetTopologyAlgorithms1;
typedef TetrahedronSetTopologyAlgorithms2f TetrahedronSetTopologyAlgorithms2;
typedef TetrahedronSetTopologyAlgorithms3f TetrahedronSetTopologyAlgorithms3;
typedef TriangleSetGeometryAlgorithms1f TriangleSetGeometryAlgorithms1;
typedef TriangleSetGeometryAlgorithms2f TriangleSetGeometryAlgorithms2;
typedef TriangleSetGeometryAlgorithms3f TriangleSetGeometryAlgorithms3;
typedef TriangleSetTopologyAlgorithms1f TriangleSetTopologyAlgorithms1;
typedef TriangleSetTopologyAlgorithms2f TriangleSetTopologyAlgorithms2;
typedef TriangleSetTopologyAlgorithms3f TriangleSetTopologyAlgorithms3;
#endif

#endif
