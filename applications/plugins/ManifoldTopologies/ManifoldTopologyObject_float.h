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



// File automatically generated by "generateTypedef"


#ifndef SOFA_TYPEDEF_TopologyObject_float_H
#define SOFA_TYPEDEF_TopologyObject_float_H

//Default files containing the declaration of the vector type
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>


#ifdef SOFA_GPU_CUDA
#include <sofa/gpu/cuda/CudaTypes.h>
#endif
#ifdef SOFA_GPU_OPENCL
#include <sofa/gpu/opencl/OpenCLTypes.h>
#endif

#include "ManifoldEdgeSetGeometryAlgorithms.h"
#include "ManifoldEdgeSetTopologyAlgorithms.h"
#include "ManifoldTriangleSetTopologyAlgorithms.h"


//---------------------------------------------------------------------------------------------
//Typedef for ManifoldEdgeSetGeometryAlgorithms
typedef sofa::component::topology::ManifoldEdgeSetGeometryAlgorithms<sofa::defaulttype::StdRigidTypes<2, float> > ManifoldEdgeSetGeometryAlgorithmsRigid2f;
typedef sofa::component::topology::ManifoldEdgeSetGeometryAlgorithms<sofa::defaulttype::StdRigidTypes<3, float> > ManifoldEdgeSetGeometryAlgorithmsRigid3f;
typedef sofa::component::topology::ManifoldEdgeSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > ManifoldEdgeSetGeometryAlgorithms1f;
typedef sofa::component::topology::ManifoldEdgeSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > ManifoldEdgeSetGeometryAlgorithms2f;
typedef sofa::component::topology::ManifoldEdgeSetGeometryAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > ManifoldEdgeSetGeometryAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for ManifoldEdgeSetTopologyAlgorithms
typedef sofa::component::topology::ManifoldEdgeSetTopologyAlgorithms<sofa::defaulttype::StdRigidTypes<2, float> > ManifoldEdgeSetTopologyAlgorithmsRigid2f;
typedef sofa::component::topology::ManifoldEdgeSetTopologyAlgorithms<sofa::defaulttype::StdRigidTypes<3, float> > ManifoldEdgeSetTopologyAlgorithmsRigid3f;
typedef sofa::component::topology::ManifoldEdgeSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > ManifoldEdgeSetTopologyAlgorithms1f;
typedef sofa::component::topology::ManifoldEdgeSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > ManifoldEdgeSetTopologyAlgorithms2f;
typedef sofa::component::topology::ManifoldEdgeSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > ManifoldEdgeSetTopologyAlgorithms3f;



//---------------------------------------------------------------------------------------------
//Typedef for ManifoldTriangleSetTopologyAlgorithms
typedef sofa::component::topology::ManifoldTriangleSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<1, float>, sofa::defaulttype::Vec<1, float>, float> > ManifoldTriangleSetTopologyAlgorithms1f;
typedef sofa::component::topology::ManifoldTriangleSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<2, float>, sofa::defaulttype::Vec<2, float>, float> > ManifoldTriangleSetTopologyAlgorithms2f;
typedef sofa::component::topology::ManifoldTriangleSetTopologyAlgorithms<sofa::defaulttype::StdVectorTypes<sofa::defaulttype::Vec<3, float>, sofa::defaulttype::Vec<3, float>, float> > ManifoldTriangleSetTopologyAlgorithms3f;

#endif
