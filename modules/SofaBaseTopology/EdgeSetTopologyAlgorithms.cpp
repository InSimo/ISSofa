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
#define SOFA_COMPONENT_TOPOLOGY_EDGESETTOPOLOGYALGORITHMS_CPP
#include <SofaBaseTopology/EdgeSetTopologyAlgorithms.h>
#include <SofaBaseTopology/EdgeSetTopologyAlgorithms.inl>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace topology
{

using namespace sofa::defaulttype;
SOFA_DECL_CLASS(EdgeSetTopologyAlgorithms)
int EdgeSetTopologyAlgorithmsClass = core::RegisterObject("Edge set topology algorithms")
#ifdef SOFA_FLOAT
        .add< EdgeSetTopologyAlgorithms<Vec3fTypes> >(true) // default template
#else
        .add< EdgeSetTopologyAlgorithms<Vec3dTypes> >(true) // default template
#ifndef SOFA_DOUBLE
        .add< EdgeSetTopologyAlgorithms<Vec3fTypes> >() // default template
#endif
#endif
#ifndef SOFA_FLOAT
        .add< EdgeSetTopologyAlgorithms<Vec2dTypes> >()
        .add< EdgeSetTopologyAlgorithms<Vec1dTypes> >()
        .add< EdgeSetTopologyAlgorithms<Rigid3dTypes> >()
        .add< EdgeSetTopologyAlgorithms<Rigid2dTypes> >()
#endif
#ifndef SOFA_DOUBLE
        .add< EdgeSetTopologyAlgorithms<Vec2fTypes> >()
        .add< EdgeSetTopologyAlgorithms<Vec1fTypes> >()
        .add< EdgeSetTopologyAlgorithms<Rigid3fTypes> >()
        .add< EdgeSetTopologyAlgorithms<Rigid2fTypes> >()
#endif
        ;
#ifndef SOFA_FLOAT
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Vec3dTypes>;
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Vec2dTypes>;
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Vec1dTypes>;
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Rigid3dTypes>;
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Rigid2dTypes>;
#endif

#ifndef SOFA_DOUBLE
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Vec3fTypes>;
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Vec2fTypes>;
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Vec1fTypes>;
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Rigid3fTypes>;
template class SOFA_BASE_TOPOLOGY_API EdgeSetTopologyAlgorithms<Rigid2fTypes>;
#endif

} // namespace topology

} // namespace component

} // namespace sofa

