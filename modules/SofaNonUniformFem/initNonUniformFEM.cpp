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

#include <sofa/SofaAdvanced.h>
#include <sofa/helper/system/config.h>
#include <SofaNonUniformFem/initNonUniformFEM.h>


namespace sofa
{

namespace component
{


void initNonUniformFEM()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

SOFA_LINK_CLASS(NonUniformHexahedralFEMForceFieldAndMass)
//SOFA_LINK_CLASS(NonUniformHexahedronFEMForceFieldDensity)
SOFA_LINK_CLASS(DynamicSparseGridGeometryAlgorithms)
SOFA_LINK_CLASS(DynamicSparseGridTopologyAlgorithms)
SOFA_LINK_CLASS(DynamicSparseGridTopologyContainer)
SOFA_LINK_CLASS(DynamicSparseGridTopologyModifier)
SOFA_LINK_CLASS(MultilevelHexahedronSetTopologyContainer)
SOFA_LINK_CLASS(SparseGridMultipleTopology)
SOFA_LINK_CLASS(SparseGridRamificationTopology)
#ifdef SOFA_HAVE_NEWMAT
SOFA_LINK_CLASS(HexahedronCompositeFEMForceFieldAndMass)
#endif
SOFA_LINK_CLASS(HexahedronCompositeFEMMapping)

} // namespace component

} // namespace sofa
