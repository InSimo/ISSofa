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
#ifndef SOFA_COMPONENT_TOPOLOGY_CYLINDERGRIDTOPOLOGY_H
#define SOFA_COMPONENT_TOPOLOGY_CYLINDERGRIDTOPOLOGY_H

#include <SofaBaseTopology/CubeTopology.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa
{

namespace component
{

namespace topology
{

class SOFA_BASE_TOPOLOGY_API CylinderGridTopology : public CubeTopology
{
public:
    SOFA_CLASS(CylinderGridTopology,CubeTopology);
    typedef sofa::defaulttype::Vector3 Vector3;
protected:
    CylinderGridTopology(int nx, int ny, int nz);
    CylinderGridTopology();
public:
    Vector3 getPoint(int x, int y, int z) const;

protected:
    Data< Vector3 > center;
    Data< Vector3 > axis;
    Data< SReal > radius, length;
};

} // namespace topology

} // namespace component

} // namespace sofa

#endif
