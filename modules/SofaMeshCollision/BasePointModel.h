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
#ifndef SOFA_COMPONENT_COLLISION_BASEPOINTMODEL_H
#define SOFA_COMPONENT_COLLISION_BASEPOINTMODEL_H

#include "initMeshCollision.h"
#include <sofa/core/CollisionModel.h>

namespace sofa
{

namespace component
{

namespace collision
{

class SOFA_MESH_COLLISION_API BasePointModel : public sofa::core::CollisionModel
{
public:
    SOFA_ABSTRACT_CLASS_EXTERNAL((BasePointModel), ((sofa::core::CollisionModel)));

    void init() override;

    bool canCollideWithElement(int index, core::CollisionModel* model2, int index2) override;

    void handleTopologyChange(sofa::core::topology::Topology* t) override;
    sofa::core::topology::BaseMeshTopology* getTopology() const final { return this->m_topology; }

protected:
    BasePointModel();

    virtual ~BasePointModel();

    sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

    bool m_hasTopologicalChange = true;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
