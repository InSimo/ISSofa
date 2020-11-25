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
#include "BaseLineModel.h"
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_ABSTRACT_CLASS_IMPL((BaseLineModel));

BaseLineModel::BaseLineModel()
{
    enum_type = LINE_TYPE;
}

BaseLineModel::~BaseLineModel()
{
}

void BaseLineModel::init()
{
    Inherit1::init();

    m_topology = this->getContext()->getMeshTopology();

    if (!m_topology)
    {
        serr << "No BaseMeshTopology found" << sendl;
    }

    updateTopologicalLineFlags();
}


void BaseLineModel::resize(int size)
{
    m_lineFlags.resize(size);
    Inherit1::resize(size);
}

bool BaseLineModel::canCollideWithElement(int index, CollisionModel* model2, int index2)
{
    if (!this->getSelfCollision() || this->getContext() != model2->getContext())
    {
        // Not a self-collision
        return true;
    }

    const core::topology::Topology::Edge& e1 = this->m_topology->getEdge(index);
    int p11 = e1[0];
    int p12 = e1[1];

    if (model2->getEnumType() == sofa::core::CollisionModel::LINE_TYPE)
    {
        // do not collide if the edges have a point in common
        const core::topology::Topology::Edge& e2 = this->m_topology->getEdge(index2);
        int p21 = e2[0];
        int p22 = e2[1];

        if (p11 == p21 || p11 == p22 || p12 == p21 || p12 == p22)
            return false;

        return true;
    }
    else if (model2->getEnumType() == sofa::core::CollisionModel::POINT_TYPE)
    {
        // do not collide if the point belongs to the edge
        if (index2 == p11 || index2 == p12)
            return false;

        return true;
    }
    else
    {
        return model2->canCollideWithElement(index2, this, index);
    }
}

void BaseLineModel::handleTopologyChange(sofa::core::topology::Topology* t)
{
    sofa::core::topology::BaseMeshTopology* topology = sofa::core::topology::BaseMeshTopology::DynamicCast(t);
    assert(topology != nullptr);

    if (t != m_topology) return;

    for (auto itBegin = topology->beginChange(), itEnd = topology->endChange(); itBegin != itEnd; ++itBegin)
    {
        core::topology::TopologyChangeType changeType = (*itBegin)->getChangeType();
        switch (changeType)
        {
        case core::topology::ENDING_EVENT:
        {
            m_hasTopologicalChange = true;
            resize(m_topology->getNbEdges());
            updateTopologicalLineFlags();
            break;
        }
        default: break;
        }
    }
}


void BaseLineModel::updateTopologicalLineFlags()
{
    using EdgeID = sofa::core::topology::Topology::EdgeID;

    if (!this->m_hasTopologicalChange) return;

    m_lineFlags.resize(m_topology->getNbEdges());

    for (EdgeID eid(0); eid < EdgeID(m_topology->getNbEdges()); ++eid)
    {
        int f = 0;
        const auto& e = m_topology->getEdge(eid);
        const sofa::core::topology::BaseMeshTopology::EdgesAroundVertex& eav0 = m_topology->getEdgesAroundVertex(e[0]);
        const sofa::core::topology::BaseMeshTopology::EdgesAroundVertex& eav1 = m_topology->getEdgesAroundVertex(e[1]);
        if (eav0[0] == eid)
        {
            f |= FLAG_P1;
        }
        if (eav1[0] == eid)
        {
            f |= FLAG_P2;
        }
        if (eav0.size() == 1)
        {
            f |= FLAG_BP1;
        }
        if (eav1.size() == 1)
        {
            f |= FLAG_BP2;
        }
        m_lineFlags[eid] = f;
    }
}

} // namespace collision

} // namespace component

} // namespace sofa
