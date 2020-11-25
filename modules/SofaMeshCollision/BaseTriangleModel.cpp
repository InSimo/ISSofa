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
#include "BaseTriangleModel.h"
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_ABSTRACT_CLASS_IMPL((BaseTriangleModel));

BaseTriangleModel::BaseTriangleModel()
    : d_intersectOnlyBoundaryEdges(initData(&d_intersectOnlyBoundaryEdges, false, "intersectOnlyBoundaryEdges", "If true, compute intersections only with boundary edges instead of all edges"))
{
    enum_type = TRIANGLE_TYPE;

    d_intersectOnlyBoundaryEdges.setGroup("IntersectionParams");
}

BaseTriangleModel::~BaseTriangleModel()
{
}

void BaseTriangleModel::init()
{
    Inherit1::init();

    m_topology = this->getContext()->getMeshTopology();

    if (!m_topology)
    {
        serr << "No BaseMeshTopology found" << sendl;
    }

    updateTopologicalTriangleFlags();
}

void BaseTriangleModel::resize(int size)
{
    m_triangleFlags.resize(size);
    Inherit1::resize(size);
}

bool BaseTriangleModel::canCollideWithElement(int index, core::CollisionModel* model2, int index2)
{
    if (!this->getSelfCollision() || this->getContext() != model2->getContext())
    {
        // Not a self-collision
        return true;
    }

    const core::topology::Topology::Triangle& t1 = this->m_topology->getTriangle(index);
    int p11 = t1[0];
    int p12 = t1[1];
    int p13 = t1[2];

    if (model2->getEnumType() == sofa::core::CollisionModel::TRIANGLE_TYPE)
    {
        // do not collide if the triangles have a point in common
        const core::topology::Topology::Triangle& t2 = this->m_topology->getTriangle(index2);
        int p21 = t2[0];
        int p22 = t2[1];
        int p23 = t2[2];

        if (p11 == p21 || p11 == p22 || p11 == p23 ||
            p12 == p21 || p12 == p22 || p12 == p23 ||
            p13 == p21 || p13 == p22 || p13 == p23)
            return false;

        return true;
    }
    else if (model2->getEnumType() == sofa::core::CollisionModel::LINE_TYPE)
    {
        // do not collide if the edge shares a point with the triangle
        const core::topology::BaseMeshTopology::Edge& e2 = this->m_topology->getEdge(index2);
        int e21 = e2[0];
        int e22 = e2[1];

        if (p11 == e21 || p11 == e22 || p12 == e21 || p12 == e22)
            return false;

        return true;
    }
    else if (model2->getEnumType() == sofa::core::CollisionModel::POINT_TYPE)
    {
        // do not collide if the point belongs to the triangle
        if (index2 == p11 || index2 == p12 || index2 == p13)
            return false;

        return true;
    }

    return true;
}

void BaseTriangleModel::handleTopologyChange(sofa::core::topology::Topology* t)
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
            resize(m_topology->getNbTriangles());
            updateTopologicalTriangleFlags();
            break;
        }
        default: break;
        }
    }
}

void BaseTriangleModel::updateTopologicalTriangleFlags()
{
    if (!this->m_hasTopologicalChange) return;

    sofa::helper::vector<sofa::core::topology::BaseMeshTopology::EdgeID> boundaryEdges;

    const unsigned int ntris = m_topology->getNbTriangles();
    const unsigned int nquads = m_topology->getNbQuads();
    m_triangleFlags.resize(ntris + 2 * nquads);

    const auto& triangles = m_topology->getTriangles();

    for (sofa::core::topology::BaseMeshTopology::TriangleID tid = 0; tid< ntris; ++tid)
    {
        const sofa::core::topology::BaseMeshTopology::Triangle& t = triangles[tid];
        int f = 0;
        for (unsigned int j=0; j<3; ++j)
        {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex& tav = m_topology->getTrianglesAroundVertex(t[j]);
            if (tav[0] == tid)
                f |= (FLAG_P1 << j);
            if (tav.size() == 1)
                f |= (FLAG_BP1 << j);
        }

        const sofa::core::topology::BaseMeshTopology::EdgesInTriangle& e = m_topology->getEdgesInTriangle(tid);

        for (unsigned int j=0; j<3; ++j)
        {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundEdge& tae = m_topology->getTrianglesAroundEdge(e[j]);
            if (tae[0] == tid)
            {
                f |= (FLAG_E23 << j);

                if (tae.size() == 1)
                {
                    f |= (FLAG_BE23 << j);
                    boundaryEdges.push_back(e[j]);
                }
            }
        }
        m_triangleFlags[tid] = f;
    }

    // 2nd pass to set up boundary points according to boundary edges
    for (sofa::core::topology::BaseMeshTopology::TriangleID tid = 0; tid < ntris; ++tid)
    {
        const sofa::core::topology::BaseMeshTopology::Triangle& t = triangles[tid];
        int f = m_triangleFlags[tid];
        for (unsigned int i = 0; i < 3; ++i)
        {
            if (!(f&FLAG_P1 << i)) continue; // this point is not attached to the triangle
            if (f&FLAG_BP1 << i) continue; // already classified as a boundary point

            const sofa::core::topology::BaseMeshTopology::EdgesAroundVertex& eav = m_topology->getEdgesAroundVertex(t[i]);

            // a point is a boundary if at least one adjacent edge is a boundary
            for (auto eid : eav)
            {
                if (std::find(boundaryEdges.begin(), boundaryEdges.end(), eid) != boundaryEdges.end())
                {
                    f |= (FLAG_BP1 << i);
                    break;
                }
            }
        }
        m_triangleFlags[tid] = f;
    }

    // each quad [0,1,2,3] is split in two triangles: [1,2,0] and [3,0,2]
    for (sofa::core::topology::BaseMeshTopology::TriangleID tid = ntris; tid < m_triangleFlags.size(); ++tid)
    {
        sofa::core::topology::BaseMeshTopology::QuadID qid = (tid-ntris)/2;
        int tIndexInQuad = (tid-ntris)&1;
        const sofa::core::topology::BaseMeshTopology::Triangle& t = triangles[tid];
        int f = 0;
        // only look at the first 2 vertices of the triangles, covering all 4 quad vertices in the pair of triangles
        for (unsigned int j=0; j<2; ++j)
        {
            if (m_topology->getTrianglesAroundVertex(t[j]).empty() && m_topology->getQuadsAroundVertex(t[j])[0] == qid)
                f |= (FLAG_P1 << j);
        }

        const sofa::core::topology::BaseMeshTopology::EdgesInQuad eq = m_topology->getEdgesInQuad(qid);
        sofa::core::topology::BaseMeshTopology::EdgesInTriangle e;
        e[0] = sofa::core::topology::BaseMeshTopology::InvalidID;
        if (tIndexInQuad == 0)
        {
            e[1] = eq[3]; // 01
            e[2] = eq[0]; // 12
            f |= (FLAG_E23 << 0); // we arbitrarly associate the diagonal edge to the first triangle in the quad
        }
        else
        {
            e[1] = eq[1]; // 23
            e[2] = eq[2]; // 30
        }

        // skip the first edge of each triangle, which is the quad diagonal
        for (unsigned int j=1; j<3; ++j)
        {
            const sofa::core::topology::BaseMeshTopology::QuadsAroundEdge& qae = m_topology->getQuadsAroundEdge(e[j]);
            if (qae[0] == qid)
            {
                f |= (FLAG_E23 << j);
                if (qae.size() == 1)
                {
                    f |= (FLAG_BE23 << j);
                }
            }
        }
        m_triangleFlags[tid] = f;
    }
}

} // namespace collision

} // namespace component

} // namespace sofa 
