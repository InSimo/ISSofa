/*******************************************************************************
*          Private SOFA components, (c) 2018 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_COMPONENT_COLLISION_GENERICTRIANGLEMODEL_INL
#define SOFA_COMPONENT_COLLISION_GENERICTRIANGLEMODEL_INL

#include "GenericTriangleModel.h"
#include "helpers.h"
#include <cassert>

namespace sofa
{

namespace component
{

namespace collision
{

template<class DataTypes>
GenericTriangleModel<DataTypes>::GenericTriangleModel()
    : d_boundaryAngleThreshold(initData(&d_boundaryAngleThreshold, Real(180), "boundaryAngleThreshold", "Angle threshold (in degrees) above which an edge or a point is qualified as boundary."))
//                                                                                                       0    -> All edges/points are flagged as boundary.
//                                                                                                       180  -> Only edges with a single adjacent triangle are marked as boundary
//                                                                                                               and only points attached to these boundary edges are marked as boundary."))
{
    enum_type = TRIANGLE_TYPE;

    d_boundaryAngleThreshold.setGroup("TriangleFlags_");
}

template<class DataTypes>
void GenericTriangleModel<DataTypes>::init()
{
    Inherit1::init();

    m_mstate = sofa::core::behavior::MechanicalState<DataTypes>::DynamicCast(this->getContext()->getMechanicalState());
    m_topology = this->getContext()->getMeshTopology();

    if (!m_mstate)
    {
        serr << "No MechanicalState found" << sendl;
    }
    if (!m_topology)
    {
        serr << "No BaseMeshTopology found" << sendl;
    }

    updateTopologicalTriangleFlags();
    updateMechanicalTriangleFlags();
}

template<class DataTypes>
void GenericTriangleModel<DataTypes>::resize(int size)
{
    m_triangleFlags.resize(size);
    Inherit1::resize(size);
}

template<class DataTypes>
void GenericTriangleModel<DataTypes>::handleTopologyChange(sofa::core::topology::Topology* t)
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

template<class DataTypes>
void GenericTriangleModel<DataTypes>::computeBoundingTree(int maxDepth)
{
    updateMechanicalTriangleFlags(); // TODO call this from a better place

    helpers::computeBoundingTree(this, maxDepth, m_hasTopologicalChange);
    m_hasTopologicalChange = false;
}

template<class DataTypes>
void GenericTriangleModel<DataTypes>::computeContinuousBoundingTree(double dt, int maxDepth)
{
    updateMechanicalTriangleFlags(); // TODO call this from a better place

    helpers::computeContinuousBoundingTree(this, maxDepth, dt, m_hasTopologicalChange);
    m_hasTopologicalChange = false;
}

template< class DataTypes>
typename DataTypes::Deriv computeTriangleNormal(const typename DataTypes::Coord& p0, const typename DataTypes::Coord& p1, const typename DataTypes::Coord& p2)
{
    typename DataTypes::Deriv n;
    DataTypes::setDPos(n,cross(DataTypes::getCPos(p1-p0), DataTypes::getCPos(p2-p0)).normalized());
    return n;
}

template<class DataTypes>
void GenericTriangleModel<DataTypes>::updateTopologicalTriangleFlags()
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

template<class DataTypes>
void GenericTriangleModel<DataTypes>::updateMechanicalTriangleFlags()
{
    using Deriv = typename DataTypes::Deriv;

    auto x0 = m_mstate->readRestPositions();
    auto x  = m_mstate->readPositions();

    // if the topology of the triangle mesh used for the collision is modified
    // and the rest position is not updated, we cannot update the edge border
    // flags value based on the angle made by the normal between adjacent triangles.
    if (x0.size() != x.size()) return;

    Real angleThreshold = d_boundaryAngleThreshold.getValue();
    angleThreshold *= M_PI / Real(180);
    const Real cosAngleThreshold = std::cos(angleThreshold);
    sofa::helper::vector<sofa::core::topology::BaseMeshTopology::EdgeID> boundaryEdges;

    const unsigned int ntris = m_topology->getNbTriangles();
    const auto& triangles = m_topology->getTriangles();

    for (sofa::core::topology::BaseMeshTopology::TriangleID tid = 0; tid < ntris; ++tid)
    {
        int& f = m_triangleFlags[tid];
        const sofa::core::topology::BaseMeshTopology::EdgesInTriangle& e = m_topology->getEdgesInTriangle(tid);

        for (unsigned int j=0; j<3; ++j)
        {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundEdge& tae = m_topology->getTrianglesAroundEdge(e[j]);
            if (tae[0] == tid)
            {
                if (tae.size() > 1) // if tae.size() == 1, FLAG_BE23 was already set by updateTopologicalTriangleFlags
                {
                    const sofa::core::topology::Topology::Triangle tri_0 = triangles[tae[0]];
                    const Deriv n_0 = computeTriangleNormal<DataTypes>(x0[tri_0[0]],
                                                                       x0[tri_0[1]],
                                                                       x0[tri_0[2]]);
                    for (std::size_t k=1; k<tae.size(); ++k)
                    {
                        const sofa::core::topology::Topology::Triangle tri_k = triangles[tae[k]];
                        const Deriv n_k = computeTriangleNormal<DataTypes>(x0[tri_k[0]],
                                                                           x0[tri_k[1]],
                                                                           x0[tri_k[2]]);
                        const Real cos = dot(DataTypes::getDPos(n_0), DataTypes::getDPos(n_k));
                        const bool isAngleAboveThreshold = cos <= cosAngleThreshold;
                        if (isAngleAboveThreshold)
                        {
                            f |= (FLAG_BE23 << j);
                            boundaryEdges.push_back(e[j]);
                        }
                    }
                }
            }
        }
    }

    // 2nd pass to set up boundary points according to boundary edges
    for (sofa::core::topology::BaseMeshTopology::TriangleID tid = 0; tid < ntris; ++tid)
    {
        const sofa::core::topology::BaseMeshTopology::Triangle& t = triangles[tid];
        int& f = m_triangleFlags[tid];
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
    }

    const auto& quads = m_topology->getQuads();

    // each quad [0,1,2,3] is split in two triangles: [1,2,0] and [3,0,2]
    for (sofa::core::topology::BaseMeshTopology::TriangleID tid = ntris; tid < m_triangleFlags.size(); ++tid)
    {
        sofa::core::topology::BaseMeshTopology::QuadID qid = (tid-ntris)/2;
        int tIndexInQuad = (tid-ntris)&1;
        int& f = m_triangleFlags[tid];
        const sofa::core::topology::BaseMeshTopology::EdgesInQuad eq = m_topology->getEdgesInQuad(qid);
        sofa::core::topology::BaseMeshTopology::EdgesInTriangle e;
        e[0] = sofa::core::topology::BaseMeshTopology::InvalidID;
        if (tIndexInQuad == 0)
        {
            e[1] = eq[3]; // 01
            e[2] = eq[0]; // 12
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
                if (qae.size() > 1) // if qae.size() == 1, FLAG_BE23 was already set by updateTopologicalTriangleFlags
                {
                    const sofa::core::topology::Topology::Quad q_0 = quads[qae[0]];
                    const Deriv n_0 = computeTriangleNormal<DataTypes>(x0[q_0[0]], x0[q_0[1]], x0[q_0[2]]);
                    for (std::size_t k=1; k<qae.size(); ++k)
                    {
                        const sofa::core::topology::Topology::Quad q_k = quads[qae[k]];
                        const Deriv n_k = computeTriangleNormal<DataTypes>(x0[q_k[0]], x0[q_k[1]], x0[q_k[2]]);
                        const Real cos = dot(DataTypes::getDPos(n_0), DataTypes::getDPos(n_k));
                        const bool isAngleAboveThreshold = cos < cosAngleThreshold;
                        if (isAngleAboveThreshold)
                        {
                            f |= (FLAG_BE23 << j);
                        }
                    }
                }
            }
        }
    }
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
