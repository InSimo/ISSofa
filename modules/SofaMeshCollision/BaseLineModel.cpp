/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

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
