/*******************************************************************************
*          Private SOFA components, (c) 2019 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include "BasePointModel.h"
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_ABSTRACT_CLASS_IMPL((BasePointModel));

BasePointModel::BasePointModel()
{
    enum_type = POINT_TYPE;
}

BasePointModel::~BasePointModel()
{
}

void BasePointModel::init()
{
    Inherit1::init();

    m_topology = this->getContext()->getMeshTopology();

    if (!m_topology)
    {
        serr << "No BaseMeshTopology found" << sendl;
    }
}

bool BasePointModel::canCollideWithElement(int index, core::CollisionModel* model2, int index2)
{
    if (!this->getSelfCollision() || this->getContext() != model2->getContext())
    {
        // Not a self-collision
        return true;
    }

    if (model2 == this)
    {
        if (index <= index2) // we only consider the case when index > index2 in order to avoid computing the same self-collision twice
        {
            return false;
        }

        // do not collide if a common point is found in the neighborhood
        const helper::vector<unsigned int>& verticesAroundVertex1 = this->m_topology->getVerticesAroundVertex(index);
        const helper::vector<unsigned int>& verticesAroundVertex2 = this->m_topology->getVerticesAroundVertex(index2);

        for (unsigned int v1 : verticesAroundVertex1)
        {
            for (unsigned int v2 : verticesAroundVertex2)
            {
                if (v1 == v2 || static_cast<int>(v1) == index2 || static_cast<int>(v2) == index)
                {
                    return false;
                }
            }
        }
        return true;
    }
    else
    {
        return model2->canCollideWithElement(index2, this, index);
    }
}

void BasePointModel::handleTopologyChange(sofa::core::topology::Topology* t)
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
            resize(m_topology->getNbPoints());
            break;
        }
        default: break;
        }
    }
}

} // namespace collision

} // namespace component

} // namespace sofa
