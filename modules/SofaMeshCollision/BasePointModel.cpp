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
