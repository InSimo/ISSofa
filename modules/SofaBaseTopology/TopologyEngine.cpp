/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
#ifndef SOFA_COMPONENT_TOPOLOGY_TOPOLOGYENGINE_CPP
#define SOFA_COMPONENT_TOPOLOGY_TOPOLOGYENGINE_CPP

#include <SofaBaseTopology/TopologyEngine.h>

#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/HexahedronSetTopologyContainer.h>


namespace sofa
{
namespace component
{
namespace topology
{

TopologyEngineImpl::TopologyEngineImpl(t_topologicalData *_topologicalData,
                                       sofa::core::topology::BaseMeshTopology *_topology,
                                       sofa::core::topology::TopologyHandler *_topoHandler) :
    m_topologicalData(_topologicalData),
    m_topology(NULL),
    m_topoHandler(_topoHandler),
    m_pointsLinked(false), m_edgesLinked(false), m_trianglesLinked(false),
    m_quadsLinked(false), m_tetrahedraLinked(false), m_hexahedraLinked(false)
{
    m_topology =  sofa::core::topology::TopologyContainer::DynamicCast(_topology);

    if (m_topology == NULL)
        serr <<"Error: Topology is not dynamic" << sendl;

    if (m_topoHandler == NULL)
        serr <<"Error: Topology Handler not available" << sendl;
}

void TopologyEngineImpl::init()
{
    // A pointData is by default child of positionSet Data
    //this->linkToPointDataArray();  // already done while creating engine

    // Name creation
    if (m_prefix.empty()) m_prefix = "TopologyEngine_";
    m_data_name = this->m_topologicalData->getName();
    this->addOutput(this->m_topologicalData);

    sofa::core::topology::TopologyEngine::init();

    // Register Engine in containter list
    //if (m_topology)
    //   m_topology->addTopologyEngine(this);
    //this->registerTopology(m_topology);
}


void TopologyEngineImpl::reinit()
{
    this->requestUpdate();
}


void TopologyEngineImpl::update()
{
#ifndef NDEBUG // too much warnings
    sout << "TopologyEngine::update" << sendl;
    sout<< "Number of topological changes: " << m_changeList.getValue().size() << sendl;
#endif
    this->cleanDirty();
    this->ApplyTopologyChanges();
}


void TopologyEngineImpl::registerTopology(sofa::core::topology::BaseMeshTopology *_topology)
{
    m_topology =  sofa::core::topology::TopologyContainer::DynamicCast(_topology);

    if (m_topology == NULL)
    {
#ifndef NDEBUG // too much warnings
        serr <<"Error: Topology is not dynamic" << sendl;
#endif
        return;
    }
    else
        m_topology->addTopologyEngine(this);
}


void TopologyEngineImpl::registerTopology()
{
    if (m_topology == NULL)
    {
#ifndef NDEBUG // too much warnings
        serr <<"Error: Topology is not dynamic" << sendl;
#endif
        return;
    }
    else
        m_topology->addTopologyEngine(this);
}


void TopologyEngineImpl::ApplyTopologyChanges()
{
    if(m_topoHandler)
    {
        m_topoHandler->ApplyTopologyChanges(m_changeList.getValue(), m_topology->getNbPoints());

        m_changeList.endEdit();
    }
}


/// Function to link DataEngine with Data array from topology
void TopologyEngineImpl::linkToPointDataArray()
{
    if (m_pointsLinked) // avoid second registration
        return;

    sofa::component::topology::PointSetTopologyContainer* _container = sofa::component::topology::PointSetTopologyContainer::DynamicCast(m_topology);

    if (_container == NULL)
    {
#ifndef NDEBUG
        serr <<"Error: Can't dynamic cast topology as PointSetTopologyContainer" << sendl;
#endif
        return;
    }

    (_container->getPointDataArray()).addOutput(this);
    m_pointsLinked = true;
}


void TopologyEngineImpl::linkToEdgeDataArray()
{
    if (m_edgesLinked) // avoid second registration
        return;

    sofa::component::topology::EdgeSetTopologyContainer* _container = sofa::component::topology::EdgeSetTopologyContainer::DynamicCast(m_topology);

    if (_container == NULL)
    {
#ifndef NDEBUG
        serr <<"Error: Can't dynamic cast topology as EdgeSetTopologyContainer" << sendl;
#endif
        return;
    }

    (_container->getEdgeDataArray()).addOutput(this);
    m_edgesLinked = true;
}


void TopologyEngineImpl::linkToTriangleDataArray()
{
    if (m_trianglesLinked) // avoid second registration
        return;

    sofa::component::topology::TriangleSetTopologyContainer* _container = sofa::component::topology::TriangleSetTopologyContainer::DynamicCast(m_topology);

    if (_container == NULL)
    {
#ifndef NDEBUG
        serr <<"Error: Can't dynamic cast topology as TriangleSetTopologyContainer" << sendl;
#endif
        return;
    }

    (_container->getTriangleDataArray()).addOutput(this);
    m_trianglesLinked = true;
}


void TopologyEngineImpl::linkToQuadDataArray()
{
    if (m_quadsLinked) // avoid second registration
        return;

    sofa::component::topology::QuadSetTopologyContainer* _container = sofa::component::topology::QuadSetTopologyContainer::DynamicCast(m_topology);

    if (_container == NULL)
    {
#ifndef NDEBUG
        serr <<"Error: Can't dynamic cast topology as QuadSetTopologyContainer" << sendl;
#endif
        return;
    }

    (_container->getQuadDataArray()).addOutput(this);
    m_quadsLinked = true;
}


void TopologyEngineImpl::linkToTetrahedronDataArray()
{
    if (m_tetrahedraLinked) // avoid second registration
        return;

    sofa::component::topology::TetrahedronSetTopologyContainer* _container = sofa::component::topology::TetrahedronSetTopologyContainer::DynamicCast(m_topology);

    if (_container == NULL)
    {
#ifndef NDEBUG
        serr <<"Error: Can't dynamic cast topology as TetrahedronSetTopologyContainer" << sendl;
#endif
        return;
    }

    (_container->getTetrahedronDataArray()).addOutput(this);
    m_tetrahedraLinked = true;
}


void TopologyEngineImpl::linkToHexahedronDataArray()
{
    if (m_hexahedraLinked) // avoid second registration
        return;

    sofa::component::topology::HexahedronSetTopologyContainer* _container = sofa::component::topology::HexahedronSetTopologyContainer::DynamicCast(m_topology);

    if (_container == NULL)
    {
#ifndef NDEBUG
        serr <<"Error: Can't dynamic cast topology as HexahedronSetTopologyContainer" << sendl;
#endif
        return;
    }

    (_container->getHexahedronDataArray()).addOutput(this);
    m_hexahedraLinked = true;
}


}// namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_TOPOLOGY_TOPOLOGYENGINE_CPP
