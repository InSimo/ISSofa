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
#ifndef SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATA_INL
#define SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATA_INL

#include <SofaBaseTopology/TopologyData.h>
#include <SofaBaseTopology/TopologyDataHandler.inl>

namespace sofa
{

namespace component
{

namespace topology
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Generic Topology Data Implementation   /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename TopologyElementType, typename ContainerType>
TopologyDataImpl <TopologyElementType, ContainerType>::~TopologyDataImpl()
{
    if (m_deleteTopologyHandler && m_topologyHandler)
        delete m_topologyHandler;
}


template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::createTopologicalEngine(sofa::core::topology::BaseMeshTopology *_topology, sofa::component::topology::TopologyDataHandler<TopologyElementType,ContainerType>* _topologyHandler, bool deleteHandler)
{
    this->m_topology = _topology;
    if (_topology && sofa::core::topology::TopologyContainer::DynamicCast(_topology))
    {
        this->m_topologyHandler = _topologyHandler;
        this->m_deleteTopologyHandler = deleteHandler;
        this->m_topologicalEngine = sofa::core::objectmodel::New<TopologyEngineImpl>((sofa::component::topology::TopologyDataImpl<TopologyElementType, ContainerType>*)this, _topology, _topologyHandler);
        this->m_topologicalEngine->setNamePrefix(std::string(sofa::core::topology::TopologyElementTypeInfo<TopologyElementType>::name()) + std::string("Engine_"));
        if (this->getOwner() && sofa::core::objectmodel::BaseObject::DynamicCast(this->getOwner())) sofa::core::objectmodel::BaseObject::DynamicCast(this->getOwner())->addSlave(this->m_topologicalEngine.get());
        this->m_topologicalEngine->init();
        if (this->m_topologyHandler)
        {
            this->m_topologyHandler->init();
        }
        this->linkToElementDataArray((TopologyElementType*)NULL);
        this->getOwner()->sout<<"TopologyDataImpl: " << this->getName() << " initialized with dynamic " << _topology->getClassName() << "Topology." << this->getOwner()->sendl;
    }
    else if (_topology)
    {
        this->getOwner()->sout<<"TopologyDataImpl: " << this->getName() << " initialized with static " << _topology->getClassName() << " Topology." << this->getOwner()->sendl;
        if (deleteHandler && _topologyHandler) delete _topologyHandler;
    }
    else
    {
        this->getOwner()->sout<<"TopologyDataImpl: No Topology given to " << this->getName() << " to createTopologicalEngine. Topological changes will be disabled." << this->getOwner()->sendl;
        if (deleteHandler && _topologyHandler) delete _topologyHandler;
    }
}


template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::createTopologicalEngine(sofa::core::topology::BaseMeshTopology *_topology)
{
    createTopologicalEngine(_topology, new TopologyDataHandler<TopologyElementType, ContainerType>(this), true);
}


template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::registerTopologicalData()
{
    if (this->m_topologicalEngine)
    {
        this->m_topologicalEngine->registerTopology();
    }
    else if (!this->m_topology)
    {
        this->getOwner()->serr << "TopologyDataImpl: " << this->getName() << " has no engine. Topological changes will be disabled. Use createTopologicalEngine method before registerTopologicalData to allow topological changes." << this->getOwner()->sendl;
    }
}

template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::addInputData(sofa::core::objectmodel::BaseData *_data)
{
    if (this->m_topologicalEngine)
    {
        this->m_topologicalEngine->addInput(_data);
    }
    else if (!this->m_topology)
    {
        this->getOwner()->serr << "Warning: TopologyDataImpl: " << this->getName() << " has no engine. Use createTopologicalEngine function before addInputData." << this->getOwner()->sendl;
    }
}


/// Method used to link Data to point Data array, using the engine's method
template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::linkToPointDataArray()
{
    if(this->m_topologicalEngine)
        this->m_topologicalEngine->linkToPointDataArray();
}

/// Method used to link Data to edge Data array, using the engine's method
template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::linkToEdgeDataArray()
{
    if(this->m_topologicalEngine)
        this->m_topologicalEngine->linkToEdgeDataArray();
}

/// Method used to link Data to triangle Data array, using the engine's method
template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::linkToTriangleDataArray()
{
    if(this->m_topologicalEngine)
        this->m_topologicalEngine->linkToTriangleDataArray();
}

/// Method used to link Data to quad Data array, using the engine's method
template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::linkToQuadDataArray()
{
    if(this->m_topologicalEngine)
        this->m_topologicalEngine->linkToQuadDataArray();
}

/// Method used to link Data to tetrahedron Data array, using the engine's method
template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::linkToTetrahedronDataArray()
{
    if(this->m_topologicalEngine)
        this->m_topologicalEngine->linkToTetrahedronDataArray();
}

/// Method used to link Data to hexahedron Data array, using the engine's method
template <typename TopologyElementType, typename ContainerType>
void TopologyDataImpl <TopologyElementType, ContainerType>::linkToHexahedronDataArray()
{
    if(this->m_topologicalEngine)
        this->m_topologicalEngine->linkToHexahedronDataArray();
}


} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATA_INL
