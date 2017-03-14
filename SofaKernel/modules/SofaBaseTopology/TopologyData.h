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
#ifndef SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATA_H
#define SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATA_H
#include "config.h"

#include <sofa/helper/vector.h>


#include <sofa/core/topology/BaseTopologyData.h>
#include <SofaBaseTopology/TopologyDataHandler.h>
#include <SofaBaseTopology/TopologyEngine.h>


namespace sofa
{

namespace component
{

namespace topology
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Generic Topology Data Implementation   /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief A class for storing topology related data. Automatically manages topology changes.
*
* This class is a wrapper of class helper::vector that is made to take care transparently of all topology changes that might
* happen (non exhaustive list: element added, removed, fused, renumbered).
*/
template< class TopologyElementType, class ContainerType>
class TopologyDataImpl : public sofa::core::topology::BaseTopologyData<ContainerType>
{

public:

    /// Constructor
    TopologyDataImpl( const typename sofa::core::topology::BaseTopologyData< ContainerType >::InitData& data)
        : sofa::core::topology::BaseTopologyData< ContainerType >(data),
          m_topologyHandler(NULL),
          m_deleteTopologyHandler(false)
    {}

    virtual ~TopologyDataImpl();


    /** Public functions to handle topological engine creation */
    /// To create topological engine link to this Data. Pointer to current topology is needed.
    virtual void createTopologicalEngine(sofa::core::topology::BaseMeshTopology* _topology, sofa::component::topology::TopologyDataHandler<TopologyElementType,ContainerType>* _topologyHandler, bool deleteHandler = false);

    /** Public functions to handle topological engine creation */
    /// To create topological engine link to this Data. Pointer to current topology is needed.
    virtual void createTopologicalEngine(sofa::core::topology::BaseMeshTopology* _topology);

    /// Allow to add additionnal dependencies to others Data.
    void addInputData(sofa::core::objectmodel::BaseData* _data);

    /// Function to link the topological Data with the engine and the current topology. And init everything.
    /// This function should be used at the end of the all declaration link to this Data while using it in a component.
    void registerTopologicalData();

    /// Link Data to topology arrays
    void linkToPointDataArray();
    void linkToEdgeDataArray();
    void linkToTriangleDataArray();
    void linkToQuadDataArray();
    void linkToTetrahedronDataArray();
    void linkToHexahedronDataArray();

    sofa::component::topology::TopologyDataHandler<TopologyElementType,ContainerType>* getTopologyHandler()
    {
        return m_topologyHandler;
    }

protected:

    sofa::component::topology::TopologyDataHandler<TopologyElementType,ContainerType>* m_topologyHandler;
    bool m_deleteTopologyHandler;

    void linkToElementDataArray(sofa::core::topology::BaseMeshTopology::Point*      ) { linkToPointDataArray();       }
    void linkToElementDataArray(sofa::core::topology::BaseMeshTopology::Edge*       ) { linkToEdgeDataArray();        }
    void linkToElementDataArray(sofa::core::topology::BaseMeshTopology::Triangle*   ) { linkToTriangleDataArray();    }
    void linkToElementDataArray(sofa::core::topology::BaseMeshTopology::Quad*       ) { linkToQuadDataArray();        }
    void linkToElementDataArray(sofa::core::topology::BaseMeshTopology::Tetrahedron*) { linkToTetrahedronDataArray(); }
    void linkToElementDataArray(sofa::core::topology::BaseMeshTopology::Hexahedron* ) { linkToHexahedronDataArray();  }

};

// c++11 power!
template< class ContainerType > using PointData       = TopologyDataImpl<sofa::core::topology::BaseMeshTopology::Point      , ContainerType>;
template< class ContainerType > using EdgeData        = TopologyDataImpl<sofa::core::topology::BaseMeshTopology::Edge       , ContainerType>;
template< class ContainerType > using TriangleData    = TopologyDataImpl<sofa::core::topology::BaseMeshTopology::Triangle   , ContainerType>;
template< class ContainerType > using QuadData        = TopologyDataImpl<sofa::core::topology::BaseMeshTopology::Quad       , ContainerType>;
template< class ContainerType > using TetrahedronData = TopologyDataImpl<sofa::core::topology::BaseMeshTopology::Tetrahedron, ContainerType>;
template< class ContainerType > using HexahedronData  = TopologyDataImpl<sofa::core::topology::BaseMeshTopology::Hexahedron , ContainerType>;

} // namespace topology

} // namespace component

} // namespace sofa


#endif // SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATA_H
