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
#ifndef SOFA_COMPONENT_TOPOLOGY_TOPOLOGYSPARSEDATA_H
#define SOFA_COMPONENT_TOPOLOGY_TOPOLOGYSPARSEDATA_H

#include <sofa/helper/map.h>

#include <sofa/core/topology/BaseTopologyData.h>
#include <SofaBaseTopology/TopologyEngine.h>
#include <SofaBaseTopology/TopologySparseDataHandler.h>

#include <sofa/core/topology/BaseMeshTopology.h>
#include <map>

namespace sofa
{

namespace component
{

namespace topology
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Generic Topology Data Implementation   /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

template< class TopologyElementType, class VecT>
class TopologySparseDataImpl : public sofa::core::topology::BaseTopologyData<VecT>
{

public:
    typedef VecT container_type;
    typedef typename container_type::value_type value_type;

    /// size_type
    typedef typename container_type::size_type size_type;
    /// reference to a value (read-write)
    typedef typename container_type::reference reference;
    /// const reference to a value (read only)
    typedef typename container_type::const_reference const_reference;
    /// const iterator
    typedef typename container_type::const_iterator const_iterator;


    /// Constructor
    TopologySparseDataImpl( const typename sofa::core::topology::BaseTopologyData< VecT >::InitData& data)
        : sofa::core::topology::BaseTopologyData< VecT >(data),
          m_topologicalEngine(NULL),
          m_topology(NULL),
          m_topologyHandler(NULL),
          m_isConcerned(false)
    {}

    virtual ~TopologySparseDataImpl();

    /** Public functions to handle topological engine creation */
    /// To create topological engine link to this Data. Pointer to current topology is needed.
    virtual void createTopologicalEngine(sofa::core::topology::BaseMeshTopology* _topology, sofa::core::topology::TopologyHandler* _topologyHandler);

    /** Public functions to handle topological engine creation */
    /// To create topological engine link to this Data. Pointer to current topology is needed.
    virtual void createTopologicalEngine(sofa::core::topology::BaseMeshTopology* _topology);

    /// Allow to add additionnal dependencies to others Data.
    void addInputData(sofa::core::objectmodel::BaseData* _data);

    /// Function to link the topological Data with the engine and the current topology. And init everything.
    /// This function should be used at the end of the all declaration link to this Data while using it in a component.
    void registerTopologicalData();


    value_type& operator[](int i)
    {
        container_type& data = *(this->beginEdit());
        value_type& result = data[i];
        this->endEdit();
        return result;
    }


    /// Link Data to topology arrays
    void linkToPointDataArray();
    void linkToEdgeDataArray();
    void linkToTriangleDataArray();
    void linkToQuadDataArray();
    void linkToTetrahedronDataArray();
    void linkToHexahedronDataArray();

    void setMap2Elements(const sofa::helper::vector<unsigned int> _map2Elements)
    {
        m_map2Elements = _map2Elements;
    }

    sofa::helper::vector<unsigned int>& getMap2Elements() {return m_map2Elements;}

    bool getSparseDataStatus() {return m_isConcerned;}

    void activateSparseData() {m_isConcerned = true;}
    void desactivateSparseData() {m_isConcerned = false;}


protected:

    virtual void createTopologyHandler() {}

    sofa::helper::vector<unsigned int> m_map2Elements; // same size as SparseData but contain id of Triangle link to each data[]

    typename sofa::component::topology::TopologyEngineImpl::SPtr m_topologicalEngine;
    sofa::core::topology::BaseMeshTopology* m_topology;
    sofa::component::topology::TopologySparseDataHandler<TopologyElementType,VecT>* m_topologyHandler;

    bool m_isConcerned;

    void linkToElementDataArray(Point*      ) { linkToPointDataArray();       }
    void linkToElementDataArray(Edge*       ) { linkToEdgeDataArray();        }
    void linkToElementDataArray(Triangle*   ) { linkToTriangleDataArray();    }
    void linkToElementDataArray(Quad*       ) { linkToQuadDataArray();        }
    void linkToElementDataArray(Tetrahedron*) { linkToTetrahedronDataArray(); }
    void linkToElementDataArray(Hexahedron* ) { linkToHexahedronDataArray();  }

};


////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////   Point Topology Data Implementation   /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

template< class VecT >
class PointSparseData : public TopologySparseDataImpl<Point, VecT>
{
public:
    PointSparseData( const typename sofa::core::topology::BaseTopologyData< VecT >::InitData& data)
        : TopologySparseDataImpl<Point, VecT>(data)
    {}
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////   Edge Topology Data Implementation   /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

template< class VecT >
class EdgeSparseData : public TopologySparseDataImpl<Edge, VecT>
{
public:
    EdgeSparseData( const typename sofa::core::topology::BaseTopologyData< VecT >::InitData& data)
        : TopologySparseDataImpl<Edge, VecT>(data)
    {}
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////   Triangle Topology Data Implementation   ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

template< class VecT >
class TriangleSparseData : public TopologySparseDataImpl<Triangle, VecT>
{
public:
    TriangleSparseData( const typename sofa::core::topology::BaseTopologyData< VecT >::InitData& data)
        : TopologySparseDataImpl<Triangle, VecT>(data)
    {}
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////   Quad Topology Data Implementation   /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

template< class VecT >
class QuadSparseData : public TopologySparseDataImpl<Quad, VecT>
{
public:
    QuadSparseData( const typename sofa::core::topology::BaseTopologyData< VecT >::InitData& data)
        : TopologySparseDataImpl<Quad, VecT>(data)
    {}
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Tetrahedron Topology Data Implementation   /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

template< class VecT >
class TetrahedronSparseData : public TopologySparseDataImpl<Tetrahedron, VecT>
{
public:
    TetrahedronSparseData( const typename sofa::core::topology::BaseTopologyData< VecT >::InitData& data)
        : TopologySparseDataImpl<Tetrahedron, VecT>(data)
    {}
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////   Hexahedron Topology Data Implementation   /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

template< class VecT >
class HexahedronSparseData : public TopologySparseDataImpl<Hexahedron, VecT>
{
public:
    HexahedronSparseData( const typename sofa::core::topology::BaseTopologyData< VecT >::InitData& data)
        : TopologySparseDataImpl<Hexahedron, VecT>(data)
    {}
};


} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_TOPOLOGY_TOPOLOGYSPARSEDATA_H
