/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_TOPOLOGY_BASETOPOLOGYDATA_H
#define SOFA_COMPONENT_TOPOLOGY_BASETOPOLOGYDATA_H

#include <sofa/core/topology/Topology.h>
#include <sofa/core/topology/BaseTopologyEngine.h>

namespace sofa
{

namespace core
{

namespace topology
{



/** A class that define topological Data general methods
*/

template < class ContainerType = void* >
class BaseTopologyData : public sofa::core::objectmodel::Data <ContainerType>
{
public:
    //SOFA_CLASS(SOFA_TEMPLATE2(BaseTopologyData,ContainerType,VecT), SOFA_TEMPLATE(sofa::core::objectmodel::Data, ContainerType));


    class InitData : public sofa::core::objectmodel::BaseData::BaseInitData
    {
    public:
        InitData() : value(ContainerType()) {}
        InitData(const ContainerType& v) : value(v) {}
        InitData(const sofa::core::objectmodel::BaseData::BaseInitData& i) : sofa::core::objectmodel::BaseData::BaseInitData(i), value(ContainerType()) {}

        ContainerType value;
    };

    /** \copydoc Data(const BaseData::BaseInitData&) */
    explicit BaseTopologyData(const sofa::core::objectmodel::BaseData::BaseInitData& init)
        : Data<ContainerType>(init),
        m_topologicalEngine(NULL),
        m_topology(NULL)
    {
    }

    /** \copydoc Data(const InitData&) */
    explicit BaseTopologyData(const InitData& init)
        : Data<ContainerType>(init),
        m_topologicalEngine(NULL),
        m_topology(NULL)
    {
    }


    /** \copydoc Data(const char*, bool, bool) */
    BaseTopologyData( const char* helpMsg=0, bool isDisplayed=true, bool isReadOnly=false)
        : Data<ContainerType>(helpMsg, isDisplayed, isReadOnly),
        m_topologicalEngine(NULL),
        m_topology(NULL)
    {

    }

    /** \copydoc Data(const ContainerType&, const char*, bool, bool) */
    BaseTopologyData( const ContainerType& value, const char* helpMsg=0, bool isDisplayed=true, bool isReadOnly=false)
        : Data<ContainerType>(helpMsg, isDisplayed, isReadOnly),
        m_topologicalEngine(NULL),
        m_topology(NULL)
    {
    }


    // Generic methods to apply changes on the Data
    //{
    /// Apply adding points elements.
    virtual void applyCreatePointFunction(const sofa::helper::vector<unsigned int>& ) {}
    /// Apply removing points elements.
    virtual void applyDestroyPointFunction(const sofa::helper::vector<unsigned int>& ) {}

    /// Apply adding edges elements.
    virtual void applyCreateEdgeFunction(const sofa::helper::vector<unsigned int>& ) {}
    /// Apply removing edges elements.
    virtual void applyDestroyEdgeFunction(const sofa::helper::vector<unsigned int>& ) {}

    /// Apply adding triangles elements.
    virtual void applyCreateTriangleFunction(const sofa::helper::vector<unsigned int>& ) {}
    /// Apply removing triangles elements.
    virtual void applyDestroyTriangleFunction(const sofa::helper::vector<unsigned int>& ) {}

    /// Apply adding quads elements.
    virtual void applyCreateQuadFunction(const sofa::helper::vector<unsigned int>& ) {}
    /// Apply removing quads elements.
    virtual void applyDestroyQuadFunction(const sofa::helper::vector<unsigned int>& ) {}

    /// Apply adding tetrahedra elements.
    virtual void applyCreateTetrahedronFunction(const sofa::helper::vector<unsigned int>& ) {}
    /// Apply removing tetrahedra elements.
    virtual void applyDestroyTetrahedronFunction(const sofa::helper::vector<unsigned int>& ) {}

    /// Apply adding hexahedra elements.
    virtual void applyCreateHexahedronFunction(const sofa::helper::vector<unsigned int>& ) {}
    /// Apply removing hexahedra elements.
    virtual void applyDestroyHexahedronFunction(const sofa::helper::vector<unsigned int>& ) {}
    //}

    sofa::core::topology::TopologyEngine* getTopologicalEngine()
    {
        return m_topologicalEngine.get();
    }

    sofa::core::topology::BaseMeshTopology* getTopology()
    {
        return m_topology;
    }

protected:
    typename sofa::core::topology::TopologyEngine::SPtr    m_topologicalEngine;
    sofa::core::topology::BaseMeshTopology*                         m_topology;

};


} // namespace topology

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_TOPOLOGY_BASETOPOLOGYDATA_H
