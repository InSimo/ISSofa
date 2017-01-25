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
#ifndef SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATAHANDLER_H
#define SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATAHANDLER_H

#include <sofa/core/topology/TopologyElementHandler.h>
#include <sofa/core/topology/BaseTopologyData.h>
#include <sofa/helper/map.h>
#include <sofa/defaulttype/DataTypeInfo.h>


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
* happen (non exhaustive list: elements added, removed, fused, renumbered).
*/

template< class TopologyElementType, class ContainerT>
class TopologyDataHandler : public sofa::core::topology::TopologyElementHandler< TopologyElementType >
{
public:
    enum { InvalidID = (unsigned)-1 };
    typedef ContainerT                                  ContainerType;
    typedef defaulttype::DataTypeInfo<ContainerType>    DataTypeInfo;
    typedef typename DataTypeInfo::KeyType              KeyType; 
    typedef typename DataTypeInfo::MappedType           MappedType;

    typedef typename core::topology::TopologyElementInfo<TopologyElementType>    TopologyElementInfoT;
    typedef typename core::topology::TopologyObjectType                          TopologyObjectType;

    typedef typename sofa::core::topology::TopologyElementHandler< TopologyElementType > Inherit;
    typedef typename Inherit::AncestorElem AncestorElem;

protected:
    sofa::core::topology::BaseTopologyData <ContainerType>* m_topologyData;
    MappedType* m_defaultValue; // default value when adding an element (or NULL if not specified)

public:

    // Constructors
    TopologyDataHandler(sofa::core::topology::BaseTopologyData <ContainerType>* _topologyData);
    TopologyDataHandler(sofa::core::topology::BaseTopologyData <ContainerType>* _topologyData, MappedType defaultValue);

    /// destructor
    ~TopologyDataHandler();

    // initialization method called after the link between the Data and the Topology is established
    virtual void init();

    bool isTopologyDataRegistered() { return (m_topologyData != NULL);}

    /// Apply removing current elementType elements
    virtual void applyDestroyFunction(unsigned int, MappedType&) {}

    /// WARNING NEED TO UNIFY THIS
    /// Apply adding current elementType elements
    virtual void applyCreateFunction(unsigned int, MappedType& t,
        const sofa::helper::vector< unsigned int > &,
        const sofa::helper::vector< double > &);

    virtual void applyCreateFunction(unsigned int i, MappedType&t, const TopologyElementType&,
        const sofa::helper::vector< unsigned int > &ancestors,
        const sofa::helper::vector< double > &coefs)
    {
        applyCreateFunction(i, t, ancestors, coefs);
    }

    virtual void applyCreateFunction(unsigned int i, MappedType&t, const TopologyElementType& e,
        const sofa::helper::vector< unsigned int > &ancestors,
        const sofa::helper::vector< double > &coefs,
        const AncestorElem* /*ancestorElem*/)
    {
        applyCreateFunction(i, t, e, ancestors, coefs);
    }

    virtual void applyCreateFunction(unsigned int index, ContainerType& container,
        const TopologyElementType& e,
        const sofa::helper::vector< unsigned int > & ancestors,
        const sofa::helper::vector< double > & coefs,
        const AncestorElem* ancestorElem);

    virtual void applyDestroyFunction(unsigned int index, ContainerType& container);

	// update the default value used during creation
    void setDefaultValue(const MappedType &v);

protected:
    /// Swaps values at indices i1 and i2.
    virtual void swap( unsigned int i1, unsigned int i2 );

    template<class T>
    inline void swapHelper(unsigned int i1, unsigned int i2, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Array>);
    template<class T>
    inline void swapHelper(unsigned int i1, unsigned int i2, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Map>);
    template<class T>
    inline void swapHelper(unsigned int i1, unsigned int i2, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Set>);

    /// Add some values. Values are added at the end of the vector.
    /// This (new) version gives more information for element indices and ancestry
    virtual void add( const sofa::helper::vector<unsigned int> & index,
            const sofa::helper::vector< TopologyElementType >& elems,
            const sofa::helper::vector< sofa::helper::vector< unsigned int > > &ancestors,
            const sofa::helper::vector< sofa::helper::vector< double > >& coefs,
            const sofa::helper::vector< AncestorElem >& ancestorElems);

    /// Remove the values corresponding to the elements removed.
    virtual void remove( const sofa::helper::vector<unsigned int> &index );

    template<class T>
    inline void removeHelper(const sofa::helper::vector<unsigned int> &index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Array>);
    template<class T>
    inline void removeHelper(const sofa::helper::vector<unsigned int> &index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Map>);
    template<class T>
    inline void removeHelper(const sofa::helper::vector<unsigned int> &index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Set>);

    /// Reorder the values.
    virtual void renumber( const sofa::helper::vector<unsigned int> &index, const sofa::helper::vector<unsigned int>& inv_index);

    template<class T>
    inline void renumberHelper(const sofa::helper::vector<unsigned int> &index, const sofa::helper::vector<unsigned int>& inv_index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Array>);
    template<class T>
    inline void renumberHelper(const sofa::helper::vector<unsigned int> &index, const sofa::helper::vector<unsigned int>& inv_index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Map>);
    template<class T>
    inline void renumberHelper(const sofa::helper::vector<unsigned int> &index, const sofa::helper::vector<unsigned int>& inv_index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Set>);

    /// Move a list of points
    virtual void move( const sofa::helper::vector<unsigned int> &indexList,
            const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
            const sofa::helper::vector< sofa::helper::vector< double > >& coefs);

    /// Add Element after a displacement of vertices, ie. add element based on previous position topology revision.
    virtual void addOnMovedPosition(const sofa::helper::vector<unsigned int> &indexList,
            const sofa::helper::vector< TopologyElementType > & elems);

    /// Remove Element after a displacement of vertices, ie. add element based on previous position topology revision.
    virtual void removeOnMovedPosition(const sofa::helper::vector<unsigned int> &indices);


};


} // namespace topology

} // namespace component

} // namespace sofa


#endif // SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATAHANDLER_H
