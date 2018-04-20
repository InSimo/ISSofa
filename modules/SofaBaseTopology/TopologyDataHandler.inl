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
#ifndef SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATAHANDLER_INL
#define SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATAHANDLER_INL

#include <SofaBaseTopology/TopologyDataHandler.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa
{

namespace component
{

namespace topology
{

// TODO: this could be part of the Topology API instead
inline unsigned int TopologyDataHandler_GetTopologySize(sofa::core::topology::BaseMeshTopology* t, core::topology::TopologyObjectType e)
{
    switch (e)
    {
    case(core::topology::POINT): return t->getNbPoints();
    case(core::topology::EDGE): return t->getNbEdges();
    case(core::topology::TRIANGLE): return t->getNbTriangles();
    case(core::topology::QUAD): return t->getNbQuads();
    case(core::topology::TETRAHEDRON): return t->getNbTetrahedra();
    case(core::topology::HEXAHEDRON): return t->getNbHexahedra();
    }
    return 0;
}

/// constructor without default value
template <typename TopologyElementType, typename ContainerType>
TopologyDataHandler <TopologyElementType, ContainerType>::TopologyDataHandler(sofa::core::topology::BaseTopologyData <ContainerType>* _topologyData)
    :Inherit(),
    m_topologyData(_topologyData),
    m_defaultValue(NULL)
{
}

/// constructor with default value
template <typename TopologyElementType, typename ContainerType>
TopologyDataHandler <TopologyElementType, ContainerType>::TopologyDataHandler(sofa::core::topology::BaseTopologyData <ContainerType>* _topologyData, MappedType defaultValue)
    :Inherit(),
    m_topologyData(_topologyData),
    m_defaultValue(new MappedType(defaultValue))
{
}

/// Destructors
template <typename TopologyElementType, typename ContainerType>
TopologyDataHandler <TopologyElementType, ContainerType>::~TopologyDataHandler()
{
    if (m_defaultValue)
    {
        delete m_defaultValue;
    }
}

template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::init()
{
    this->setLastTopoSize(TopologyDataHandler_GetTopologySize(m_topologyData->getTopology(), TopologyElementInfoT::type()));
}

template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::applyCreateFunction(unsigned int, 
    MappedType& t,
    const sofa::helper::vector< unsigned int > &,
    const sofa::helper::vector< double > &)
{
    if (m_defaultValue)
    {
        t = *m_defaultValue;
    }
    else
    {
        defaulttype::DataTypeInfo<MappedType>::resetValue(t);
    }
}


template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::applyCreateFunction(unsigned int index, ContainerType& container,
    const TopologyElementType& e,
    const sofa::helper::vector< unsigned int > & ancestors,
    const sofa::helper::vector< double > & coefs,
    const AncestorElem* ancestorElem)
{
    MappedType* p = sofa::defaulttype::DataTypeInfo<ContainerType>::insertItem(container, (KeyType)index);
    if (p != nullptr)
    {
        applyCreateFunction(
            index,
            *p,
            e,
            ancestors,
            coefs,
            ancestorElem);
    }
}


template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::applyDestroyFunction(unsigned int index, ContainerType& container)
{
    MappedType* p = sofa::defaulttype::DataTypeInfo<ContainerType>::findEditItem(container, index);
    if (p != nullptr)
    {
        applyDestroyFunction(index, *p);
    }
}

template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::setDefaultValue(const MappedType &v)
{
    if (m_defaultValue)
    {
        *m_defaultValue = v;
    }
    else
    {
        m_defaultValue = new MappedType(v);
    }
}

///////////////////// Private functions on TopologyDataHandler changes /////////////////////////////
template <typename TopologyElementType, typename ContainerType> template<class T>
void TopologyDataHandler <TopologyElementType, ContainerType>::swapHelper(unsigned int i1, unsigned int i2, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Array>)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());
    MappedType* elem1Ptr = DataTypeInfo::findEditItem(containerData, i1);
    MappedType* elem2Ptr = DataTypeInfo::findEditItem(containerData, i2);
    if (elem1Ptr && elem2Ptr)
    {
        std::swap(*elem1Ptr, *elem2Ptr);
    }
    else
    {
        m_topologyData->getOwner()->serr << "cannot swap element : " << i1 << " and " << i2 << " in the TopologyData " << m_topologyData->getName() << m_topologyData->getOwner()->sendl;
    }
    m_topologyData->endEdit();
}
template <typename TopologyElementType, typename ContainerType> template<class T>
void TopologyDataHandler <TopologyElementType, ContainerType>::swapHelper(unsigned int i1, unsigned int i2, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Map>)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());
    MappedType* elem1Ptr = DataTypeInfo::findEditItem(containerData, i1);
    MappedType* elem2Ptr = DataTypeInfo::findEditItem(containerData, i2);

    if (elem1Ptr && elem2Ptr)
    {
        std::swap(*elem1Ptr, *elem2Ptr);
    }
    else if ((elem1Ptr && !elem2Ptr) || (!elem1Ptr && elem2Ptr)) 
    {
        if (elem1Ptr)
        {
            MappedType* newElem2Ptr = DataTypeInfo::insertItem(containerData, i2);
            std::swap(*newElem2Ptr, *elem1Ptr);
            DataTypeInfo::eraseItem(containerData, i1);
        }
        if (elem2Ptr)
        {
            MappedType* newElem1Ptr = DataTypeInfo::insertItem(containerData, i1);
            std::swap(*newElem1Ptr, *elem2Ptr);
            DataTypeInfo::eraseItem(containerData, i2);
        }
    }
    m_topologyData->endEdit();
}
template <typename TopologyElementType, typename ContainerType> template<class T>
void TopologyDataHandler <TopologyElementType, ContainerType>::swapHelper(unsigned int i1, unsigned int i2, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Set>)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());
    const MappedType* elem1Ptr = DataTypeInfo::findItem(containerData, i1);
    const MappedType* elem2Ptr = DataTypeInfo::findItem(containerData, i2);

    if (elem1Ptr && !elem2Ptr)
    {
        MappedType* newElem2Ptr = DataTypeInfo::insertItem(containerData, i2);
        DataTypeInfo::eraseItem(containerData, i1);
    }
    if (!elem1Ptr && elem2Ptr)
    {
        MappedType* newElem1Ptr = DataTypeInfo::insertItem(containerData, i1);
        DataTypeInfo::eraseItem(containerData, i2);
    }
    m_topologyData->endEdit();
}

template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::swap(unsigned int i1, unsigned int i2)
{
    swapHelper<ContainerType>(i1, i2, std::integral_constant<defaulttype::ContainerKindEnum, DataTypeInfo::ContainerKind>());
}

template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::add(const sofa::helper::vector<unsigned int> & index,
    const sofa::helper::vector< TopologyElementType >& elems,
    const sofa::helper::vector<sofa::helper::vector<unsigned int> > &ancestors,
    const sofa::helper::vector<sofa::helper::vector<double> > &coefs,
    const sofa::helper::vector< AncestorElem >& ancestorElems)
{
    unsigned int nbElements = index.size();
    if (nbElements == 0) return;
    // Using default values
    ContainerType& containerData = *(m_topologyData->beginEdit());
    unsigned int nbElemsTopo = this->getLastTopoSize(); // DataTypeInfo::containerSize(containerData);

    if (nbElemsTopo != index[0])
    {
        std::cerr << "TODO\n";
        nbElemsTopo = index[0];
    }

    DataTypeInfo::setContainerSize(containerData, nbElemsTopo + nbElements);

    const sofa::helper::vector< unsigned int > empty_vecint;
    const sofa::helper::vector< double > empty_vecdouble;

    for (unsigned int i = 0; i < nbElements; ++i)
    {
        MappedType* newElemPtr = DataTypeInfo::insertItem(containerData, nbElemsTopo + i);
        if (newElemPtr)
        {
            this->applyCreateFunction(nbElemsTopo + i, *newElemPtr, elems[i],
                (ancestors.empty() || coefs.empty()) ? empty_vecint : ancestors[i],
                (ancestors.empty() || coefs.empty()) ? empty_vecdouble : coefs[i],
                (ancestorElems.empty()) ? NULL : &ancestorElems[i]);
        }
    }
    //this->m_lastTopoElementId += nbElements;
    this->setLastTopoSize(nbElemsTopo + nbElements);
    m_topologyData->endEdit();
}

template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::move(const sofa::helper::vector<unsigned int> &indexList,
    const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
    const sofa::helper::vector< sofa::helper::vector< double > >& coefs)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());

    for (unsigned int i = 0; i <indexList.size(); i++)
    {
        this->applyDestroyFunction(indexList[i], containerData);
        this->applyCreateFunction(indexList[i], containerData, TopologyElementType(), ancestors[i], coefs[i], NULL); // Not clean     
    }

    m_topologyData->endEdit();
}

template <typename TopologyElementType, typename ContainerType> template<class T>
void TopologyDataHandler <TopologyElementType, ContainerType>::removeHelper(const sofa::helper::vector<unsigned int> &index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Array>)
{
    ContainerType& data = *(m_topologyData->beginEdit());

    unsigned int nbElemsTopo = this->getLastTopoSize(); // DataTypeInfo::containerSize(containerData);
    if (!data.empty())
    {
        for (unsigned int i = 0; i < index.size(); ++i)
        {
            this->applyDestroyFunction(index[i], data[index[i]]);
            this->swap(index[i], nbElemsTopo-1);
            --nbElemsTopo;
        }

        DataTypeInfo::setContainerSize(data, nbElemsTopo);
    }
    else // should not happen for arrays
    {
        nbElemsTopo -= index.size();
    }
    this->setLastTopoSize(nbElemsTopo);
    m_topologyData->endEdit();
}

template <typename TopologyElementType, typename ContainerType> template<class T>
void TopologyDataHandler <TopologyElementType, ContainerType>::removeHelper(const sofa::helper::vector<unsigned int> &index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Map>)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());
    unsigned int nbElemsTopo = this->getLastTopoSize(); // DataTypeInfo::containerSize(containerData);
    if (!containerData.empty())
    {
        for (unsigned int i = 0; i < index.size(); ++i)
        {
            this->applyDestroyFunction(index[i], containerData);
            MappedType* elemPtr = DataTypeInfo::findEditItem(containerData, index[i]);
            MappedType* lastPtr = (index[i] == nbElemsTopo - 1) ? nullptr : DataTypeInfo::findEditItem(containerData, nbElemsTopo - 1);
            if (elemPtr)
            {
                if (lastPtr)
                {
                    std::swap(*elemPtr, *lastPtr);
                    DataTypeInfo::eraseItem(containerData, nbElemsTopo-1);
                }
                else
                {
                    DataTypeInfo::eraseItem(containerData, index[i]);
                }
            }
            else
            {
                if (lastPtr)
                {
                    MappedType* newElemPtr = DataTypeInfo::insertItem(containerData, index[i]);
                    std::swap(*newElemPtr, *lastPtr);
                    DataTypeInfo::eraseItem(containerData, nbElemsTopo - 1);
                }
                else
                {
                    // nothing to do
                }
            }
            --nbElemsTopo;
        }
    }
    else
    {
        nbElemsTopo -= index.size();
    }
    this->setLastTopoSize(nbElemsTopo);
    m_topologyData->endEdit();
}

template <typename TopologyElementType, typename ContainerType> template<class T>
void TopologyDataHandler <TopologyElementType, ContainerType>::removeHelper(const sofa::helper::vector<unsigned int> &index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Set>)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());
    unsigned int nbElemsTopo = this->getLastTopoSize();
    if (!containerData.empty())
    {
        for (unsigned int i = 0; i < index.size(); ++i)
        {
            this->applyDestroyFunction(index[i], containerData);
            const MappedType* elemPtr = DataTypeInfo::findItem(containerData, index[i]);
            const MappedType* lastPtr = (index[i] == nbElemsTopo - 1) ? nullptr : DataTypeInfo::findItem(containerData, nbElemsTopo - 1);
            if (elemPtr)
            {
                if (lastPtr)
                {
                    DataTypeInfo::eraseItem(containerData, nbElemsTopo - 1);
                }
                else
                {
                    DataTypeInfo::eraseItem(containerData, index[i]);
                }
            }
            else
            {
                if (lastPtr)
                {
                    DataTypeInfo::insertItem(containerData, index[i]);
                    DataTypeInfo::eraseItem(containerData, nbElemsTopo - 1);
                }
                else
                {
                    // nothing to do
                }
            }
            --nbElemsTopo;
        }
    }
    else
    {
        nbElemsTopo -= index.size();
    }
    this->setLastTopoSize(nbElemsTopo);
    m_topologyData->endEdit();
}

template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::remove(const sofa::helper::vector<unsigned int> &index)
{
    removeHelper<ContainerType>(index, std::integral_constant<defaulttype::ContainerKindEnum, DataTypeInfo::ContainerKind>());
}


template <typename TopologyElementType, typename ContainerType> template<class T>
void TopologyDataHandler <TopologyElementType, ContainerType>::renumberHelper(const sofa::helper::vector<unsigned int> &index, const sofa::helper::vector<unsigned int>& inv_index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Array>)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());
    if (!containerData.empty())
    {
        ContainerType copy = m_topologyData->getValue(); // not very efficient memory-wise, but I can see no better solution...
        for (unsigned int i = 0; i < index.size(); ++i)
        {
            containerData[i] = std::move(copy[index[i]]);
        }
    }
    m_topologyData->endEdit();
}

template <typename TopologyElementType, typename ContainerType> template<class T>
void TopologyDataHandler <TopologyElementType, ContainerType>::renumberHelper(const sofa::helper::vector<unsigned int> &index, const sofa::helper::vector<unsigned int>& inv_index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Map>)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());
    if (!containerData.empty())
    {
        ContainerType copy = containerData; // not very efficient memory-wise, but I can see no better solution...

        containerData.clear();

        for (typename ContainerType::iterator it = DataTypeInfo::begin(copy); it != DataTypeInfo::end(copy); ++it)
        {
            MappedType* newElemVal = DataTypeInfo::insertItem(containerData, inv_index[it->first]);
            std::swap(*newElemVal, it->second);
        }
    }
    m_topologyData->endEdit();
}

template <typename TopologyElementType, typename ContainerType> template<class T>
void TopologyDataHandler <TopologyElementType, ContainerType>::renumberHelper(const sofa::helper::vector<unsigned int> &index, const sofa::helper::vector<unsigned int>& inv_index, std::integral_constant<defaulttype::ContainerKindEnum, defaulttype::ContainerKindEnum::Set>)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());
    if (!containerData.empty())
    {
        ContainerType copy = containerData; // not very efficient memory-wise, but I can see no better solution...

        containerData.clear();

        for (typename ContainerType::const_iterator it = DataTypeInfo::cbegin(copy); it != DataTypeInfo::cend(copy); ++it)
        {
            DataTypeInfo::insertItem(containerData, KeyType(inv_index[*it]));
        }
    }
    m_topologyData->endEdit();
}

template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::renumber(const sofa::helper::vector<unsigned int> &index, const sofa::helper::vector<unsigned int>& inv_index)
{
    renumberHelper<ContainerType>(index, inv_index, std::integral_constant<defaulttype::ContainerKindEnum, DataTypeInfo::ContainerKind>());
}


template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::addOnMovedPosition(const sofa::helper::vector<unsigned int> &indexList,
    const sofa::helper::vector<TopologyElementType> &elems)
{
    ContainerType& containerData = *(m_topologyData->beginEdit());

    // Recompute data
    sofa::helper::vector< unsigned int > ancestors;
    sofa::helper::vector< double >  coefs;
    coefs.push_back(1.0);
    ancestors.resize(1);

    for (unsigned int i = 0; i <indexList.size(); i++)
    {
        ancestors[0] = indexList[i];
        MappedType* elemPtr = DataTypeInfo::findEditItem(containerData, indexList[i]);
        if (elemPtr)
        {
            this->applyCreateFunction(indexList[i], *elemPtr, elems[i], ancestors, coefs);
        }
    }
    m_topologyData->endEdit();
}



template <typename TopologyElementType, typename ContainerType>
void TopologyDataHandler <TopologyElementType, ContainerType>::removeOnMovedPosition(const sofa::helper::vector<unsigned int> &indices)
{
    ContainerType& data = *(m_topologyData->beginEdit());

    for (unsigned int i = 0; i < indices.size(); i++)
    {
        this->applyDestroyFunction(indices[i], data);
    }

    m_topologyData->endEdit();

    // TODO check why this call.
    //this->remove( indices );
}

} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATAHANDLER_INL
