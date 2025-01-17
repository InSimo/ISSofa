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
#ifndef SOFA_COMPONENT_MAPPING_SUBSETMAPPING_INL
#define SOFA_COMPONENT_MAPPING_SUBSETMAPPING_INL

#include "SubsetMapping.h"

#include <sofa/core/Mapping.inl>
#include <SofaBaseTopology/TopologySubsetData.inl>
#include <sofa/simulation/common/AnimateBeginEvent.h>

namespace sofa
{

namespace component
{

namespace mapping
{


template <class TIn, class TOut>
SubsetMapping<TIn, TOut>::SubsetMapping()
    : Inherit()
    , f_indices( initData(&f_indices, "indices", "list of input indices"))
    , f_first( initData(&f_first, -1, "first", "first index (use if indices are sequential)"))
    , f_last( initData(&f_last, -1, "last", "last index (use if indices are sequential)"))
    , f_radius( initData(&f_radius, (Real)1.0e-5, "radius", "search radius to find corresponding points in case no indices are given"))
    , f_handleTopologyChange( initData(&f_handleTopologyChange, true, "handleTopologyChange", "Enable support of topological changes for indices (disable if it is linked from SubsetTopologicalMapping::pointD2S)"))
    , f_ignoreNotFound( initData(&f_ignoreNotFound, false, "ignoreNotFound", "True to ignore points that are not found in the input model, they will be treated as fixed points"))
    , f_resizeToModel( initData(&f_resizeToModel, false, "resizeToModel", "True to resize the output MechanicalState to match the size of indices"))
    , updateJ(false)
    , m_indicesEditCounter(0)
{
    this->f_listening.setValue(true);
}

template <class TIn, class TOut>
SubsetMapping<TIn, TOut>::~SubsetMapping()
{
}


template <class TIn, class TOut>
void SubsetMapping<TIn, TOut>::clear(int reserve)
{
    IndexArray& indices = *f_indices.beginEdit();
    indices.clear();
    if (reserve) indices.reserve(reserve);
    f_indices.endEdit();
}

template <class TIn, class TOut>
int SubsetMapping<TIn, TOut>::addPoint(int index)
{
    IndexArray& indices = *f_indices.beginEdit();
    int i = indices.size();
    indices.push_back(index);
    f_indices.endEdit();
    return i;
}

// Handle topological changes
/*
template <class TIn, class TOut>
void SubsetMapping<TIn, TOut>::handleTopologyChange(core::topology::Topology* t)
{
    core::topology::BaseMeshTopology* topoFrom = this->fromModel->getContext()->getMeshTopology();
    if (t != topoFrom) return;

    std::list<const core::topology::TopologyChange *>::const_iterator itBegin=topoFrom->beginChange();
    std::list<const core::topology::TopologyChange *>::const_iterator itEnd=topoFrom->endChange();
    f_indices.beginEdit()->handleTopologyEvents(itBegin,itEnd,this->fromModel->getX()->size());
    f_indices.endEdit();
}
*/

template <class TIn, class TOut>
void SubsetMapping<TIn, TOut>::init()
{
    const bool ignoreNotFound = f_ignoreNotFound.getValue();
    int numnotfound = 0;
    unsigned int inSize = this->fromModel->read(core::ConstVecCoordId::position())->getValue().size();
    if (f_indices.getValue().empty() && f_first.getValue() != -1)
    {
        IndexArray& indices = *f_indices.beginEdit();
        unsigned int first = (unsigned int)f_first.getValue();
        unsigned int last = (unsigned int)f_last.getValue();
        if (first >= inSize)
            first = 0;
        if (last >= inSize)
            last = inSize-1;
        indices.resize(last-first+1);
        for (unsigned int i=0; i<indices.size(); ++i)
            indices[i] = first+i;
        f_indices.endEdit();
    }
    else if (f_indices.getValue().empty() && !f_resizeToModel.getValue())
    {

        // We have to construct the correspondance index
        const InVecCoord& in   =this->fromModel->read(core::ConstVecCoordId::position())->getValue();
        const OutVecCoord& out =this->toModel->read(core::ConstVecCoordId::position())->getValue();
        IndexArray& indices = *f_indices.beginEdit();

        indices.resize(out.size());

        // searching for the first corresponding point in the 'from' model (there might be several ones).
        for (unsigned int i = 0; i < out.size(); ++i)
        {
            bool found = false;
            Real rmax = f_radius.getValue();
            for (unsigned int j = 0;  j < in.size(); ++j )
            {
                Real r = (Real)((out[i] - in[j]).norm());
                if ( r < rmax )
                {
                    indices[i] = j;
                    found = true;
                    rmax = r;
                }
            }
            if(!found)
            {
                ++numnotfound;
                if(!ignoreNotFound)
                {
                    sout<<"ERROR(SubsetMapping): point "<<i<<"="<<out[i]<<" not found in input model within a radius of "<<rmax<<"."<<sendl;
                }
                indices[i] = (unsigned int)-1;
            }
        }
        f_indices.endEdit();
        if (numnotfound > 0)
        {
            sout << out.size() << " points, " << out.size()-numnotfound << " found, " << numnotfound << " fixed points" << sendl;
        }
    }
    else if (!ignoreNotFound)
    {
        IndexArray& indices = *f_indices.beginEdit();
        for (unsigned int i=0; i<indices.size(); ++i)
        {
            if ((unsigned)indices[i] >= inSize)
            {
                serr << "ERROR(SubsetMapping): incorrect index "<<indices[i]<<" (input size "<<inSize<<")"<<sendl;
                indices.erase(indices.begin()+i);
                --i;
            }
        }
        f_indices.endEdit();
    }
    this->Inherit::init();

    topology = this->getContext()->getMeshTopology();

    if (f_handleTopologyChange.getValue())
    {
        // Initialize functions and parameters for topological changes
        f_indices.createTopologicalEngine(topology);
        f_indices.registerTopologicalData();
    }

    m_indicesEditCounter = f_indices.getCounter();

    postInit();
}



template <class TIn, class TOut>
void SubsetMapping<TIn, TOut>::postInit()
{
}


template <class TIn, class TOut>
void SubsetMapping<TIn, TOut>::handleEvent(core::objectmodel::Event *e)
{
#ifdef SOFA_MAPPING_APPLY_HANDLETOPOLOGYCHANGE
    if (sofa::simulation::AnimateBeginEvent::DynamicCast(e))
    {
        f_indices.getValue();
        unsigned indicesCurrentEditCounter = f_indices.getCounter();

        if (indicesCurrentEditCounter != m_indicesEditCounter)
        {
            this->sofa::core::Mapping<TIn, TOut>::apply(sofa::core::MechanicalParams::defaultInstance(), sofa::core::VecCoordId::position(), sofa::core::ConstVecCoordId::position());
            if (this->f_applyRestPosition.getValue())
            {
                this->sofa::core::Mapping<TIn, TOut>::apply(sofa::core::MechanicalParams::defaultInstance(), sofa::core::VecCoordId::restPosition(), sofa::core::ConstVecCoordId::restPosition());
            }
            this->sofa::core::Mapping<TIn, TOut>::applyJ(sofa::core::MechanicalParams::defaultInstance(), sofa::core::VecDerivId::velocity(), sofa::core::ConstVecDerivId::velocity());
            m_indicesEditCounter = indicesCurrentEditCounter;
        }

    }
#else
    SOFA_UNUSED(e);
#endif
}

template <class TIn, class TOut>
void SubsetMapping<TIn, TOut>::apply ( const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, OutDataVecCoord& dOut, const InDataVecCoord& dIn )
{
    const IndexArray& indices = f_indices.getValue();

    if ((unsigned int)this->toModel->getSize() != indices.size())
    { 
        if (f_resizeToModel.getValue())
        {
            this->toModel->resize(indices.size()); 
        }
        else
        {
            serr << "Output topology size does not match number of indices, did you forget to add a topological mapping or to set resizeToModel to true ?" << sendl;
        }
    }
    
    const InVecCoord& in = dIn.getValue();
    const OutVecCoord& out0 = this->toModel->read(core::ConstVecCoordId::restPosition())->getValue();
    OutVecCoord& out = *dOut.beginEdit();
    const unsigned int fromSize = in.size();

    out.resize(indices.size());
    for(unsigned int i = 0; i < out.size(); ++i)
    {
        if(indices[i] < fromSize)
            out[i] = in[ indices[i] ];
        else
        {
            serr<<"Indice "<<indices[i]<<" doesn't exist in input state"<<sendl;
            out[i] = out0[i];
        }
    }

    dOut.endEdit();
}

template <class TIn, class TOut>
void SubsetMapping<TIn, TOut>::applyJ( const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, OutDataVecDeriv& dOut, const InDataVecDeriv& dIn )
{
    const IndexArray& indices = f_indices.getValue();

    const InVecDeriv& in = dIn.getValue();
    OutVecDeriv& out = *dOut.beginEdit();
    const unsigned int fromSize = in.size();

    out.resize(indices.size());
    for(unsigned int i = 0; i < out.size(); ++i)
    {
        if(indices[i] < fromSize)
            out[i] = in[ indices[i] ];
    }

    dOut.endEdit();
}

template <class TIn, class TOut>
void SubsetMapping<TIn, TOut>::applyJT ( const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, InDataVecDeriv& dOut, const OutDataVecDeriv& dIn )
{
    const IndexArray& indices = f_indices.getValue();

    const OutVecDeriv& in = dIn.getValue();
    InVecDeriv& out = *dOut.beginEdit();
    const unsigned int fromSize = out.size();

    if (indices.empty())
        return;

    if (!in.empty())
    {
        out.resize(this->fromModel->getSize());
    }

    for(unsigned int i = 0; i < in.size(); ++i)
    {
        if(indices[i] < fromSize)
            out[indices[i]] += in[ i ];
    }

    dOut.endEdit();
}

template <class TIn, class TOut>
void SubsetMapping<TIn, TOut>::applyJT ( const core::ConstraintParams * /*cparams*/ /* PARAMS FIRST */, InDataMatrixDeriv& dOut, const OutDataMatrixDeriv& dIn)
{
    const IndexArray& indices = f_indices.getValue();

    InMatrixDeriv& out = *dOut.beginEdit();
    const OutMatrixDeriv& in = dIn.getValue();
    const unsigned int fromSize = this->fromModel->getSize();

    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();

        // Creates a constraints if the input constraint is not empty.
        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            while (colIt != colItEnd)
            {
                if(indices[colIt.index()] < fromSize)
                    o.addCol(indices[colIt.index()], colIt.val());
                ++colIt;
            }
        }
    }
    dOut.endEdit();
    //int offset = out.size();
    //out.resize(offset+in.size());

    //const IndexArray& indices = f_indices.getValue();
    //for(unsigned int i = 0; i < in.size(); ++i)
    //{
    //  OutConstraintIterator itOut;
    //  std::pair< OutConstraintIterator, OutConstraintIterator > iter=in[i].data();
    //
    //  for (itOut=iter.first;itOut!=iter.second;itOut++)
    //    {
    //      unsigned int indexIn = itOut->first;
    //      OutDeriv data = (OutDeriv) itOut->second;
    //      out[i+offset].add( indices[indexIn] , data );
    //    }
    //}
}

template<class TIn, class TOut>
const sofa::defaulttype::BaseMatrix* SubsetMapping<TIn, TOut>::getJ()
{
#ifdef SOFA_HAVE_EIGEN2
    getJs();
    return &eigen;
#else
    return nullptr;
#endif
}



#ifdef SOFA_HAVE_EIGEN2

template <class TIn, class TOut>
const typename SubsetMapping<TIn, TOut>::js_type* SubsetMapping<TIn, TOut>::getJs()
{
    if( !eigen.compressedMatrix.nonZeros() || updateJ ) {
        updateJ = false;

        const IndexArray& indices = f_indices.getValue();
        const unsigned rowsBlock = indices.size();
        const unsigned colsBlock = this->fromModel->getSize();

        const unsigned rows = rowsBlock * NOut;
        const unsigned cols = colsBlock * NIn;

        eigen.resize( rows, cols );

        for (unsigned i = 0; i < indices.size(); ++i) {
            for( unsigned j = 0; j < NOut; ++j) {
                eigen.beginRow( i*NOut+j );
                eigen.insertBack( i*NOut+j, indices[i]*NIn+j ,(SReal)1. );
            }
        }
        eigen.compress();
    }

    js.resize( 1 );
    js[0] = &eigen;
    return &js;
}

#endif

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
