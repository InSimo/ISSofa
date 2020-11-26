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
#ifndef SOFA_COMPONENT_MAPPING_SUBSETMULTIMAPPING_INL
#define SOFA_COMPONENT_MAPPING_SUBSETMULTIMAPPING_INL

#include <sofa/core/visual/VisualParams.h>
#include <SofaMiscMapping/SubsetMultiMapping.h>
#ifdef SOFA_HAVE_EIGEN2
#include <SofaEigen2Solver/EigenSparseMatrix.h>
#endif
#include <sofa/core/MultiMapping.inl>
#include <iostream>

namespace sofa
{

namespace component
{

namespace mapping
{

template <class TIn, class TOut>
void SubsetMultiMapping<TIn, TOut>::fillIndexPairs()
{
    const vector<unsigned>& identityIndices = d_identityIndices.getValue();

    vector<unsigned>& w_indexPairs = *indexPairs.beginEdit();
    w_indexPairs.clear();
    w_indexPairs.resize(2 * identityIndices.size());

    vector<unsigned int> fromModelsSizes(this->fromModels.size());

    for (unsigned int i = 0; i < identityIndices.size(); ++i)
    {
        w_indexPairs[2 * i] = identityIndices[i];
        w_indexPairs[2 * i + 1] = fromModelsSizes[identityIndices[i]];
        fromModelsSizes[identityIndices[i]]++;
    }
}


template <class TIn, class TOut>
void SubsetMultiMapping<TIn, TOut>::init()
{
    assert( indexPairs.getValue().size()%2==0 );
    
    if (indexPairs.getValue().empty() && !d_identityIndices.getValue().empty())
    {
        fillIndexPairs();
    }

    const unsigned indexPairSize = indexPairs.getValue().size()/2;

    this->toModels[0]->resize( indexPairSize );

    Inherit::init();
}

template <class TIn, class TOut>
SubsetMultiMapping<TIn, TOut>::~SubsetMultiMapping()
{
}


template <class TIn, class TOut>
void SubsetMultiMapping<TIn, TOut>::handleTopologyChange(sofa::core::topology::Topology* t)
{
    sofa::core::topology::BaseMeshTopology* topo = sofa::core::topology::BaseMeshTopology::DynamicCast(t);

    if (topo == this->getTo()[0]->getContext()->getActiveMeshTopology())
    {

        std::list<const sofa::core::topology::TopologyChange *>::const_iterator it = topo->beginChange();
        std::list<const sofa::core::topology::TopologyChange *>::const_iterator itEnd   = topo->endChange();
        typedef  sofa::Data<sofa::helper::vector<unsigned>> IndexPairType;
        sofa::helper::WriteAccessor<sofa::Data<sofa::helper::vector<unsigned>>>  w_indexPairs = indexPairs;
        while ( it != itEnd )
        {
            switch ( ( *it )->getChangeType() )
            {
                case core::topology::POINTSADDED:
                {
                    const sofa::core::topology::PointsAdded* topoChange = static_cast<const sofa::core::topology::PointsAdded*>(*it);
                    const sofa::helper::vector<unsigned int>& points = topoChange->getIndexArray();
                    w_indexPairs.resize(w_indexPairs.size() + 2 * points.size());
                    break;
                }
                case core::topology::POINTSREMOVED:
                {
                    const sofa::helper::vector<unsigned int>& index = (static_cast<const sofa::core::topology::PointsRemoved*>(*it))->getArray();
                    unsigned int nbPoints = w_indexPairs.size() / 2;
                    for (unsigned int i = 0; i < index.size(); ++i)
                    {
                        w_indexPairs[2*index[i]] = w_indexPairs[2*(nbPoints - 1)];
                        w_indexPairs[2*index[i] + 1] = w_indexPairs[2*(nbPoints - 1) + 1];
                        --nbPoints;
                    }
                    w_indexPairs.resize(w_indexPairs.size() - 2 * index.size());
                    break;
                }
                default:
                {
                    break;
                }
            }

            ++it;
        }

    }
    else
    {

        for (unsigned int i = 0; i < this->fromModels.size(); i++)
        {
             if (topo == this->getFrom()[i]->getContext()->getActiveMeshTopology())
             {
                std::list<const sofa::core::topology::TopologyChange *>::const_iterator it = topo->beginChange();
                std::list<const sofa::core::topology::TopologyChange *>::const_iterator itEnd   = topo->endChange();

                sofa::helper::WriteAccessor<sofa::Data<sofa::helper::vector<unsigned>>>  w_indexPairs = indexPairs;
                while ( it != itEnd )
                {
                    switch ( ( *it )->getChangeType() )
                    {
                        case core::topology::POINTSREMOVED:
                        {
                            const sofa::helper::vector<unsigned int>& index = (static_cast<const sofa::core::topology::PointsRemoved*>(*it))->getArray();
                            const unsigned int nbPoints = this->getFrom()[i]->getSize();
                            //make a reverse copy of tab
                            sofa::helper::vector<unsigned int> lastIndexVec;
                            lastIndexVec.reserve(index.size());
                            for (unsigned int i_init = 0; i_init < index.size(); ++i_init)
                            {
                                lastIndexVec.push_back(nbPoints - 1 - i_init);
                            }

                            const sofa::helper::vector<unsigned int> copy(w_indexPairs.ref());

                            for (unsigned int id = 0; id < index.size(); id++)
                            {
                                auto swappedIndexInlastIndexVec = std::find(lastIndexVec.begin() + i, lastIndexVec.end(), index[id]);
                                bool willBeRemoved = (swappedIndexInlastIndexVec != lastIndexVec.end());
                                if (willBeRemoved)
                                    *swappedIndexInlastIndexVec = lastIndexVec[i];

                                for (unsigned int outid = 0; outid < (w_indexPairs.size()/2); outid++)
                                {
                                    if (copy[2*outid] == i && copy[2*outid + 1] == lastIndexVec[id] )
                                    {
                                        if (index[i] == nbPoints - 1)
                                        {
                                            w_indexPairs[2*outid + 1] = (unsigned int)(-1);
                                        }
                                        else
                                        {
                                            w_indexPairs[2*outid + 1] = index[id];
                                        }

                                    }
                                    else if(copy[2*outid] == i && copy[2*outid + 1] == index[id])
                                    {
                                        w_indexPairs[2*outid + 1] = (unsigned int)(-1);
                                    }

                                }
                            }

                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }

                    ++it;
                }
            }
        }
    }

}


template <class TIn, class TOut>
void SubsetMultiMapping<TIn, TOut>::addPoint( const core::BaseState* from, int index)
{

    // find the index of the parent state
    unsigned i;
    for(i=0; i<this->fromModels.size(); i++)
        if(this->fromModels.get(i)==from )
            break;
    if(i==this->fromModels.size())
    {
        serr<<"SubsetMultiMapping<TIn, TOut>::addPoint, parent "<<from->getName()<<" not found !"<< sendl;
        assert(0);
    }


    vector<unsigned>& indexPairsVector = *indexPairs.beginEdit();
    indexPairsVector.push_back(i);
    indexPairsVector.push_back(index);
    indexPairs.endEdit();

}

template <class TIn, class TOut>
void SubsetMultiMapping<TIn, TOut>::addPoint( int from, int index)
{
    assert(from < (int)this->fromModels.size());
    vector<unsigned>& indexPairsVector = *indexPairs.beginEdit();
    indexPairsVector.push_back(from);
    indexPairsVector.push_back(index);
    indexPairs.endEdit();
}


template <class TIn, class TOut>
void SubsetMultiMapping<TIn, TOut>::apply(const core::MechanicalParams* mparams /* PARAMS FIRST */, const helper::vector<OutDataVecCoord*>& dataVecOutPos, const helper::vector<const InDataVecCoord*>& dataVecInPos)
{
    //apply(const vecOutVecCoord& outPos, const vecConstInVecCoord& inPos)
    //OutVecCoord& out = *outPos[0];

    OutVecCoord& out = *(dataVecOutPos[0]->beginEdit(mparams));
    helper::ReadAccessor< Data< vector<unsigned> > > indexP = indexPairs;

    out.resize(indexP.size()/2);
    for(unsigned i=0; i<out.size(); i++)
    {
//        cerr<<"SubsetMultiMapping<TIn, TOut>::apply, i = "<< i <<", indexPair = " << indexPairs[i*2] << ", " << indexPairs[i*2+1] <<", inPos size = "<< inPos.size() <<", inPos[i] = " << (*inPos[indexPairs[i*2]]) << endl;
//        cerr<<"SubsetMultiMapping<TIn, TOut>::apply, out = "<< out << endl;
        const InDataVecCoord* inPosPtr = dataVecInPos[indexP[i*2]];
        const InVecCoord& inPos = (*inPosPtr).getValue();

        if (indexP[i*2+1] < inPos.size())
        {
            out[i] = inPos[indexP[i*2+1]];
        }
        else
        {
            serr << "Invalid indexPair for " << i << ": " << indexP[i*2] << " " << indexP[i*2+1] << " while in size is " << inPos.size() << sendl;
        }
    }

    dataVecOutPos[0]->endEdit(mparams);

}

template <class TIn, class TOut>
void SubsetMultiMapping<TIn, TOut>::applyJ(const core::MechanicalParams* mparams /* PARAMS FIRST */, const helper::vector<OutDataVecDeriv*>& dataVecOutVel, const helper::vector<const InDataVecDeriv*>& dataVecInVel)
{
    OutVecDeriv& out = *(dataVecOutVel[0]->beginEdit(mparams));
    helper::ReadAccessor< Data< vector<unsigned> > > indexP = indexPairs;
    out.resize(indexP.size()/2);
    for(unsigned i=0; i<out.size(); i++)
    {
        const InDataVecDeriv* inDerivPtr = dataVecInVel[indexP[i*2]];
        const InVecDeriv& inDeriv = (*inDerivPtr).getValue();
        if (indexP[i*2+1] < inDeriv.size())
        {
            out[i] = inDeriv[indexP[i*2+1]];
        }
        else
        {
            serr << "Invalid indexPair for " << i << ": " << indexP[i*2] << " " << indexP[i*2+1] << " while in size is " << inDeriv.size() << "(mstate size = " << this->fromModels[indexP[i*2]]->getSize() << ")" << sendl;
        }
    }

    dataVecOutVel[0]->endEdit(mparams);
}

template <class TIn, class TOut>
void SubsetMultiMapping<TIn, TOut>::applyJT( const core::ConstraintParams* /*cparams*/ /* PARAMS FIRST */, const helper::vector< InDataMatrixDeriv* >& dOut, const helper::vector< const OutDataMatrixDeriv* >& dIn)
{
    helper::ReadAccessor< Data< vector<unsigned> > > indexP = indexPairs;

    // hypothesis: one child only:
    const OutMatrixDeriv& in = dIn[0]->getValue();

    if (dOut.size() != this->fromModels.size())
    {
        serr<<"problem with number of output constraint matrices"<<sendl;
        return;
    }

    typename OutMatrixDeriv::RowConstIterator rowItEnd = in.end();
    // loop on the constraints defined on the child of the mapping
    for (typename OutMatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {

        typename OutMatrixDeriv::ColConstIterator colIt = rowIt.begin();
        typename OutMatrixDeriv::ColConstIterator colItEnd = rowIt.end();


        // A constraint can be shared by several nodes,
        // these nodes can be linked to 2 different parents. // TODO handle more parents
        // we need to add a line to each parent that is concerned by the constraint


        while (colIt != colItEnd)
        {
            unsigned int index_parent=  indexP[colIt.index()*2]; // 0 or 1 (for now...)
            // writeLine provide an iterator on the line... if this line does not exist, the line is created:
            typename InMatrixDeriv::RowIterator o = dOut[index_parent]->beginEdit()->writeLine(rowIt.index());

            // for each col of the constraint direction, it adds a col in the corresponding parent's constraint direction
            if(indexP[colIt.index()*2+1] < (unsigned int)this->fromModels[index_parent]->getSize())
                o.addCol(indexP[colIt.index()*2+1], colIt.val());

            dOut[index_parent]->endEdit();

            ++colIt;
        }



    }

    //    std::cout<<" dIn ="<<(*dIn[0])<<std::endl;
    //    std::cout<<" dOut ="<<(*dOut[0])<<"  "<<(*dOut[1])<<std::endl;
}


template <class TIn, class TOut>
void SubsetMultiMapping<TIn, TOut>::applyJT(const core::MechanicalParams* mparams /* PARAMS FIRST */, const helper::vector<InDataVecDeriv*>& dataVecOutForce, const helper::vector<const OutDataVecDeriv*>& dataVecInForce)
//void SubsetMultiMapping<TIn, TOut>::applyJT(const helper::vector<typename SubsetMultiMapping<TIn, TOut>::InVecDeriv*>& parentDeriv , const helper::vector<const OutVecDeriv*>& childDeriv )
{
    const OutDataVecDeriv* cderData = dataVecInForce[0];
    const OutVecDeriv& cder = cderData->getValue();
    //const InVecDeriv& cder = *childDeriv[0];
    helper::ReadAccessor< Data< vector<unsigned> > > indexP = indexPairs;

    for(unsigned i=0; i<cder.size(); i++)
    {
        //(*parentDeriv[indexP[i*2]])[indexP[i*2+1]] += cder[i];
        InDataVecDeriv* inDerivPtr = dataVecOutForce[indexP[i*2]];
        InVecDeriv& inDeriv = *(*inDerivPtr).beginEdit(mparams);

        if (indexP[i*2+1] < inDeriv.size())
        {
            inDeriv[indexP[i*2+1]] += cder[i];
        }
        else
        {
            serr << "Invalid indexPair for " << i << ": " << indexP[i*2] << " " << indexP[i*2+1] << " while in size is " << inDeriv.size() << sendl;
        }

        (*inDerivPtr).endEdit(mparams);
    }
}

} // namespace mapping

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_MAPPING_SUBSETMULTIMAPPING_INL
