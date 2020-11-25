/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
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
#ifndef SOFA_COMPONENT_MAPPING_SHAPEMATCHINGMAPPING_INL
#define SOFA_COMPONENT_MAPPING_SHAPEMATCHINGMAPPING_INL

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <SofaRigid/ShapeMatchingMapping.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/Mapping.inl>

#include <sofa/helper/gl/template.h>
#include <sofa/helper/decompose.h>

#include <iostream>

#ifdef USING_OMP_PRAGMAS
    #include <omp.h>
#endif

namespace sofa
{

namespace component
{

namespace mapping
{

template <class TIn, class TOut>
ShapeMatchingMapping<TIn, TOut>::ShapeMatchingMapping()
    : d_clusters(initData(&d_clusters, "clusters", "Input clusters, groups of points id used to compute associated rigid frames"))
    , d_cluster(initData(&d_cluster, "cluster", "Workaround to use only one cluster. Override clusters data if not empty"))
    , d_updateClusters(initData(&d_updateClusters, false, "updateClusters", "Update clusters when associated data are updated if set to true"))
    , d_drawTargetPositions(initData(&d_drawTargetPositions, false, "drawTargetPositions", "Draw target positions (input model cluster points after shape matching application)"))
    , d_drawClusters(initData(&d_drawClusters, false, "drawClusters", "Draw clusters"))
    , d_drawScale(initData(&d_drawScale, 1.0, "drawScale", "Draw scale"))
{
}


template <class TIn, class TOut>
ShapeMatchingMapping<TIn, TOut>::~ShapeMatchingMapping()
{
}


template <class TIn, class TOut>
void ShapeMatchingMapping<TIn, TOut>::init()
{
    reinit();
}


template <class TIn, class TOut>
void ShapeMatchingMapping<TIn, TOut>::reinit()
{
    const VPointID clusterData = d_cluster.getValue();
    if (!clusterData.empty() && (d_updateClusters.getValue() || d_clusters.getValue().empty()))
    {
        d_clusters.setValue({clusterData});
    }

    m_clusters = d_clusters.getValue();
    m_nbClusters = m_clusters.size();

    for (VPointID cluster : m_clusters)
    {
        std::sort(cluster.begin(), cluster.end());
        cluster.erase(std::unique(cluster.begin(), cluster.end()), cluster.end());
    }

    m_Xcm0.resize(m_nbClusters);
    m_Xcm.resize(m_nbClusters);
    m_W.resize(m_nbClusters);
    m_T.resize(m_nbClusters);

    const auto inX0 = this->fromModel->read(core::ConstVecCoordId::restPosition())->getValue();
    m_nbPositions = inX0.size();

    for (size_t i = 0; i < m_nbClusters; ++i)
    {
        m_W[i] = 0;
        m_Xcm0[i] = InCoord();

        for (VPointID::const_iterator it = m_clusters[i].begin(); it != m_clusters[i].end(); ++it)
        {
            if (*it > m_nbPositions)
                serr << "Cluster index " << *it << " not in input model rest positions (size " << m_nbPositions << ")" << sendl;

            InCoord p0 = inX0[*it];
            m_Xcm0[i] += p0;
            m_W[i]++;
        }

        if (m_W[i] != 0)
            m_Xcm0[i] /= m_W[i];
    }
}


template <class TIn, class TOut>
void ShapeMatchingMapping<TIn, TOut>::apply( const sofa::core::MechanicalParams* mparams /* PARAMS FIRST */, OutDataVecCoord& outData, const InDataVecCoord& inData)
{
    const auto inX0 = this->fromModel->read(core::ConstVecCoordId::restPosition())->getValue();
    const bool needClusterUpdate = d_updateClusters.getValue() && (d_clusters.isDirty() || d_cluster.isDirty());
    const bool needPositionsUpdate = m_nbPositions != inX0.size();

    if(!m_nbClusters || needClusterUpdate || needPositionsUpdate)
        reinit();

    if(!m_nbClusters || !m_nbPositions)
        return;

    const auto inX = sofa::helper::read(inData, mparams);
    auto outFrames = sofa::helper::write(outData, mparams);
    outFrames.resize(m_nbClusters);

    for (unsigned int i = 0; i < m_nbClusters; ++i)
    {
        m_Xcm[i] = InCoord();
        m_T[i].fill(0);

        for (VPointID::const_iterator it = m_clusters[i].begin(); it != m_clusters[i].end(); ++it)
        {
            if (*it > m_nbPositions)
                serr << "Cluster index " << *it << " not in input model positions (size " << m_nbPositions << ")" << sendl;

            InCoord p0 = inX0[*it];
            InCoord p  = inX[*it];
            m_Xcm[i] += p;
            m_T[i]   += dyad(p, p0);
        }

        m_T[i] -= dyad(m_Xcm[i], m_Xcm0[i]); // sum wi.(X-m_Xcm)(X0-m_Xcm0)^m_T = sum wi.X.X0^m_T - sum(wi.X).m_Xcm0^m_T
        m_Xcm[i] /= m_W[i];

        Mat3x3 R;
        helper::Decompose<InReal>::polarDecomposition(m_T[i], R);
        m_T[i] = R;

        outFrames[i].getCenter() = m_Xcm[i] - m_Xcm0[i];
        outFrames[i].getOrientation().fromMatrix(m_T[i]);
    }

    // debug draw
    m_clustersPositions.resize(m_nbClusters);
    m_targetPositions.clear();
    m_targetPositions.resize(m_nbPositions);
    m_nbClustersAffectingPoint.clear();
    m_nbClustersAffectingPoint.resize(m_nbPositions);

    for (size_t i = 0; i < m_nbClusters; ++i)
    {
        InVecCoord& clusterPositions = m_clustersPositions[i];
        clusterPositions.clear();
        for (VPointID::const_iterator it = m_clusters[i].begin(); it != m_clusters[i].end(); ++it)
        {
            m_targetPositions[*it] += m_Xcm[i] + m_T[i] * (inX0[*it] - m_Xcm0[i]);
            m_nbClustersAffectingPoint[*it]++;
            clusterPositions.push_back(inX[*it]);
        }
    }

    for (unsigned int i = 0; i < m_targetPositions.size(); ++i)
    {
        const unsigned int nbClustersAffectingPoint = m_nbClustersAffectingPoint[i];
        if (nbClustersAffectingPoint)
            m_targetPositions[i] /= nbClustersAffectingPoint;
        else
            m_targetPositions[i] = inX[i];
    }
}

template <class TIn, class TOut>
void ShapeMatchingMapping<TIn, TOut>::applyJ( const sofa::core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, OutDataVecDeriv& /*outData*/, const InDataVecDeriv& /*inData*/)
{
}


template <class TIn, class TOut>
void ShapeMatchingMapping<TIn, TOut>::applyJT( const sofa::core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, InDataVecDeriv& /*outData*/, const OutDataVecDeriv& /*inData*/)
{
}


template <class TIn, class TOut>
void ShapeMatchingMapping<TIn, TOut>::applyJT( const sofa::core::ConstraintParams* /*cparams*/ /* PARAMS FIRST */, InDataMatrixDeriv& /*outData*/, const OutDataMatrixDeriv& /*inData*/)
{
}


template <class TIn, class TOut>
void ShapeMatchingMapping<TIn, TOut>::draw(const core::visual::VisualParams* vparams)
{
    const float scale = static_cast<float>(d_drawScale.getValue());

    if (d_drawTargetPositions.getValue())
        vparams->drawTool()->drawPoints(m_targetPositions, scale, {0, 1, 1, 1});

    if (d_drawClusters.getValue())
    {
        float g = 0.f;
        float gStep = 1.f / m_clustersPositions.size();
        for (const InVecCoord& cluster : m_clustersPositions)
        {
            vparams->drawTool()->drawPoints(cluster, scale, {1, g, 0.5, 1});
            g += gStep;
        }
    }
}


} // namespace engine

} // namespace component

} // namespace sofa

#endif
