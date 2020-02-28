/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
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
#ifndef SOFA_COMPONENT_MAPPING_SHAPEMATCHINGMAPPING_H
#define SOFA_COMPONENT_MAPPING_SHAPEMATCHINGMAPPING_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/core/Mapping.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/SofaCommon.h>
// #include <SofaBaseTopology/TopologyData.h>

namespace sofa
{

namespace component
{

namespace mapping
{

/**
 * This class computes target frames using shape matching deformation [Muller05][Muller11]
 */
template <class TIn, class TOut>
class ShapeMatchingMapping : public core::Mapping<TIn, TOut>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(ShapeMatchingMapping,TIn,TOut), SOFA_TEMPLATE2(core::Mapping,TIn,TOut));

    typedef TIn In;
    typedef typename In::Real Real;
    typedef typename In::Coord InCoord;
    typedef typename In::Deriv InDeriv;
    typedef typename In::VecCoord InVecCoord;
    typedef typename In::VecDeriv InVecDeriv;
    typedef typename In::MatrixDeriv InMatrixDeriv;
    typedef typename In::Real InReal;

    typedef Data<InVecCoord> InDataVecCoord;
    typedef Data<InVecDeriv> InDataVecDeriv;
    typedef Data<InMatrixDeriv> InDataMatrixDeriv;

    typedef TOut Out;
    typedef typename Out::Coord OutCoord;
    typedef typename Out::Deriv OutDeriv;
    typedef typename Out::VecCoord OutVecCoord;
    typedef typename Out::VecDeriv OutVecDeriv;
    typedef typename Out::MatrixDeriv OutMatrixDeriv;
    typedef typename Out::Real OutReal;

    typedef Data<OutVecCoord> OutDataVecCoord;
    typedef Data<OutVecDeriv> OutDataVecDeriv;
    typedef Data<OutMatrixDeriv> OutDataMatrixDeriv;

    typedef defaulttype::Mat<3,3,Real> Mat3x3;

    typedef core::topology::Topology::PointID PointID;
    typedef helper::vector<PointID> VPointID;
    typedef helper::vector<VPointID> VVPointID;
    typedef helper::vector<InReal> VReal;

public:

    ShapeMatchingMapping();

    virtual ~ShapeMatchingMapping();

    void init();

    void reinit();

    virtual void apply( const core::MechanicalParams* mparams /* PARAMS FIRST */, OutDataVecCoord& out, const InDataVecCoord& in);
    //void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );

    virtual void applyJ( const core::MechanicalParams* mparams /* PARAMS FIRST */, OutDataVecDeriv& out, const InDataVecDeriv& in);
    //void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );

    virtual void applyJT( const core::MechanicalParams* mparams /* PARAMS FIRST */, InDataVecDeriv& out, const OutDataVecDeriv& in);
    //void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );

    virtual void applyJT( const core::ConstraintParams* /*cparams*/ /* PARAMS FIRST */, InDataMatrixDeriv& /*out*/, const OutDataMatrixDeriv& /*in*/);

    void draw(const core::visual::VisualParams* vparams);

    Data< VPointID >    d_cluster;
    Data< VVPointID >   d_clusters;
    Data< bool >        d_updateClusters;
    Data< bool >        d_drawTargetPositions;
    Data< bool >        d_drawClusters;
    Data< double >      d_drawScale;

private:

    VVPointID m_clusters; // clusters of points to compute shape matching
    unsigned int m_nbClusters = 0u; // number of clusters
    unsigned int m_nbPositions = 0u; // number of input model positions
    sofa::helper::vector<unsigned int> m_nbClustersAffectingPoint; // number of clusters affecting points of input model, used for debug draw
    sofa::helper::vector<InVecCoord> m_clustersPositions; // position of each cluster points, used for debug draw
    InVecCoord m_targetPositions; // position of input model points after shape matching, used for debug draw

    InVecCoord m_Xcm0;
    InVecCoord m_Xcm;
    VReal m_W;
    helper::vector<Mat3x3> m_T;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_MAPPING_SHAPEMATCHING_CPP)
extern template class SOFA_RIGID_API ShapeMatchingMapping<defaulttype::Vec3dTypes, defaulttype::Rigid3dTypes>;
#endif

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
