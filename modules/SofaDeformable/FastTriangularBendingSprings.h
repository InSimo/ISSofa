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
//
// C++ Interface: FastTriangularBendingSprings
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_H
#define SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/SofaCommon.h>
#include <map>

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>

#include <SofaBaseTopology/TopologyData.h>

#include <cassert>
#include <iostream>
#include <array>

#ifdef SOFA_HAVE_EIGEN2
#include <SofaEigen2Solver/EigenSparseMatrix.h>
#endif


namespace sofa
{

namespace component
{

namespace forcefield
{

/**
Bending elastic force added between vertices of triangles sharing a common edge.

Adapted from: P. Volino, N. Magnenat-Thalmann. Simple Linear Bending Stiffness in Particle Systems.
Eurographics Symposium on Computer Animation (SIGGRAPH), pp. 101-105, September 2006. http://www.miralab.ch/repository/papers/165.pdf

EL: Add quadratic bending model for Inextensible Surfaces Eurographics Symposium on Geometry Processing (2006). http://www.cs.columbia.edu/cg/pdfs/9-QuadBend.pdf

 @author François Faure, 2012
*/

template<class _DataTypes>
class TEdgeSpring;

template<class _DataTypes>
struct TVecEdgeSpring;

template<class _DataTypes>
class FastTriangularBendingSprings : public core::behavior::ForceField< _DataTypes>
{
public:
    typedef _DataTypes DataTypes;
    SOFA_CLASS(SOFA_TEMPLATE(FastTriangularBendingSprings, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef core::behavior::ForceField<DataTypes> Inherited;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real Real;
    typedef TEdgeSpring<DataTypes> EdgeSpring;
    typedef TVecEdgeSpring<DataTypes> VecEdgeSpring;
    typedef core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef sofa::core::topology::Edge     Edge;
    typedef sofa::core::topology::Triangle Triangle;
    typedef sofa::core::topology::Topology::PointID PointID;
    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;

    enum { N=_DataTypes::spatial_dimensions };
    typedef defaulttype::Mat<N,N,Real>          Mat;

    Data<double> f_bendingStiffness;  ///< Material parameter

	Data<double> d_minDistValidity; ///< Minimal distance to consider a spring valid

    Data<bool>   d_useRestCurvature; ///< Use the rest curvature as the zero energy bending.
    Data<bool>   d_useCorotational;  ///< Use edge rotation to make restCurvature invariant to rotation
    Data<bool>   d_forceContinuity;  ///< Handle discontinuity for 180° edge rotation by using an hysterisis

    Data<bool>   d_quadraticBendingModel; /// Use quadratic bending model method for Inextensible Surfaces

    Data<Real> d_drawMaxSpringEnergy;
    Data<float> d_drawSpringSize;
    Data<bool>  d_drawSpringBase;
    /// The list of edge springs, one for each edge between two triangles
    sofa::component::topology::EdgeData<helper::vector<VecEdgeSpring> > d_edgeSprings;

    /// Searches triangle topology and creates the bending springs
    virtual void init();

    virtual void reinit();

	virtual void createBendingSprings(unsigned int edgeIndex, const sofa::helper::vector<unsigned int> &trianleAroundEdge);

    virtual void addForce(const core::MechanicalParams* mparams, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v);
    virtual void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& d_df, const DataVecDeriv& d_dx);
    /// compute and add all the element stiffnesses to the global stiffness matrix
    virtual void addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix);
    template<class MatrixWriter>
    void addKToMatrixT(const core::MechanicalParams* mparams, MatrixWriter mwriter);
    virtual double getPotentialEnergy(const core::MechanicalParams* mparams, const DataVecCoord& d_x) const;

    void draw(const core::visual::VisualParams* vparams);

protected:

    class TriangularBSEdgeHandler : public sofa::component::topology::TopologyDataHandler<Edge,vector<VecEdgeSpring> >
    {
    public:
        typedef typename FastTriangularBendingSprings<DataTypes>::VecEdgeSpring VecEdgeSpring;
        TriangularBSEdgeHandler(FastTriangularBendingSprings<DataTypes>* _ff, sofa::component::topology::EdgeData<sofa::helper::vector<VecEdgeSpring> >* _data)
            : sofa::component::topology::TopologyDataHandler<Edge, sofa::helper::vector<VecEdgeSpring> >(_data), ff(_ff) {}

        void applyCreateFunction(unsigned int edgeIndex,
				VecEdgeSpring &ei,
                const Edge& ,  const sofa::helper::vector< unsigned int > &,
                const sofa::helper::vector< double >&);

        void applyTriangleCreation(const sofa::helper::vector<unsigned int> &triangleAdded,
                const sofa::helper::vector<Triangle> & ,
                const sofa::helper::vector<sofa::helper::vector<unsigned int> > & ,
                const sofa::helper::vector<sofa::helper::vector<double> > &);

        void applyTriangleDestruction(const sofa::helper::vector<unsigned int> &triangleRemoved);

        void applyPointDestruction(const sofa::helper::vector<unsigned int> &pointIndices);

        void applyPointRenumbering(const sofa::helper::vector<unsigned int> &pointToRenumber);

        /// Callback to add triangles elements.
        void ApplyTopologyChange(const core::topology::TrianglesAdded* /*event*/);
        /// Callback to remove triangles elements.
        void ApplyTopologyChange(const core::topology::TrianglesRemoved* /*event*/);

        /// Callback to remove points elements.
        void ApplyTopologyChange(const core::topology::PointsRemoved* /*event*/);
        /// Callback to renumbering on points elements.
        void ApplyTopologyChange(const core::topology::PointsRenumbering* /*event*/);

    protected:
        FastTriangularBendingSprings<DataTypes>* ff;
    };

    sofa::core::topology::BaseMeshTopology* _topology;


    FastTriangularBendingSprings();

    virtual ~FastTriangularBendingSprings();

    sofa::component::topology::EdgeData<helper::vector<VecEdgeSpring> > &getEdgeInfo() {return d_edgeSprings;}

    TriangularBSEdgeHandler* edgeHandler;

    double m_potentialEnergy;
};


enum class BendingShellQuadrant : unsigned int
{
    UNUSED  = 0,
    I       = 1,
    II      = 2,
    III     = 3,
    IV      = 4
};

SOFA_ENUM_DECL(BendingShellQuadrant, I, II, III, IV);
SOFA_ENUM_STREAM_METHODS(BendingShellQuadrant);

template<class _DataTypes>
class TEdgeSpring
{
public:
    enum { N=_DataTypes::spatial_dimensions };
    typedef typename _DataTypes::Real           Real;
    typedef typename _DataTypes::Deriv          Deriv;
    typedef typename _DataTypes::VecCoord       VecCoord;
    typedef typename _DataTypes::VecDeriv       VecDeriv;
    typedef defaulttype::Mat<N,N,Real>          Mat;
    typedef defaulttype::Mat<12,12,Real>        StiffnessMatrix;

    TEdgeSpring() { resetCache(); }

    enum {QA=0,QB,QC,QD};          ///< vertex names as in Bergou paper
    enum {VA=QC,VB=QD,VC=QB,VD=QA};                    ///< vertex names as in Volino's paper
    sofa::defaulttype::Vec<4,unsigned> vid;  ///< vertex indices, in circular order
    sofa::defaulttype::Vec<4,Real> alpha;    ///< weight of each vertex in the bending vector
    Real lambda = Real(0.0);                             ///< bending stiffness
    Deriv R0;                                ///< rest curvature
    bool is_activated = false;
    bool is_initialized = false;
    bool invertNormal = false;
    BendingShellQuadrant quadrant = BendingShellQuadrant::UNUSED;

    mutable std::array<std::array<int, 2>, 4> mwriterCacheD;
    mutable std::array<std::array<int, 4>, 6> mwriterCacheS;
    void resetCache()
    {
        for (std::array<int, 2>& a : mwriterCacheD)
        {
            a.fill(-1);
        }
        for (std::array<int, 4>& a : mwriterCacheS)
        {
            a.fill(-1);
        }
    }

    BendingShellQuadrant computeShellQuadrant(const Deriv& n1 /*normal of first triangle*/, const Deriv& n2 /*normal of second triangle*/, const Deriv& u /*edge direction*/) const
    {
        assert(n1.isNormalized());
        assert(n2.isNormalized());
        assert(u.isNormalized());

        Deriv v2 = u.cross(n2);

        const bool IorII = n1*v2 > 0;
        const bool IorIV = n1*n2 > 0;

        if ( IorII && IorIV)
            return BendingShellQuadrant::I;
        if (IorII && !IorIV)
            return BendingShellQuadrant::II;
        if (!IorII && !IorIV)
            return BendingShellQuadrant::III;
        if (!IorII && IorIV)
            return BendingShellQuadrant::IV;

        assert(false);
        return BendingShellQuadrant::UNUSED;
    }

    /// Store the vertex indices and perform all the precomputations
    void setEdgeSpring( const VecCoord& p, unsigned iA, unsigned iB, unsigned iC, unsigned iD, Real materialBendingStiffness)
    {
        is_activated = is_initialized = true;
        vid[VA]=iA;
        vid[VB]=iB;
        vid[VC]=iC;
        vid[VD]=iD;

        Deriv NA = cross( p[vid[VA]]-p[vid[VC]], p[vid[VA]]-p[vid[VD]] );
        Deriv NB = cross( p[vid[VB]]-p[vid[VD]], p[vid[VB]]-p[vid[VC]] );
        Deriv NC = cross( p[vid[VC]]-p[vid[VB]], p[vid[VC]]-p[vid[VA]] );
        Deriv ND = cross( p[vid[VD]]-p[vid[VA]], p[vid[VD]]-p[vid[VB]] );

        alpha[VA] =  NB.norm() / (NA.norm() + NB.norm());
        alpha[VB] =  NA.norm() / (NA.norm() + NB.norm());
        alpha[VC] = -ND.norm() / (NC.norm() + ND.norm());
        alpha[VD] = -NC.norm() / (NC.norm() + ND.norm());

        // stiffness
        Deriv edgeDir = p[vid[VC]]-p[vid[VD]];
        edgeDir.normalize();
        Deriv AC = p[vid[VC]]-p[vid[VA]];
        Deriv BC = p[vid[VC]]-p[vid[VB]];
        Real ha = (AC - edgeDir * (AC*edgeDir)).norm(); // distance from A to CD
        Real hb = (BC - edgeDir * (BC*edgeDir)).norm(); // distance from B to CD
        Real l = (p[vid[VC]]-p[vid[VD]]).norm();          // distance from C to D
        lambda = (Real)(2./3) * (ha+hb)/(ha*ha*hb*hb) * l * materialBendingStiffness;

        //            cerr<<"EdgeInformation::setEdgeSpring, vertices = " << vid << endl;
    }

    /// For each edge includes between two triangles -> calculate coefficients "coef" and "K0" used in Matrix Q(ei) function edge ei
    void setEdgeSpringQuadratic( const VecCoord& p, unsigned iA, unsigned iB, unsigned iC, unsigned iD, Real materialBendingStiffness)
    {
        is_activated = is_initialized = true;

        vid[QA]=iD;  ///< WARNING vertex names and orientation as in Bergou's paper, different compared Linear method from Volino's paper
        vid[QB]=iC;
        vid[QC]=iA;
        vid[QD]=iB;

        Deriv e0 = p[vid[QB]]-p[vid[QA]];
        Deriv e1 = p[vid[QC]]-p[vid[QA]];
        Deriv e2 = p[vid[QD]]-p[vid[QA]];
        Deriv e3 = p[vid[QC]]-p[vid[QB]];
        Deriv e4 = p[vid[QD]]-p[vid[QB]];

        double c01 = cotTheta( e0, e1);
        double c02 = cotTheta( e0, e2);
        double c03 = cotTheta(-e0, e3);
        double c04 = cotTheta(-e0, e4);

        alpha[QA] = (Real)(c03+c04);
        alpha[QB] = (Real)(c01+c02);
        alpha[QC] = (Real)(-c01-c03);
        alpha[QD] = (Real)(-c02-c04);

        double A0 = 0.5 * cross(e0,e1).norm();
        double A1 = 0.5 * cross(e0,e2).norm();

        lambda = (Real)(( 3. / (2.*(A0+A1))) * materialBendingStiffness);
    }

    void computeRestCurvature( const VecCoord& p, bool useCorotational )
    {
        if (useCorotational)
        {
            Mat K0;
            computeSpringRotation(K0, p);
            R0 = K0.transposed()*computeBendingVector( p );
        }
        else
        {
            R0 = computeBendingVector( p );
        }
    }

    ///Compute the Rotation Matrix to SpringBase
    void computeSpringRotation(Mat& result, const VecCoord& p)
    {
        //The spring base is defined as:
        // u : the edge direction
        // n : the mean normal of adjacent triangles
        // v : u X n
        // Rotation Matrix is R = [u,v,n]

        unsigned int pe1 = vid[VC]; //First point id of edge
        unsigned int pe2 = vid[VD]; //Second point id of edge
        unsigned int pt1 = vid[VA]; //point id of first triangle
        unsigned int pt2 = vid[VB]; //point id of second triangle

        Deriv u = p[pe2] - p[pe1];
        u.normalize();
        const Deriv ea1 = p[pt1] - p[pe1];
        const Deriv ea2 = p[pt2] - p[pe1];
        Deriv n1 = u.cross(ea1);
        n1.normalize();
        Deriv n2 = -u.cross(ea2);
        n2.normalize();
        Deriv n = ((n1+n2) * 0.5);
        n.normalize();

        if (quadrant != BendingShellQuadrant::UNUSED)
        {
            const BendingShellQuadrant newQuadrant = computeShellQuadrant(n1, n2, u);
            if (newQuadrant != quadrant)
            {
                if (quadrant == BendingShellQuadrant::II && newQuadrant == BendingShellQuadrant::III)
                {
                    invertNormal = !invertNormal;
                }
                else if (quadrant == BendingShellQuadrant::III && newQuadrant == BendingShellQuadrant::II)
                {
                    invertNormal = !invertNormal;
                }
            }

            if(invertNormal)
            {
                n = -n;
            }
            quadrant = newQuadrant;
        }

        Deriv v = u.cross(n);
        v.normalize();

        for( int i = 0; i < 3; i++)
        {
            result[i][0] = u[i];
            result[i][1] = v[i];
            result[i][2] = n[i];
        }
    }


    double cotTheta(const Deriv& v, const Deriv& w)
    {
        assert(v.norm() > 0);
        assert(w.norm () > 0);
        const double cosTheta = dot(v,w);
        const double sinTheta = cross(v,w).norm();

        return (cosTheta / sinTheta);
    }

    Deriv computeBendingVector( const VecCoord& p) const
    {
       return ( p[vid[QA]]*alpha[QA] +  p[vid[QB]]*alpha[QB] +  p[vid[QC]]*alpha[QC] +  p[vid[QD]]*alpha[QD] );
    }

    /// Accumulates force and return potential energy
    Real addForce( VecDeriv& f, const VecCoord& p, const VecDeriv& /*v*/, bool useCorotational)
    {
        if( !is_activated ) return 0;

        Deriv R = computeBendingVector(p);
        if (useCorotational)
        {
            Mat Kx;
            computeSpringRotation(Kx,p);
            R -= Kx*R0;
        }
        else
        {
            R -= R0;
        }
        f[vid[QA]] -= R * (lambda * alpha[QA]);
        f[vid[QB]] -= R * (lambda * alpha[QB]);
        f[vid[QC]] -= R * (lambda * alpha[QC]);
        f[vid[QD]] -= R * (lambda * alpha[QD]);

        return (R * R) * lambda * (Real)0.5;
    }

    // Optimized version of addDForce
    void addDForce( VecDeriv& df, const VecDeriv& dp, Real kfactor) const
    {
        if( !is_activated ) return;
        //Deriv R = computeBendingVector(dp);
        //R *= (lambda*kfactor);
        Deriv R = (dp[vid[0]] * (kfactor *  alpha[0]) + dp[vid[1]] * (kfactor * alpha[1]) + dp[vid[2]] * (kfactor * alpha[2]) + dp[vid[3]] * (kfactor * alpha[3]))*lambda;
        df[vid[0]] -= R*alpha[0];
        df[vid[1]] -= R*alpha[1];
        df[vid[2]] -= R*alpha[2];
        df[vid[3]] -= R*alpha[3];
    }

    /// Stiffness matrix assembly
    template<class MWriter>
    void addStiffness( MWriter& mwriter, SReal scale) const
    {
        if( !is_activated ) return;

        const SReal flambda = -lambda*scale;
        for( unsigned j=0; j<4; j++ )
        {
            const Real flambda_aj = (Real)(flambda*alpha[j]);
            std::array<int, 2>& cache = mwriterCacheD[j];
            mwriter.addDiagDValue(vid[j], cache[0], cache[1], flambda_aj*alpha[j]);

            for( unsigned k=j+1; k<4; k++ )
            {
                std::array<int, 4>& cache = mwriterCacheS[((k-1)*k)/2 + j];
                mwriter.addSymDValue(vid[j], vid[k], cache[0], cache[1], cache[2], cache[3], flambda_aj*alpha[k]);
            }
        }
    }

    /// Compliant stiffness matrix assembly
    void getStiffness( StiffnessMatrix &K ) const
    {
        for( unsigned j=0; j<4; j++ )
            for( unsigned k=0; k<4; k++ )
            {
                K[j*3][k*3] = K[j*3+1][k*3+1] = K[j*3+2][k*3+2] = -lambda * alpha[j] * alpha[k];
            }
    }

    /// replace a vertex index with another one
    void replaceIndex( unsigned oldIndex, unsigned newIndex )
    {
        for(unsigned i=0; i<4; i++)
            if( vid[i] == oldIndex )
                vid[i] = newIndex;
    }

    /// replace all the vertex indices with the given ones
    void replaceIndices( const vector<unsigned> &newIndices )
    {
        for(unsigned i=0; i<4; i++)
            vid[i] = newIndices[vid[i]];
    }


    SOFA_STRUCT_DECL(TEdgeSpring, vid, alpha, lambda, R0, is_activated, is_initialized, quadrant, invertNormal);
    SOFA_STRUCT_STREAM_METHODS(TEdgeSpring);
    SOFA_STRUCT_COMPARE_METHOD(TEdgeSpring);
};


template<class _DataTypes>
struct TVecEdgeSpring
{
    sofa::helper::vector<TEdgeSpring<_DataTypes>> springs;

    /// Output stream
    inline friend std::ostream& operator<< ( std::ostream& os, const TVecEdgeSpring& ei )
    {
        os << "VecEdgeSpring : { ";
        for (unsigned int i = 0; i < ei.springs.size(); i++)
        {
            os << "isActive : " << ei.springs[i].is_activated << " lambda : " << ei.springs[i].lambda << " vid : " << ei.springs[i].vid;
        }
        os << " }";
        return os;
    }

    /// Input stream
    inline friend std::istream& operator>> ( std::istream& in, TVecEdgeSpring& /*ei*/ )
    {
        return in;
    }

    SOFA_STRUCT_DECL(TVecEdgeSpring, springs);
//    SOFA_STRUCT_STREAM_METHODS(TVecEdgeSpring);
//    SOFA_STRUCT_COMPARE_METHOD(TVecEdgeSpring);
};

} // namespace forcefield

} // namespace component

} // namespace sofa


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_DEFORMABLE_API sofa::component::forcefield::FastTriangularBendingSprings<sofa::defaulttype::Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_DEFORMABLE_API sofa::component::forcefield::FastTriangularBendingSprings<sofa::defaulttype::Vec3fTypes>;
#endif
#endif //defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_CPP)

#endif //SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_H
