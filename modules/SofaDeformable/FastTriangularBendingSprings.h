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

#include <sofa/defaulttype/Mat.h>
#include <SofaBaseTopology/TopologyData.h>

#include <cassert>
#include <math.h>
#include <iostream>

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
    typedef core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef sofa::core::topology::Edge     Edge;
    typedef sofa::core::topology::Triangle Triangle;
    typedef sofa::core::topology::Topology::PointID PointID;
    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;

    enum { N=DataTypes::spatial_dimensions };
    typedef defaulttype::Mat<N,N,Real> Mat;


    Data<double> f_bendingStiffness;  ///< Material parameter

	Data<double> d_minDistValidity; ///< Minimal distance to consider a spring valid

    Data<bool>   d_useRestCurvature; ///< Use the rest curvature as the zero energy bending.
    Data<bool>   d_useOldAddForce; //warning: bug version

    Data<bool>   d_quadraticBendingModel; /// Use quadratic bending model method for Inextensible Surfaces

    /// Searches triangle topology and creates the bending springs
    virtual void init();

    virtual void reinit();

    virtual void addForce(const core::MechanicalParams* mparams, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v);
    virtual void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& d_df, const DataVecDeriv& d_dx);
    virtual void addKToMatrix(sofa::defaulttype::BaseMatrix *mat, SReal k, unsigned int &offset); // compute and add all the element stiffnesses to the global stiffness matrix
    virtual double getPotentialEnergy(const core::MechanicalParams* mparams, const DataVecCoord& d_x) const;

    void draw(const core::visual::VisualParams* vparams);

protected:

    class EdgeSpring
    {
    public:
        enum {A=0,B,C,D};                        ///< vertex names as in Volino's paper
        sofa::defaulttype::Vec<4,unsigned> vid;  ///< vertex indices, in circular order
        sofa::defaulttype::Vec<4,Real> alpha;    ///< weight of each vertex in the bending vector
        Real lambda;                             ///< bending stiffness
        Deriv R0;                                ///< rest curvature;
        bool is_activated;
        bool is_initialized;

        typedef defaulttype::Mat<12,12,Real> StiffnessMatrix;

        /// Store the vertex indices and perform all the precomputations
        void setEdgeSpring( const VecCoord& p, unsigned iA, unsigned iB, unsigned iC, unsigned iD, Real materialBendingStiffness, bool computeRestCurvature=false )
        {
            is_activated = is_initialized = true;

            vid[A]=iA;
            vid[B]=iB;
            vid[C]=iC;
            vid[D]=iD;

            Deriv NA = cross( p[vid[A]]-p[vid[C]], p[vid[A]]-p[vid[D]] );
            Deriv NB = cross( p[vid[B]]-p[vid[D]], p[vid[B]]-p[vid[C]] );
            Deriv NC = cross( p[vid[C]]-p[vid[B]], p[vid[C]]-p[vid[A]] );
            Deriv ND = cross( p[vid[D]]-p[vid[A]], p[vid[D]]-p[vid[B]] );

            alpha[A] =  NB.norm() / (NA.norm() + NB.norm());
            alpha[B] =  NA.norm() / (NA.norm() + NB.norm());
            alpha[C] = -ND.norm() / (NC.norm() + ND.norm());
            alpha[D] = -NC.norm() / (NC.norm() + ND.norm());

            // stiffness
            Deriv edgeDir = p[vid[C]]-p[vid[D]];
            edgeDir.normalize();
            Deriv AC = p[vid[C]]-p[vid[A]];
            Deriv BC = p[vid[C]]-p[vid[B]];
            Real ha = (AC - edgeDir * (AC*edgeDir)).norm(); // distance from A to CD
            Real hb = (BC - edgeDir * (BC*edgeDir)).norm(); // distance from B to CD
            Real l = (p[vid[C]]-p[vid[D]]).norm();          // distance from C to D
            lambda = (Real)(2./3) * (ha+hb)/(ha*ha*hb*hb) * l * materialBendingStiffness;

            if(computeRestCurvature )
            {
                R0 = computeBendingVector( p );
            }
            //            cerr<<"EdgeInformation::setEdgeSpring, vertices = " << vid << endl;
        }

        /// For each edge includes between two triangles -> calculate coefficients "coef" and "K0" used in Matrix Q(ei) function edge ei
        void setEdgeSpringQuadratic( const VecCoord& p, unsigned iA, unsigned iB, unsigned iC, unsigned iD, Real materialBendingStiffness, bool computeRestCurvature=false )
        {
            is_activated = is_initialized = true;
 
            vid[A]=iD;  ///< WARNING vertex names and orientation as in Bergou's paper, different compared Linear method from Volino's paper
            vid[B]=iC;
            vid[C]=iA;
            vid[D]=iB;

            Deriv e0 = p[vid[B]]-p[vid[A]];
            Deriv e1 = p[vid[C]]-p[vid[A]];
            Deriv e2 = p[vid[D]]-p[vid[A]];
            Deriv e3 = p[vid[C]]-p[vid[B]];
            Deriv e4 = p[vid[D]]-p[vid[B]];

            double c01 = cotTheta( e0, e1);
            double c02 = cotTheta( e0, e2);
            double c03 = cotTheta(-e0, e3);
            double c04 = cotTheta(-e0, e4);

            alpha[A] = c03+c04;
            alpha[B] = c01+c02;
            alpha[C] = -c01-c03;
            alpha[D] = -c02-c04;

            double A0 = 0.5 * cross(e0,e1).norm();
            double A1 = 0.5 * cross(e0,e2).norm();

            lambda = ( 3. / (2.*(A0+A1))) * materialBendingStiffness;

            if(computeRestCurvature )
            {
                R0 = computeBendingVector( p );
            }

            assert(finite(c01));
            assert(finite(c02));
            assert(finite(c03));
            assert(finite(c04));
        }

        double cotTheta(const Deriv& v, const Deriv& w)
        {
            assert(finite(v.norm()));
            assert(finite(w.norm()));
            assert(v.norm() > 0);
            assert(w.norm () > 0);
            const double cosTheta = dot(v,w);
            const double sinTheta = cross(v,w).norm();

            return (cosTheta / sinTheta);
        }

        Deriv computeBendingVector( const VecCoord& p) const
        {
           return ( p[vid[A]]*alpha[A] +  p[vid[B]]*alpha[B] +  p[vid[C]]*alpha[C] +  p[vid[D]]*alpha[D] );
        }

        /// Accumulates force and return potential energy
        Real addForce( VecDeriv& f, const VecCoord& p, const VecDeriv& /*v*/) const
        {
            if( !is_activated ) return 0;

            Deriv R = computeBendingVector(p);
            R -= R0;
            f[vid[A]] -= R * (lambda * alpha[A]);
            f[vid[B]] -= R * (lambda * alpha[B]);
            f[vid[C]] -= R * (lambda * alpha[C]);
            f[vid[D]] -= R * (lambda * alpha[D]);

            return (R * R) * lambda * (Real)0.5;
        }

        void addDForceBugged( VecDeriv& df, const VecDeriv& dp, Real kfactor) const
        {
            if( !is_activated ) return;

            Deriv R = (dp[vid[0]] * (kfactor *  alpha[0]) + dp[vid[1]] * (kfactor * alpha[1]) + dp[vid[2]] * (kfactor * alpha[2]) + dp[vid[3]] * (kfactor * alpha[3]));
            df[vid[0]] -= R*alpha[0];
            df[vid[1]] -= R*alpha[1];
            df[vid[2]] -= R*alpha[2];
            df[vid[3]] -= R*alpha[3];
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
        void addStiffness( sofa::defaulttype::BaseMatrix *bm, unsigned int offset, SReal scale, core::behavior::ForceField< _DataTypes>* ff ) const
        {
            StiffnessMatrix K;
            getStiffness( K );
            ff->addToMatrix(bm,offset,vid,K,scale);
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



        /// Output stream
        inline friend std::ostream& operator<< ( std::ostream& os, const EdgeSpring& /*ei*/ )
        {
            return os;
        }

        /// Input stream
        inline friend std::istream& operator>> ( std::istream& in, EdgeSpring& /*ei*/ )
        {
            return in;
        }
    };

    /// The list of edge springs, one for each edge between two triangles
    sofa::component::topology::EdgeData<helper::vector<EdgeSpring> > edgeSprings;

    class TriangularBSEdgeHandler : public sofa::component::topology::TopologyDataHandler<Edge,vector<EdgeSpring> >
    {
    public:
        typedef typename FastTriangularBendingSprings<DataTypes>::EdgeSpring EdgeSpring;
        TriangularBSEdgeHandler(FastTriangularBendingSprings<DataTypes>* _ff, sofa::component::topology::EdgeData<sofa::helper::vector<EdgeSpring> >* _data)
            : sofa::component::topology::TopologyDataHandler<Edge, sofa::helper::vector<EdgeSpring> >(_data), ff(_ff) {}

        void applyCreateFunction(unsigned int edgeIndex,
                EdgeSpring &ei,
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

    sofa::component::topology::EdgeData<helper::vector<EdgeSpring> > &getEdgeInfo() {return edgeSprings;}

    TriangularBSEdgeHandler* edgeHandler;

    double m_potentialEnergy;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_DEFORMABLE_API FastTriangularBendingSprings<defaulttype::Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_DEFORMABLE_API FastTriangularBendingSprings<defaulttype::Vec3fTypes>;
#endif
#endif //defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_CPP)


} // namespace forcefield

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_H
