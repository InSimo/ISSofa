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
// C++ Implementation: FastTriangularBendingSprings
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_INL
#define SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_INL

#include <SofaDeformable/FastTriangularBendingSprings.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/topology/TopologyChange.h>
#include <fstream> // for reading the file
#include <iostream> //for debugging

#include <SofaBaseTopology/TopologyData.inl>

#include <sofa/helper/AdvancedTimer.h>
#include <SofaBaseLinearSolver/BlocMatrixWriter.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

typedef sofa::core::topology::BaseMeshTopology::EdgesInTriangle EdgesInTriangle;

template< class DataTypes>
void FastTriangularBendingSprings<DataTypes>::TriangularBSEdgeHandler::applyCreateFunction(unsigned int /*edgeIndex*/, VecEdgeSpring &ei, const sofa::component::topology::Edge &, const sofa::helper::vector<unsigned int> &, const sofa::helper::vector<double> &)
{
    if (ff)
    {
        ei.springs.clear();
    }
}



template< class DataTypes>
void FastTriangularBendingSprings<DataTypes>::TriangularBSEdgeHandler::applyTriangleCreation(const sofa::helper::vector<unsigned int> &triangleAdded, const sofa::helper::vector<sofa::component::topology::Triangle> &, const sofa::helper::vector<sofa::helper::vector<unsigned int> > &, const sofa::helper::vector<sofa::helper::vector<double> > &)
{
    using namespace sofa::component::topology;
    if (ff)
    {
        for (unsigned int i=0; i<triangleAdded.size(); ++i)
        {
            /// edges of the new triangle
            EdgesInTriangle te2 = ff->_topology->getEdgesInTriangle(triangleAdded[i]);
            // for each edge in the new triangle
            for (unsigned int j = 0; j < 3; ++j)
            {
                unsigned int edgeIndex = te2[j];
                const sofa::helper::vector< unsigned int > shell = ff->_topology->getTrianglesAroundEdge(edgeIndex);
                ff->createBendingSprings(edgeIndex, shell);

            }
        }
    }
}


template< class DataTypes>
void FastTriangularBendingSprings<DataTypes>::TriangularBSEdgeHandler::applyTriangleDestruction(const sofa::helper::vector<unsigned int> &triangleRemoved)
{
    if (ff)
    {
        for (unsigned int i = 0; i < triangleRemoved.size(); ++i)
        {
            /// describe the jth edge index of triangle no i
            EdgesInTriangle te = ff->_topology->getEdgesInTriangle(triangleRemoved[i]);
            /// describe the jth vertex index of triangle no i
            //Triangle t = ff->_topology->getTriangle(triangleRemoved[i]);

            for (unsigned int j = 0; j < 3; ++j)
            {
                unsigned int edgeIndex = te[j];

                const sofa::helper::vector< unsigned int > shell = ff->_topology->getTrianglesAroundEdge(edgeIndex);
                sofa::helper::vector< unsigned int> keepingTri;
                for (unsigned int k = 0; k < shell.size(); ++k)
                {
                    if (std::find(triangleRemoved.begin(), triangleRemoved.end(), shell[k]) == triangleRemoved.end())
                    {
                        keepingTri.push_back(shell[k]);
                    }
                }

                ff->createBendingSprings(edgeIndex, keepingTri);

            }
        }
    }

}

template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::TriangularBSEdgeHandler::ApplyTopologyChange(const core::topology::TrianglesAdded* e)
{
    using namespace sofa::component::topology;
    const sofa::helper::vector<unsigned int> &triangleAdded = e->getIndexArray();
    const sofa::helper::vector<Triangle> &elems = e->getElementArray();
    const sofa::helper::vector<sofa::helper::vector<unsigned int> > & ancestors = e->ancestorsList;
    const sofa::helper::vector<sofa::helper::vector<double> > & coefs = e->coefs;

    applyTriangleCreation(triangleAdded, elems, ancestors, coefs);
}

template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::TriangularBSEdgeHandler::ApplyTopologyChange(const core::topology::TrianglesRemoved* e)
{
    const sofa::helper::vector<unsigned int> &triangleRemoved = e->getArray();

    applyTriangleDestruction(triangleRemoved);
}

template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::TriangularBSEdgeHandler::applyPointDestruction(const sofa::helper::vector<unsigned int> &tab)
{
    if(ff)
    {
        bool debug_mode = false;

        unsigned int last = ff->_topology->getNbPoints() -1;
        unsigned int i,j;

        helper::vector<VecEdgeSpring>& edgeInf = *(ff->d_edgeSprings.beginEdit());

        //make a reverse copy of tab
        sofa::helper::vector<unsigned int> lastIndexVec;
        lastIndexVec.reserve(tab.size());
        for(unsigned int i_init = 0; i_init < tab.size(); ++i_init)
            lastIndexVec.push_back(last - i_init);

        for ( i = 0; i < tab.size(); ++i)
        {
            unsigned int i_next = i;
            bool is_reached = false;
            while( (!is_reached) && (i_next < lastIndexVec.size() - 1))
            {
                ++i_next;
                is_reached = (lastIndexVec[i_next] == tab[i]);
            }

            if(is_reached)
                lastIndexVec[i_next] = lastIndexVec[i];

            const sofa::helper::vector<unsigned int> &shell= ff->_topology->getTrianglesAroundVertex(lastIndexVec[i]);
            for (j=0; j<shell.size(); ++j)
            {
                EdgesInTriangle tej = ff->_topology->getEdgesInTriangle(shell[j]);
                for(unsigned int k=0; k < 3 ; ++k)
                {
                    unsigned int ind_j = tej[k];
                    for (unsigned int l = 0; l < edgeInf[ind_j].springs.size(); l++)
                    {
                        edgeInf[ind_j].springs[l].replaceIndex(last, tab[i]);
                    }
                }
            }

            if(debug_mode)
            {
                for (unsigned int j_loc=0; j_loc<edgeInf.size(); ++j_loc)
                {
                    for (unsigned int l = 0; l < edgeInf[j_loc].springs.size(); l++)
                    {
                        edgeInf[j_loc].springs[l].replaceIndex(last, tab[i]);
                    }
                }
            }

            --last;
        }

        ff->d_edgeSprings.endEdit();
    }
}


template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::TriangularBSEdgeHandler::applyPointRenumbering(const sofa::helper::vector<unsigned int> &newIndices)
{
    if(ff)
    {
        helper::vector<VecEdgeSpring>& edgeInf = *(ff->d_edgeSprings.beginEdit());
        for (int i = 0; i < ff->_topology->getNbEdges(); ++i)
        {
            for (unsigned int j = 0; j < edgeInf[i].springs.size(); j++)
            {
                if (edgeInf[i].springs[j].is_activated)
                {
                    edgeInf[i].springs[j].replaceIndices(newIndices);
                }
            }
        }
        ff->d_edgeSprings.endEdit();
    }
}

template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::TriangularBSEdgeHandler::ApplyTopologyChange(const core::topology::PointsRemoved* e)
{
    const sofa::helper::vector<unsigned int> & tab = e->getArray();
    applyPointDestruction(tab);
}

template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::TriangularBSEdgeHandler::ApplyTopologyChange(const core::topology::PointsRenumbering* e)
{
    const sofa::helper::vector<unsigned int> &newIndices = e->getIndexArray();
    applyPointRenumbering(newIndices);
}

template<class DataTypes>
FastTriangularBendingSprings<DataTypes>::FastTriangularBendingSprings()
: f_bendingStiffness(initData(&f_bendingStiffness,(double) 1.0,"bendingStiffness","bending stiffness of the material"))
, d_minDistValidity(initData(&d_minDistValidity,(double) 0.000001,"minDistValidity","Distance under which a spring is not valid"))
, d_useRestCurvature(initData(&d_useRestCurvature, false, "useRestCurvature", "Use rest curvature as the zero potential energy"))
, d_useCorotational(initData(&d_useCorotational, false, "useCorotational","Use a rotation to make rest curvature invariant to rotation"))
, d_forceContinuity(initData(&d_forceContinuity, false, "forceContinuity","handle discontinuity for 180° edge rotation by using an hysterisis"))
, d_quadraticBendingModel(initData(&d_quadraticBendingModel, false,"quadraticBendingModel","Use quadratic bending model method for Inextensible Surfaces"))
, d_drawMaxSpringEnergy(initData(&d_drawMaxSpringEnergy,(Real) 1.0,"drawMaxSpringEnergy","Maximum value of spring bending energy to draw"))
, d_drawSpringSize(initData(&d_drawSpringSize, 1.0f,"drawSpringSize","Size of drawed lines"))
, d_drawSpringBase(initData(&d_drawSpringBase, false,"drawSpringBase","draw orthogonal base use for corotational"))
, d_edgeSprings(initData(&d_edgeSprings, "edgeInfo", "Internal edge data"))
, edgeHandler(NULL)
{
    // Create specific handler for EdgeData
    edgeHandler = new TriangularBSEdgeHandler(this, &d_edgeSprings);
    //serr<<"FastTriangularBendingSprings<DataTypes>::FastTriangularBendingSprings"<<sendl;
}

template<class DataTypes>
FastTriangularBendingSprings<DataTypes>::~FastTriangularBendingSprings()
{
    if(edgeHandler) delete edgeHandler;
}


template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::init()
{
    //serr << "initializing FastTriangularBendingSprings" << sendl;
    this->Inherited::init();

    _topology = this->getContext()->getMeshTopology();

    if (!_topology)
    {
        serr << "ERROR(FastTriangularBendingSprings): object must have a Triangular Set Topology."<<sendl;
        return;
    }
    d_edgeSprings.createTopologicalEngine(_topology,edgeHandler);
    d_edgeSprings.linkToPointDataArray();
    d_edgeSprings.linkToTriangleDataArray();
    d_edgeSprings.registerTopologicalData();

    this->reinit();
}


template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::reinit()
{
    using namespace sofa::component::topology;
    /// prepare to store info in the edge array
    helper::vector<VecEdgeSpring>& edgeInf = *(d_edgeSprings.beginEdit());
    edgeInf.resize(_topology->getNbEdges());
    int i;
    // set edge tensor to 0
    for (i=0; i<_topology->getNbEdges(); ++i)
    {

        edgeHandler->applyCreateFunction(i, edgeInf[i],
                _topology->getEdge(i),  (const sofa::helper::vector< unsigned int > )0,
                (const sofa::helper::vector< double >)0);
    }

    // create edge tensor by calling the triangle creation function
    sofa::helper::vector<unsigned int> triangleAdded;
    for (i=0; i<_topology->getNbTriangles(); ++i)
        triangleAdded.push_back(i);

    edgeHandler->applyTriangleCreation(triangleAdded,
            (const sofa::helper::vector<Triangle>)0,
            (const sofa::helper::vector<sofa::helper::vector<unsigned int> >)0,
            (const sofa::helper::vector<sofa::helper::vector<double> >)0);

    d_edgeSprings.endEdit();
}


template< class DataTypes>
void FastTriangularBendingSprings<DataTypes>::createBendingSprings(unsigned int edgeIndex, const sofa::helper::vector<unsigned int>& shell)
{

    typename MechanicalState::ReadVecCoord restPosition = this->mstate->readRestPositions();
    const bool quadraticBendingModel = d_quadraticBendingModel.getValue();
    const Real bendingStiffness = (Real)f_bendingStiffness.getValue();
    const bool useRestCurvature = d_useRestCurvature.getValue();
    const bool useCorotational = d_useCorotational.getValue();
    const bool forceContinuity = d_forceContinuity.getValue();

    sofa::helper::WriteAccessor<sofa::Data<sofa::helper::vector<VecEdgeSpring> > > edgeData(d_edgeSprings);
    double epsilonSq = d_minDistValidity.getValue();
    epsilonSq *= epsilonSq;
    VecEdgeSpring& ei = edgeData[edgeIndex]; // edge spring
    ei.springs.resize(1);
    ei.springs[0].resetCache();

    if (shell.size() == 2)   // there is another triangle attached to this edge, so a spring is needed
    {
        // the other triangle and its edges
        EdgesInTriangle te1 = _topology->getEdgesInTriangle(shell[0]);
        Triangle t1 = _topology->getTriangle(shell[0]);

        EdgesInTriangle te2 = _topology->getEdgesInTriangle(shell[1]);
        Triangle t2 = _topology->getTriangle(shell[1]);

        int i1 = _topology->getEdgeIndexInTriangle(te1, edgeIndex); // index of the vertex opposed to the current edge in the other triangle (?)
        int i2 = _topology->getEdgeIndexInTriangle(te2, edgeIndex); // index of the vertex opposed to the current edge in the new triangle (?)
        Edge edge = _topology->getEdge(edgeIndex);                  // indices of the vertices of the current edge

        const PointID& v1 = t1[i1];
        const PointID& v2 = t2[i2];
        const PointID& e1 = edge[0];
        const PointID& e2 = edge[1];

        Deriv vp = restPosition[v2] - restPosition[v1];
        Deriv ve = restPosition[e2] - restPosition[e1];

        if (vp.norm2()>epsilonSq && ve.norm2()>epsilonSq)
        {
            if (!quadraticBendingModel)
            {
                ei.springs[0].setEdgeSpring(restPosition.ref(), v1, v2, e1, e2, bendingStiffness);
            }
            if (quadraticBendingModel)
            {
                ei.springs[0].setEdgeSpringQuadratic(restPosition.ref(), v1, v2, e1, e2, bendingStiffness);
            }
            if (useRestCurvature)
            {
                ei.springs[0].computeRestCurvature(restPosition.ref(), useCorotational);
                if (forceContinuity)
                {
                    const Deriv u = ve.normalized();
                    const Deriv n1 = u.cross(restPosition[v1] - restPosition[e1]).normalized();
                    const Deriv n2 = -u.cross(restPosition[v2] - restPosition[e1]).normalized();
                    ei.springs[0].quadrant = ei.springs[0].computeShellQuadrant(n1, n2, u);
                }
            }
        }
    }
    else
        ei.springs[0].is_activated = ei.springs[0].is_initialized = false;
}

template <class DataTypes>
double FastTriangularBendingSprings<DataTypes>::getPotentialEnergy(const core::MechanicalParams*, const DataVecCoord&) const
{
    return m_potentialEnergy;
}


template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::addForce(const core::MechanicalParams* /* mparams */ /* PARAMS FIRST */, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v)
{
    const VecCoord& x = d_x.getValue();
    const VecDeriv& v = d_v.getValue();
    typename MechanicalState::WriteVecDeriv f(d_f);
    sofa::helper::WriteAccessor<sofa::Data<sofa::helper::vector<VecEdgeSpring>>> edgeInf = d_edgeSprings;
    const bool useCorotational = d_useCorotational.getValue();
    f.resize(x.size());

    m_potentialEnergy = 0;
    for(unsigned i=0; i<edgeInf.size(); i++ )
    {
        for (unsigned int j = 0; j < edgeInf[i].springs.size(); j++)
        {
            m_potentialEnergy += edgeInf[i].springs[j].addForce(f.wref(), x, v, useCorotational);
        }
    }
}

template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::addDForce(const core::MechanicalParams* mparams , DataVecDeriv& d_df, const DataVecDeriv& d_dx)
{
    const VecDeriv& dx = d_dx.getValue();
    typename MechanicalState::WriteVecDeriv df(d_df);
    const helper::vector<VecEdgeSpring>& edgeInf = d_edgeSprings.getValue();
    const Real kFactor = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());
    df.resize(dx.size());

    sofa::helper::AdvancedTimer::stepBegin("FTBendingSpringAddDForce");

    for(unsigned i=0; i<edgeInf.size(); i++ )
    {
        for (unsigned int j = 0; j < edgeInf[i].springs.size(); j++)
        {
            edgeInf[i].springs[j].addDForce(df.wref(), dx, kFactor);
        }
    }

    sofa::helper::AdvancedTimer::stepEnd("FTBendingSpringAddDForce");
}

template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    linearsolver::BlocMatrixWriter<Mat> writer;
    writer.addKToMatrix(this, mparams, matrix->getMatrix(this->mstate));
}

template<class DataTypes>
template<class MatrixWriter>
void FastTriangularBendingSprings<DataTypes>::addKToMatrixT(const core::MechanicalParams* mparams, MatrixWriter mwriter)
{
    const Real kFactor = (Real)mparams->kFactor();
    const helper::vector<VecEdgeSpring>& vecsprings = d_edgeSprings.getValue();

    for (unsigned i=0; i< vecsprings.size() ; i++)
    {
        for (unsigned int j = 0; j < vecsprings[i].springs.size(); j++)
        {
            if (vecsprings[i].springs[j].is_activated)
            {
                vecsprings[i].springs[j].addStiffness(mwriter, kFactor);
            }
        }
    }
}


template<class DataTypes>
void FastTriangularBendingSprings<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    Inherited::draw(vparams);

    unsigned int i;
    if (!vparams->displayFlags().getShowForceFields()) return;
    if (!this->mstate) return;

    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();
    sofa::helper::WriteAccessor<sofa::Data<sofa::helper::vector<VecEdgeSpring>>> edgeInf = d_edgeSprings;
    const Real maxValue = d_drawMaxSpringEnergy.getValue();
    const bool useCorotational = d_useCorotational.getValue();
    const Real drawSpringSize = d_drawSpringSize.getValue();

    VecDeriv emptyVecDeriv;
    emptyVecDeriv.resize(x.size());

    for (i = 0; i<edgeInf.size(); ++i)
    {
        if (edgeInf[i].springs.size() > 0)
        {
            sofa::helper::vector<sofa::defaulttype::Vector3> points;
            const Coord pa[2] = { x[edgeInf[i].springs[0].vid[EdgeSpring::VC]], x[edgeInf[i].springs[0].vid[EdgeSpring::VD]] };
            points.assign(pa, pa + 2);
            Real energy = 0;
            bool is_activated = false;
            for (unsigned int j = 0; j < edgeInf[i].springs.size(); j++)
            {
                if (edgeInf[i].springs[j].is_activated)
                {
                    energy += edgeInf[i].springs[j].addForce(emptyVecDeriv, x, emptyVecDeriv, useCorotational);
                    is_activated = true;
                }
            }

            float R = 0.;
            float G = 0.;
            float B = 1.;

            B = (float)((maxValue - energy) / maxValue);
            B = (B < 0.f) ? 0.f : B;
            R = 1.f - B;

            if (is_activated)
            {
                vparams->drawTool()->drawLines(points, drawSpringSize, sofa::defaulttype::Vec4f(R, G, B, 1.f));

                if (d_drawSpringBase.getValue())
                {
                    const Coord middle = (pa[0]+pa[1])*0.5;
                    for (unsigned int j = 0; j < edgeInf[i].springs.size(); j++)
                    {
                        Mat Kx;
                        edgeInf[i].springs[j].computeSpringRotation(Kx, x);
                        vparams->drawTool()->drawArrow(middle, middle + Coord(Kx[0][0], Kx[1][0], Kx[2][0]) * 10*drawSpringSize, drawSpringSize, sofa::defaulttype::Vec4f(1, 0, 0, 1.f));
                        vparams->drawTool()->drawArrow(middle, middle + Coord(Kx[0][1], Kx[1][1], Kx[2][1]) * 10*drawSpringSize, drawSpringSize, sofa::defaulttype::Vec4f(0, 1, 0, 1.f));
                        vparams->drawTool()->drawArrow(middle, middle + Coord(Kx[0][2], Kx[1][2], Kx[2][2]) * 10*drawSpringSize, drawSpringSize, sofa::defaulttype::Vec4f(0, 0, 1, 1.f));


                        Deriv R = edgeInf[i].springs[j].computeBendingVector(x);
                        Deriv R0 = Kx*edgeInf[i].springs[j].R0;
                        vparams->drawTool()->drawArrow(middle, middle +R * 10*drawSpringSize, drawSpringSize, sofa::defaulttype::Vec4f(0.5, 0.5, 0, 1.f));
                        vparams->drawTool()->drawArrow(middle, middle + R0 * 10*drawSpringSize, drawSpringSize, sofa::defaulttype::Vec4f(0.0, 0.5, 0.5, 1.f));

                    }
                }
            }
        }
    }
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif //#ifndef SOFA_COMPONENT_FORCEFIELD_FastTriangularBendingSprings_INL

