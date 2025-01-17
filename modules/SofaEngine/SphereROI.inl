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
#ifndef SOFA_COMPONENT_ENGINE_SPHEREROI_INL
#define SOFA_COMPONENT_ENGINE_SPHEREROI_INL

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <SofaEngine/SphereROI.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/gl/BasicShapes.h>

namespace sofa
{

namespace component
{

namespace engine
{

template <class DataTypes>
SphereROI<DataTypes>::SphereROI()
    : centers( initData(&centers, "centers", "Center(s) of the sphere(s)") )
    , radii( initData(&radii, "radii", "Radius(i) of the sphere(s)") )
    , direction( initData(&direction, "direction", "Edge direction(if edgeAngle > 0)") )
    , normal( initData(&normal, "normal", "Normal direction of the triangles (if triAngle > 0)") )
    , edgeAngle( initData(&edgeAngle, (Real)0, "edgeAngle", "Max angle between the direction of the selected edges and the specified direction") )
    , triAngle( initData(&triAngle, (Real)0, "triAngle", "Max angle between the normal of the selected triangle and the specified normal direction") )
    , f_X0( initData (&f_X0, "position", "Rest position coordinates of the degrees of freedom") )
    , f_edges(initData (&f_edges, "edges", "Edge Topology") )
    , f_triangles(initData (&f_triangles, "triangles", "Triangle Topology") )
    , f_quads(initData (&f_quads, "quads", "Quads Topology") )
    , f_tetrahedra(initData (&f_tetrahedra, "tetrahedra", "Tetrahedron Topology") )
    , f_computeEdges( initData(&f_computeEdges, true,"computeEdges","If true, will compute edge list and index list inside the ROI.") )
    , f_computeTriangles( initData(&f_computeTriangles, true,"computeTriangles","If true, will compute triangle list and index list inside the ROI.") )
    , f_computeQuads( initData(&f_computeQuads, true,"computeQuads","If true, will compute quad list and index list inside the ROI.") )
    , f_computeTetrahedra( initData(&f_computeTetrahedra, true,"computeTetrahedra","If true, will compute tetrahedra list and index list inside the ROI.") )
    , f_strictlyInROI( initData(&f_strictlyInROI, true,"strictlyInROI","If true, all points of the a given primitive should be inside the ROI.") )
    , f_indices( initData(&f_indices,"indices","Indices of the points contained in the ROI") )
    , f_edgeIndices( initData(&f_edgeIndices,"edgeIndices","Indices of the edges contained in the ROI") )
    , f_triangleIndices( initData(&f_triangleIndices,"triangleIndices","Indices of the triangles contained in the ROI") )
    , f_quadIndices( initData(&f_quadIndices,"quadIndices","Indices of the quads contained in the ROI") )
    , f_tetrahedronIndices( initData(&f_tetrahedronIndices,"tetrahedronIndices","Indices of the tetrahedra contained in the ROI") )
    , f_pointsInROI( initData(&f_pointsInROI,"pointsInROI","Points contained in the ROI") )
    , f_edgesInROI( initData(&f_edgesInROI,"edgesInROI","Edges contained in the ROI") )
    , f_trianglesInROI( initData(&f_trianglesInROI,"trianglesInROI","Triangles contained in the ROI") )
    , f_quadsInROI( initData(&f_quadsInROI,"quadsInROI","Quads contained in the ROI") )
    , f_tetrahedraInROI( initData(&f_tetrahedraInROI,"tetrahedraInROI","Tetrahedra contained in the ROI") )
    , f_indicesOut( initData(&f_indicesOut,"indicesOut","Indices of the points not contained in the ROI") )
    , f_edgeIndicesOut( initData(&f_edgeIndicesOut,"edgeIndicesOut","Indices of the edges not contained in the ROI") )
    , f_triangleIndicesOut( initData(&f_triangleIndicesOut,"triangleIndicesOut","Indices of the triangles not contained in the ROI") )
    , f_quadIndicesOut( initData(&f_quadIndicesOut,"quadIndicesOut","Indices of the quads not contained in the ROI") )
    , f_tetrahedronIndicesOut( initData(&f_tetrahedronIndicesOut,"tetrahedronIndicesOut","Indices of the tetrahedra not contained in the ROI") )
    , f_pointsOutROI( initData(&f_pointsOutROI,"pointsOutROI","Points not contained in the ROI") )
    , f_edgesOutROI( initData(&f_edgesOutROI,"edgesOutROI","Edges not contained in the ROI") )
    , f_trianglesOutROI( initData(&f_trianglesOutROI,"trianglesOutROI","Triangles not contained in the ROI") )
    , f_quadsOutROI( initData(&f_quadsOutROI,"quadsOutROI","Quads not contained in the ROI") )
    , f_tetrahedraOutROI( initData(&f_tetrahedraOutROI,"tetrahedraOutROI","Tetrahedra not contained in the ROI") )
    , p_drawSphere( initData(&p_drawSphere,false,"drawSphere","Draw shpere(s)") )
    , p_drawPoints( initData(&p_drawPoints,false,"drawPoints","Draw Points") )
    , p_drawEdges( initData(&p_drawEdges,false,"drawEdges","Draw Edges") )
    , p_drawTriangles( initData(&p_drawTriangles,false,"drawTriangles","Draw Triangles") )
    , p_drawQuads( initData(&p_drawQuads,false,"drawQuads","Draw Quads") )
    , p_drawTetrahedra( initData(&p_drawTetrahedra,false,"drawTetrahedra","Draw Tetrahedra") )
    , _drawSize( initData(&_drawSize,0.0,"drawSize","rendering size for box and topological elements") )
{
    //Adding alias to handle TrianglesInSphereROI input/output
    addAlias(&p_drawSphere,"isVisible");
    addAlias(&triAngle,"angle");
    addAlias(&f_indices,"pointIndices");
    addAlias(&f_X0,"rest_position");

    f_indices.beginEdit()->push_back(0);
    f_indices.endEdit();
}

template <class DataTypes>
void SphereROI<DataTypes>::init()
{
    using sofa::core::objectmodel::BaseData;
    using sofa::core::topology::BaseMeshTopology;

    if (!f_X0.isSet())
    {
        sofa::core::behavior::MechanicalState<DataTypes>* mstate;
        this->getContext()->get(mstate);
        if (mstate)
        {
            BaseData* parent = mstate->findData("rest_position");
            if (parent)
            {
                f_X0.setParent(parent);
                f_X0.setReadOnly(true);
            }
        }
        else
        {
            core::loader::MeshLoader* loader = NULL;
            this->getContext()->get(loader);
            if (loader)
            {
                BaseData* parent = loader->findData("position");
                if (parent)
                {
                    f_X0.setParent(parent);
                    f_X0.setReadOnly(true);
                }
            }
        }
    }
    if (!f_edges.isSet() || !f_triangles.isSet() || !f_quads.isSet() || !f_tetrahedra.isSet())
    {
        BaseMeshTopology* topology;
        this->getContext()->get(topology);
        if (topology)
        {
            if (!f_edges.isSet() && f_computeEdges.getValue())
            {
                BaseData* eparent = topology->findData("edges");
                if (eparent)
                {
                    f_edges.setParent(eparent);
                    f_edges.setReadOnly(true);
                }
            }
            if (!f_triangles.isSet() && f_computeTriangles.getValue())
            {
                BaseData* tparent = topology->findData("triangles");
                if (tparent)
                {
                    f_triangles.setParent(tparent);
                    f_triangles.setReadOnly(true);
                }
            }
            if (!f_quads.isSet() && f_computeQuads.getValue())
            {
                BaseData* tparent = topology->findData("quads");
                if (tparent)
                {
                    f_quads.setParent(tparent);
                    f_quads.setReadOnly(true);
                }
            }
            if (!f_tetrahedra.isSet() && f_computeTetrahedra.getValue())
            {
                BaseData* tparent = topology->findData("tetrahedra");
                if (tparent)
                {
                    f_tetrahedra.setParent(tparent);
                    f_tetrahedra.setReadOnly(true);
                }
            }
        }
    }

    addInput(&f_X0);
    addInput(&f_edges);
    addInput(&f_triangles);
    addInput(&f_quads);
    addInput(&f_tetrahedra);

    addInput(&centers);
    addInput(&radii);
    addInput(&direction);
    addInput(&normal);
    addInput(&edgeAngle);
    addInput(&triAngle);

    addOutput(&f_indices);
    addOutput(&f_edgeIndices);
    addOutput(&f_triangleIndices);
    addOutput(&f_quadIndices);
    addOutput(&f_tetrahedronIndices);
    addOutput(&f_pointsInROI);
    addOutput(&f_edgesInROI);
    addOutput(&f_trianglesInROI);
    addOutput(&f_quadsInROI);
    addOutput(&f_tetrahedraInROI);

    addOutput(&f_indicesOut);
    addOutput(&f_edgeIndicesOut);
    addOutput(&f_triangleIndicesOut);
    addOutput(&f_quadIndicesOut);
    addOutput(&f_tetrahedronIndicesOut);
    addOutput(&f_pointsOutROI);
    addOutput(&f_edgesOutROI);
    addOutput(&f_trianglesOutROI);
    addOutput(&f_quadsOutROI);
    addOutput(&f_tetrahedraOutROI);

    setDirtyValue();
}

template <class DataTypes>
void SphereROI<DataTypes>::reinit()
{
    this->requestUpdate();
}


template <class DataTypes>
bool SphereROI<DataTypes>::isPointInSphere(const Vec3& c, const Real& r, const Coord& p)
{
    if((p-c).norm() > r)
        return false;
    else
        return true;
}

template <class DataTypes>
bool SphereROI<DataTypes>::isPointInSphere(const PointID& pid, const Real& r, const Coord& p)
{
    const VecCoord* x0 = &f_X0.getValue();
    CPos c =  DataTypes::getCPos((*x0)[pid]);
    return ( isPointInSphere(c, r, p) );
}

template <class DataTypes>
bool SphereROI<DataTypes>::isEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge)
{
    const VecCoord* x0 = &f_X0.getValue();
    for (unsigned int i=0; i<2; ++i)
    {
        Coord p = (*x0)[edge[i]];

        if((p-c).norm() > r)
            return false;
    }
    return true;
}

template <class DataTypes>
bool SphereROI<DataTypes>::isPartOfEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge)
{
    const VecCoord* x0 = &f_X0.getValue();
    for (unsigned int i=0; i<2; ++i)
    {
        Coord p = (*x0)[edge[i]];

        if((p-c).norm() < r)
            return true;
    }
    return false;
}

template <class DataTypes>
bool SphereROI<DataTypes>::isTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle)
{
    const VecCoord* x0 = &f_X0.getValue();
    for (unsigned int i=0; i<3; ++i)
    {
        Coord p = (*x0)[triangle[i]];

        if((p-c).norm() > r)
            return false;
    }
    return true;
}

template <class DataTypes>
bool SphereROI<DataTypes>::isPartOfTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle)
{
    const VecCoord* x0 = &f_X0.getValue();
    for (unsigned int i=0; i<3; ++i)
    {
        Coord p = (*x0)[triangle[i]];

        if((p-c).norm() < r)
            return true;
    }
    return false;
}

template <class DataTypes>
bool SphereROI<DataTypes>::isQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad)
{
    const VecCoord* x0 = &f_X0.getValue();
    for (unsigned int i=0; i<4; ++i)
    {
        Coord p = (*x0)[quad[i]];

        if((p-c).norm() > r)
            return false;
    }
    return true;
}

template <class DataTypes>
bool SphereROI<DataTypes>::isPartOfQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad)
{
    const VecCoord* x0 = &f_X0.getValue();
    for (unsigned int i=0; i<4; ++i)
    {
        Coord p = (*x0)[quad[i]];

        if((p-c).norm() < r)
            return true;
    }
    return false;
}

template <class DataTypes>
bool SphereROI<DataTypes>::isTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron)
{
    const VecCoord* x0 = &f_X0.getValue();
    for (unsigned int i=0; i<4; ++i)
    {
        Coord p = (*x0)[tetrahedron[i]];

        if((p-c).norm() > r)
            return false;
    }
    return true;
}

template <class DataTypes>
bool SphereROI<DataTypes>::isPartOfTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron)
{
    const VecCoord* x0 = &f_X0.getValue();
    for (unsigned int i=0; i<4; ++i)
    {
        Coord p = (*x0)[tetrahedron[i]];

        if((p-c).norm() < r)
            return true;
    }
    return false;
}

template <class DataTypes>
void SphereROI<DataTypes>::update()
{
    cleanDirty();

    const helper::vector<Vec3>& cen = (centers.getValue());
    const helper::vector<Real>& rad = (radii.getValue());

    if (cen.empty())
        return;

    if (cen.size() != rad.size())
    {
        serr << "WARNING: number of sphere centers and radius doesn't match." <<sendl;
        return;
    }

    Real eAngle = edgeAngle.getValue();
    Real tAngle = triAngle.getValue();
    Coord dir = direction.getValue();
    Coord norm = normal.getValue();

    if (eAngle>0)
        dir.normalize();

    if (tAngle>0)
        norm.normalize();

    // Read accessor for input topology
    helper::ReadAccessor< Data<helper::vector<Edge> > > edges = f_edges;
    helper::ReadAccessor< Data<helper::vector<Triangle> > > triangles = f_triangles;
    helper::ReadAccessor< Data<helper::vector<Quad> > > quads = f_quads;
    helper::ReadAccessor< Data<helper::vector<Tetra> > > tetrahedra = f_tetrahedra;

    // Write accessor for topological element indices in SPHERE
    SetIndex& indices = *(f_indices.beginEdit());
    SetIndex& edgeIndices = *(f_edgeIndices.beginEdit());
    SetIndex& triangleIndices = *(f_triangleIndices.beginEdit());
    SetIndex& quadIndices = *(f_quadIndices.beginEdit());
    SetIndex& tetrahedronIndices = *f_tetrahedronIndices.beginEdit();

    // Write accessor for toplogical element in SPHERE
    helper::WriteAccessor< Data<VecCoord > > pointsInROI = f_pointsInROI;
    helper::WriteAccessor< Data<helper::vector<Edge> > > edgesInROI = f_edgesInROI;
    helper::WriteAccessor< Data<helper::vector<Triangle> > > trianglesInROI = f_trianglesInROI;
    helper::WriteAccessor< Data<helper::vector<Quad> > > quadsInROI = f_quadsInROI;
    helper::WriteAccessor< Data<helper::vector<Tetra> > > tetrahedraInROI = f_tetrahedraInROI;

    // Write accessor for topological element indices in SPHERE
    SetIndex& indicesOut = *(f_indicesOut.beginEdit());
    SetIndex& edgeIndicesOut = *(f_edgeIndicesOut.beginEdit());
    SetIndex& triangleIndicesOut = *(f_triangleIndicesOut.beginEdit());
    SetIndex& quadIndicesOut = *(f_quadIndicesOut.beginEdit());
    SetIndex& tetrahedronIndicesOut = *f_tetrahedronIndicesOut.beginEdit();

    // Write accessor for toplogical element in SPHERE
    helper::WriteAccessor< Data<VecCoord > > pointsOutROI = f_pointsOutROI;
    helper::WriteAccessor< Data<helper::vector<Edge> > > edgesOutROI = f_edgesOutROI;
    helper::WriteAccessor< Data<helper::vector<Triangle> > > trianglesOutROI = f_trianglesOutROI;
    helper::WriteAccessor< Data<helper::vector<Quad> > > quadsOutROI = f_quadsOutROI;
    helper::WriteAccessor< Data<helper::vector<Tetra> > > tetrahedraOutROI = f_tetrahedraOutROI;

    // Clear lists
    indices.clear();
    edgeIndices.clear();
    triangleIndices.clear();
    quadIndices.clear();
    tetrahedronIndices.clear();

    pointsInROI.clear();
    edgesInROI.clear();
    trianglesInROI.clear();
    quadsInROI.clear();
    tetrahedraInROI.clear();

    indicesOut.clear();
    edgeIndicesOut.clear();
    triangleIndicesOut.clear();
    quadIndicesOut.clear();
    tetrahedronIndicesOut.clear();

    pointsOutROI.clear();
    edgesOutROI.clear();
    trianglesOutROI.clear();
    quadsOutROI.clear();
    tetrahedraOutROI.clear();

    const VecCoord* x0 = &f_X0.getValue();

    bool strictlyInROI = f_strictlyInROI.getValue();

    if (strictlyInROI)
    {
        //Points
        for( unsigned i=0; i<x0->size(); ++i )
        {
            for (unsigned int j=0; j<cen.size(); ++j)
            {
                if (isPointInSphere(cen[j], rad[j], (*x0)[i]))
                {
                    indices.push_back(i);
                    pointsInROI.push_back((*x0)[i]);
                    break;
                }
                else
                {
                    indicesOut.push_back(i);
                    pointsOutROI.push_back((*x0)[i]);
                }
            }
        }

        //Edges
        if (f_computeEdges.getValue())
        {
            for(unsigned int i=0 ; i<edges.size() ; i++)
            {
                Edge edge = edges[i];
                for (unsigned int j=0; j<cen.size(); ++j)
                {
                    if (isEdgeInSphere(cen[j], rad[j], edge))
                    {
                        if (eAngle > 0)
                        {
                            Coord n = (*x0)[edge[1]]-(*x0)[edge[0]];
                            n.normalize();
                            if (fabs(dot(n,dir)) < fabs(cos(eAngle*M_PI/180.0))) continue;
                        }
                        edgeIndices.push_back(i);
                        edgesInROI.push_back(edge);
                        break;
                    }
                    else
                    {
                        edgeIndicesOut.push_back(i);
                        edgesOutROI.push_back(edge);
                    }
                }
            }
        }

        //Triangles
        if (f_computeTriangles.getValue())
        {
            for(unsigned int i=0 ; i<triangles.size() ; i++)
            {
                Triangle tri = triangles[i];
                for (unsigned int j=0; j<cen.size(); ++j)
                {
                    if (isTriangleInSphere(cen[j], rad[j], tri))
                    {
                        if (tAngle > 0)
                        {
                            Coord n = cross((*x0)[tri[2]]-(*x0)[tri[0]], (*x0)[tri[1]]-(*x0)[tri[0]]);
                            n.normalize();
                            if (dot(n,norm) < cos(tAngle*M_PI/180.0)) continue;
                        }
                        triangleIndices.push_back(i);
                        trianglesInROI.push_back(tri);
                        break;
                    }
                    else
                    {
                        triangleIndicesOut.push_back(i);
                        trianglesOutROI.push_back(tri);
                    }
                }
            }
        }

        //Quads
        if (f_computeQuads.getValue())
        {
            for(unsigned int i=0 ; i<quads.size() ; i++)
            {
                Quad qua = quads[i];
                for (unsigned int j=0; j<cen.size(); ++j)
                {
                    if (isQuadInSphere(cen[j], rad[j], qua))
                    {
                        quadIndices.push_back(i);
                        quadsInROI.push_back(qua);
                        break;
                    }
                    else
                    {
                        quadIndicesOut.push_back(i);
                        quadsOutROI.push_back(qua);
                    }
                }
            }
        }

        //Tetrahedra
        if (f_computeTetrahedra.getValue())
        {
            for(unsigned int i=0 ; i<tetrahedra.size() ; i++)
            {
                Tetra t = tetrahedra[i];
                for (unsigned int j=0; j<cen.size(); ++j)
                {
                    if (isTetrahedronInSphere(cen[j], rad[j], t))
                    {
                        //tAngle > 0 ??
                        tetrahedronIndices.push_back(i);
                        tetrahedraInROI.push_back(t);
                        break;
                    }
                    else
                    {
                        tetrahedronIndicesOut.push_back(i);
                        tetrahedraOutROI.push_back(t);
                    }
                }
            }
        }
    }
    else
    {
        //Points
        for( unsigned i=0; i<x0->size(); ++i )
        {
            for (unsigned int j=0; j<cen.size(); ++j)
            {
                if (isPointInSphere(cen[j], rad[j], (*x0)[i]))
                {
                    indices.push_back(i);
                    pointsInROI.push_back((*x0)[i]);
                    break;
                }
                else
                {
                    indicesOut.push_back(i);
                    pointsOutROI.push_back((*x0)[i]);
                }
            }
        }

        //Edges
        if (f_computeEdges.getValue())
        {
            for(unsigned int i=0 ; i<edges.size() ; i++)
            {
                Edge edge = edges[i];
                for (unsigned int j=0; j<cen.size(); ++j)
                {
                    if (isPartOfEdgeInSphere(cen[j], rad[j], edge))
                    {
                        if (eAngle > 0)
                        {
                            Coord n = (*x0)[edge[1]]-(*x0)[edge[0]];
                            n.normalize();
                            if (fabs(dot(n,dir)) < fabs(cos(eAngle*M_PI/180.0))) continue;
                        }
                        edgeIndices.push_back(i);
                        edgesInROI.push_back(edge);
                        break;
                    }
                    else
                    {
                        edgeIndicesOut.push_back(i);
                        edgesOutROI.push_back(edge);
                    }
                }
            }
        }

        //Triangles
        if (f_computeTriangles.getValue())
        {
            for(unsigned int i=0 ; i<triangles.size() ; i++)
            {
                Triangle tri = triangles[i];
                for (unsigned int j=0; j<cen.size(); ++j)
                {
                    if (isPartOfTriangleInSphere(cen[j], rad[j], tri))
                    {
                        if (tAngle > 0)
                        {
                            Coord n = cross((*x0)[tri[2]]-(*x0)[tri[0]], (*x0)[tri[1]]-(*x0)[tri[0]]);
                            n.normalize();
                            if (dot(n,norm) < cos(tAngle*M_PI/180.0)) continue;
                        }
                        triangleIndices.push_back(i);
                        trianglesInROI.push_back(tri);
                        break;
                    }
                    else
                    {
                        triangleIndicesOut.push_back(i);
                        trianglesOutROI.push_back(tri);
                    }
                }
            }
        }

        //Quads
        if (f_computeQuads.getValue())
        {
            for(unsigned int i=0 ; i<quads.size() ; i++)
            {
                Quad qua = quads[i];
                for (unsigned int j=0; j<cen.size(); ++j)
                {
                    if (isPartOfQuadInSphere(cen[j], rad[j], qua))
                    {
                        quadIndices.push_back(i);
                        quadsInROI.push_back(qua);
                        break;
                    }
                    else
                    {
                        quadIndicesOut.push_back(i);
                        quadsOutROI.push_back(qua);
                    }
                }
            }
        }

        //Tetrahedra
        if (f_computeTetrahedra.getValue())
        {
            for(unsigned int i=0 ; i<tetrahedra.size() ; i++)
            {
                Tetra t = tetrahedra[i];
                for (unsigned int j=0; j<cen.size(); ++j)
                {
                    if (isPartOfTetrahedronInSphere(cen[j], rad[j], t))
                    {
                        //tAngle > 0 ??
                        tetrahedronIndices.push_back(i);
                        tetrahedraInROI.push_back(t);
                        break;
                    }
                    else
                    {
                        tetrahedronIndicesOut.push_back(i);
                        tetrahedraOutROI.push_back(t);
                    }
                }
            }
        }
    }

    f_indices.endEdit();
    f_edgeIndices.endEdit();
    f_triangleIndices.endEdit();
    f_quadIndices.endEdit();
    f_tetrahedronIndices.endEdit();

    f_indicesOut.endEdit();
    f_edgeIndicesOut.endEdit();
    f_triangleIndicesOut.endEdit();
    f_quadIndicesOut.endEdit();
    f_tetrahedronIndicesOut.endEdit();
}

template <class DataTypes>
void SphereROI<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    const VecCoord* x0 = &f_X0.getValue();
    glColor3f(0.0, 1.0, 1.0);

    if(p_drawSphere.getValue()) // old classical drawing by points
    {
        ///draw the boxes
        const helper::vector<Vec3>& c=centers.getValue();
        const helper::vector<Real>& r=radii.getValue();

        for (unsigned int i=0; i<c.size() && i<r.size(); ++i)
        {
            helper::gl::drawWireSphere(c[i], (float)(r[i]/2.0));

            if (edgeAngle.getValue() > 0)
            {
                helper::gl::drawCone(c[i], c[i] + direction.getValue()*(cos(edgeAngle.getValue()*M_PI/180.0)*r[i]), 0, (float)sin(edgeAngle.getValue()*M_PI/180.0)*((float)r[i]));
            }

            if (triAngle.getValue() > 0)
            {
                helper::gl::drawCone(c[i], c[i] + normal.getValue()*(cos(triAngle.getValue()*M_PI/180.0)*r[i]), 0, (float)sin(triAngle.getValue()*M_PI/180.0)*((float)r[i]));
            }
        }
    }

    ///draw points in ROI
    if( p_drawPoints.getValue())
    {
        glDisable(GL_LIGHTING);
        glBegin(GL_POINTS);
        glPointSize(5.0);
        helper::ReadAccessor< Data<VecCoord > > pointsInROI = f_pointsInROI;
        for (unsigned int i=0; i<pointsInROI.size() ; ++i)
        {
            CPos p = DataTypes::getCPos(pointsInROI[i]);
            helper::gl::glVertexT(p);
        }
        glEnd();
    }

    ///draw edges in ROI
    if( p_drawEdges.getValue())
    {
        glDisable(GL_LIGHTING);
        glLineWidth((GLfloat)_drawSize.getValue());
        glBegin(GL_LINES);
        helper::ReadAccessor< Data<helper::vector<Edge> > > edgesInROI = f_edgesInROI;
        for (unsigned int i=0; i<edgesInROI.size() ; ++i)
        {
            Edge e = edgesInROI[i];
            for (unsigned int j=0 ; j<2 ; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[e[j]]);
                helper::gl::glVertexT(p);
            }
        }
        glEnd();
    }

    ///draw triangles in ROI
    if( p_drawTriangles.getValue())
    {
        glDisable(GL_LIGHTING);
        glLineWidth((GLfloat)_drawSize.getValue());
        glBegin(GL_TRIANGLES);
        helper::ReadAccessor< Data<helper::vector<Triangle> > > trianglesInROI = f_trianglesInROI;
        for (unsigned int i=0; i<trianglesInROI.size() ; ++i)
        {
            Triangle t = trianglesInROI[i];
            for (unsigned int j=0 ; j<3 ; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[t[j]]);
                helper::gl::glVertexT(p);
            }
        }
        glEnd();
    }

    ///draw quads in ROI
    if( p_drawTriangles.getValue())
    {
        glDisable(GL_LIGHTING);
        glLineWidth((GLfloat)_drawSize.getValue());
        glBegin(GL_QUADS);
        helper::ReadAccessor< Data<helper::vector<Quad> > > quadsInROI = f_quadsInROI;
        for (unsigned int i=0; i<quadsInROI.size() ; ++i)
        {
            Quad t = quadsInROI[i];
            for (unsigned int j=0 ; j<4 ; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[t[j]]);
                helper::gl::glVertexT(p);
            }
        }
        glEnd();
    }


    ///draw tetrahedra in ROI
    if( p_drawTetrahedra.getValue())
    {
        glDisable(GL_LIGHTING);
        glLineWidth((GLfloat)_drawSize.getValue());
        glBegin(GL_LINES);
        helper::ReadAccessor< Data<helper::vector<Tetra> > > tetrahedraInROI = f_tetrahedraInROI;
        for (unsigned int i=0; i<tetrahedraInROI.size() ; ++i)
        {
            Tetra t = tetrahedraInROI[i];
            for (unsigned int j=0 ; j<4 ; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[t[j]]);
                helper::gl::glVertexT(p);
                p = DataTypes::getCPos((*x0)[t[(j+1)%4]]);
                helper::gl::glVertexT(p);
            }

            CPos p = DataTypes::getCPos((*x0)[t[0]]);
            helper::gl::glVertexT(p);
            p = DataTypes::getCPos((*x0)[t[2]]);
            helper::gl::glVertexT(p);
            p = DataTypes::getCPos((*x0)[t[1]]);
            helper::gl::glVertexT(p);
            p = DataTypes::getCPos((*x0)[t[3]]);
            helper::gl::glVertexT(p);
        }
        glEnd();
    }
#endif /* SOFA_NO_OPENGL */
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////											////////////////////////
////////////////////	Rigid types specialization (double)		////////////////////////
////////////////////											////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

#ifndef SOFA_FLOAT

template <>
bool SphereROI<Rigid3dTypes>::isPointInSphere(const Vec3& c, const Real& r, const Coord& p)
{
	if((p.getCenter()-c).norm() > r)
		return false;
	else
		return true;
}

template <>
bool SphereROI<Rigid3dTypes>::isPointInSphere(const PointID& pid, const Real& r, const Coord& p)
{
	const VecCoord x0 = f_X0.getValue();
	CPos c =  x0[pid].getCenter();
	return ( isPointInSphere(c, r, p) );
}

template <>
bool SphereROI<Rigid3dTypes>::isEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<2; ++i)
	{
		Coord p = (*x0)[edge[i]];

		if((p.getCenter()-c).norm() > r)
			return false;
	}
	return true;
}

template <>
bool SphereROI<Rigid3dTypes>::isPartOfEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<2; ++i)
	{
		Coord p = (*x0)[edge[i]];

		if((p.getCenter()-c).norm() < r)
			return true;
	}
	return false;
}

template <>
bool SphereROI<Rigid3dTypes>::isTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<3; ++i)
	{
		Coord p = (*x0)[triangle[i]];

		if((p.getCenter()-c).norm() > r)
			return false;
	}
	return true;
}

template <>
bool SphereROI<Rigid3dTypes>::isPartOfTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<3; ++i)
	{
		Coord p = (*x0)[triangle[i]];

		if((p.getCenter()-c).norm() < r)
			return true;
	}
	return false;
}

template <>
bool SphereROI<Rigid3dTypes>::isQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<4; ++i)
	{
		Coord p = (*x0)[quad[i]];

		if((p.getCenter()-c).norm() > r)
			return false;
	}
	return true;
}

template <>
bool SphereROI<Rigid3dTypes>::isPartOfQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<4; ++i)
	{
		Coord p = (*x0)[quad[i]];

		if((p.getCenter()-c).norm() < r)
			return true;
	}
	return false;
}

template <>
bool SphereROI<Rigid3dTypes>::isTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<4; ++i)
	{
		Coord p = (*x0)[tetrahedron[i]];

		if((p.getCenter()-c).norm() > r)
			return false;
	}
	return true;
}

template <>
bool SphereROI<Rigid3dTypes>::isPartOfTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<4; ++i)
	{
		Coord p = (*x0)[tetrahedron[i]];

		if((p.getCenter()-c).norm() < r)
			return true;
	}
	return false;
}


template <>
void SphereROI<Rigid3dTypes>::update()
{
	cleanDirty();

	const helper::vector<Vec3>& cen = (centers.getValue());
	const helper::vector<Real>& rad = (radii.getValue());

	if (cen.empty())
		return;

	if (cen.size() != rad.size())
	{
		serr << "WARNING: number of sphere centers and radius doesn't match." <<sendl;
		return;
	}

	Real eAngle = edgeAngle.getValue();
	Real tAngle = triAngle.getValue();
	Vec3 dir = direction.getValue();
	Vec3 norm = normal.getValue();

	if (eAngle>0)
		dir.normalize();

	if (tAngle>0)
		norm.normalize();

	// Read accessor for input topology
	helper::ReadAccessor< Data<helper::vector<Edge> > > edges = f_edges;
	helper::ReadAccessor< Data<helper::vector<Triangle> > > triangles = f_triangles;
	helper::ReadAccessor< Data<helper::vector<Quad> > > quads = f_quads;
	helper::ReadAccessor< Data<helper::vector<Tetra> > > tetrahedra = f_tetrahedra;

    // Write accessor for topological element indices in SPHERE
    SetIndex& indices = *(f_indices.beginEdit());
    SetIndex& edgeIndices = *(f_edgeIndices.beginEdit());
    SetIndex& triangleIndices = *(f_triangleIndices.beginEdit());
    SetIndex& quadIndices = *(f_quadIndices.beginEdit());
    SetIndex& tetrahedronIndices = *f_tetrahedronIndices.beginEdit();

    // Write accessor for toplogical element in SPHERE
    helper::WriteAccessor< Data<VecCoord > > pointsInROI = f_pointsInROI;
    helper::WriteAccessor< Data<helper::vector<Edge> > > edgesInROI = f_edgesInROI;
    helper::WriteAccessor< Data<helper::vector<Triangle> > > trianglesInROI = f_trianglesInROI;
    helper::WriteAccessor< Data<helper::vector<Quad> > > quadsInROI = f_quadsInROI;
    helper::WriteAccessor< Data<helper::vector<Tetra> > > tetrahedraInROI = f_tetrahedraInROI;

    // Write accessor for topological element indices in SPHERE
    SetIndex& indicesOut = *(f_indicesOut.beginEdit());
    SetIndex& edgeIndicesOut = *(f_edgeIndicesOut.beginEdit());
    SetIndex& triangleIndicesOut = *(f_triangleIndicesOut.beginEdit());
    SetIndex& quadIndicesOut = *(f_quadIndicesOut.beginEdit());
    SetIndex& tetrahedronIndicesOut = *f_tetrahedronIndicesOut.beginEdit();

    // Write accessor for toplogical element in SPHERE
    helper::WriteAccessor< Data<VecCoord > > pointsOutROI = f_pointsOutROI;
    helper::WriteAccessor< Data<helper::vector<Edge> > > edgesOutROI = f_edgesOutROI;
    helper::WriteAccessor< Data<helper::vector<Triangle> > > trianglesOutROI = f_trianglesOutROI;
    helper::WriteAccessor< Data<helper::vector<Quad> > > quadsOutROI = f_quadsOutROI;
    helper::WriteAccessor< Data<helper::vector<Tetra> > > tetrahedraOutROI = f_tetrahedraOutROI;

    // Clear lists
    indices.clear();
    edgeIndices.clear();
    triangleIndices.clear();
    quadIndices.clear();
    tetrahedronIndices.clear();

    pointsInROI.clear();
    edgesInROI.clear();
    trianglesInROI.clear();
    quadsInROI.clear();
    tetrahedraInROI.clear();

    indicesOut.clear();
    edgeIndicesOut.clear();
    triangleIndicesOut.clear();
    quadIndicesOut.clear();
    tetrahedronIndicesOut.clear();

    pointsOutROI.clear();
    edgesOutROI.clear();
    trianglesOutROI.clear();
    quadsOutROI.clear();
    tetrahedraOutROI.clear();

	const VecCoord* x0 = &f_X0.getValue();

    bool strictlyInROI = f_strictlyInROI.getValue();

    if (strictlyInROI)
    {
	    //Points
	    for( unsigned i=0; i<x0->size(); ++i )
	    {
		    for (unsigned int j=0; j<cen.size(); ++j)
		    {
			    if (isPointInSphere(cen[j], rad[j], (*x0)[i]))
			    {
				    indices.push_back(i);
				    pointsInROI.push_back((*x0)[i]);
				    break;
			    }
			    else
                {
                    indicesOut.push_back(i);
                    pointsOutROI.push_back((*x0)[i]);
                }
		    }
	    }

	    //Edges
	    if (f_computeEdges.getValue())
	    {
		    for(unsigned int i=0 ; i<edges.size() ; i++)
		    {
			    Edge edge = edges[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isEdgeInSphere(cen[j], rad[j], edge))
				    {
					    if (eAngle > 0)
					    {
						    Vec3 n = (*x0)[edge[1]].getCenter()-(*x0)[edge[0]].getCenter();
						    n.normalize();
						    if (fabs(dot(n,dir)) < fabs(cos(eAngle*M_PI/180.0))) continue;
					    }
					    edgeIndices.push_back(i);
					    edgesInROI.push_back(edge);
					    break;
				    }
                    else
                    {
                        edgeIndicesOut.push_back(i);
                        edgesOutROI.push_back(edge);
                    }
			    }
		    }
	    }

	    //Triangles
	    if (f_computeTriangles.getValue())
	    {
		    for(unsigned int i=0 ; i<triangles.size() ; i++)
		    {
			    Triangle tri = triangles[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isTriangleInSphere(cen[j], rad[j], tri))
				    {
					    if (tAngle > 0)
					    {
						    Vec3 n = cross((*x0)[tri[2]].getCenter()-(*x0)[tri[0]].getCenter(), (*x0)[tri[1]].getCenter()-(*x0)[tri[0]].getCenter());
						    n.normalize();
						    if (dot(n,norm) < cos(tAngle*M_PI/180.0)) continue;
					    }
					    triangleIndices.push_back(i);
					    trianglesInROI.push_back(tri);
					    break;
				    }
                    else
                    {
                        triangleIndicesOut.push_back(i);
                        trianglesOutROI.push_back(tri);
                    }
			    }
		    }
	    }

	    //Quads
	    if (f_computeQuads.getValue())
	    {
		    for(unsigned int i=0 ; i<quads.size() ; i++)
		    {
			    Quad qua = quads[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isQuadInSphere(cen[j], rad[j], qua))
				    {
					    quadIndices.push_back(i);
					    quadsInROI.push_back(qua);
					    break;
				    }
                    else
                    {
                        quadIndicesOut.push_back(i);
                        quadsOutROI.push_back(qua);
                    }
			    }
		    }
	    }

	    //Tetrahedra
	    if (f_computeTetrahedra.getValue())
	    {
		    for(unsigned int i=0 ; i<tetrahedra.size() ; i++)
		    {
			    Tetra t = tetrahedra[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isTetrahedronInSphere(cen[j], rad[j], t))
				    {
					    //tAngle > 0 ??
					    tetrahedronIndices.push_back(i);
					    tetrahedraInROI.push_back(t);
					    break;
				    }
                    else
                    {
                        tetrahedronIndicesOut.push_back(i);
                        tetrahedraOutROI.push_back(t);
                    }
			    }
		    }
	    }
    }
    else
    {
	    //Points
	    for( unsigned i=0; i<x0->size(); ++i )
	    {
		    for (unsigned int j=0; j<cen.size(); ++j)
		    {
			    if (isPointInSphere(cen[j], rad[j], (*x0)[i]))
			    {
				    indices.push_back(i);
				    pointsInROI.push_back((*x0)[i]);
				    break;
			    }
			    else
                {
                    indicesOut.push_back(i);
                    pointsOutROI.push_back((*x0)[i]);
                }
		    }
	    }

	    //Edges
	    if (f_computeEdges.getValue())
	    {
		    for(unsigned int i=0 ; i<edges.size() ; i++)
		    {
			    Edge edge = edges[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isPartOfEdgeInSphere(cen[j], rad[j], edge))
				    {
					    if (eAngle > 0)
					    {
						    Vec3 n = (*x0)[edge[1]].getCenter()-(*x0)[edge[0]].getCenter();
						    n.normalize();
						    if (fabs(dot(n,dir)) < fabs(cos(eAngle*M_PI/180.0))) continue;
					    }
					    edgeIndices.push_back(i);
					    edgesInROI.push_back(edge);
					    break;
				    }
                    else
                    {
                        edgeIndicesOut.push_back(i);
                        edgesOutROI.push_back(edge);
                    }
			    }
		    }
	    }

	    //Triangles
	    if (f_computeTriangles.getValue())
	    {
		    for(unsigned int i=0 ; i<triangles.size() ; i++)
		    {
			    Triangle tri = triangles[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isPartOfTriangleInSphere(cen[j], rad[j], tri))
				    {
					    if (tAngle > 0)
					    {
						    Vec3 n = cross((*x0)[tri[2]].getCenter()-(*x0)[tri[0]].getCenter(), (*x0)[tri[1]].getCenter()-(*x0)[tri[0]].getCenter());
						    n.normalize();
						    if (dot(n,norm) < cos(tAngle*M_PI/180.0)) continue;
					    }
					    triangleIndices.push_back(i);
					    trianglesInROI.push_back(tri);
					    break;
				    }
                    else
                    {
                        triangleIndicesOut.push_back(i);
                        trianglesOutROI.push_back(tri);
                    }
			    }
		    }
	    }

	    //Quads
	    if (f_computeQuads.getValue())
	    {
		    for(unsigned int i=0 ; i<quads.size() ; i++)
		    {
			    Quad qua = quads[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isPartOfQuadInSphere(cen[j], rad[j], qua))
				    {
					    quadIndices.push_back(i);
					    quadsInROI.push_back(qua);
					    break;
				    }
                    else
                    {
                        quadIndicesOut.push_back(i);
                        quadsOutROI.push_back(qua);
                    }
			    }
		    }
	    }

	    //Tetrahedra
	    if (f_computeTetrahedra.getValue())
	    {
		    for(unsigned int i=0 ; i<tetrahedra.size() ; i++)
		    {
			    Tetra t = tetrahedra[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isPartOfTetrahedronInSphere(cen[j], rad[j], t))
				    {
					    //tAngle > 0 ??
					    tetrahedronIndices.push_back(i);
					    tetrahedraInROI.push_back(t);
					    break;
				    }
                    else
                    {
                        tetrahedronIndicesOut.push_back(i);
                        tetrahedraOutROI.push_back(t);
                    }
			    }
		    }
	    }
    }

    f_indices.endEdit();
    f_edgeIndices.endEdit();
    f_triangleIndices.endEdit();
    f_quadIndices.endEdit();
    f_tetrahedronIndices.endEdit();

    f_indicesOut.endEdit();
    f_edgeIndicesOut.endEdit();
    f_triangleIndicesOut.endEdit();
    f_quadIndicesOut.endEdit();
    f_tetrahedronIndicesOut.endEdit();
}
#endif //#ifndef SOFA_FLOAT


////////////////////////////////////////////////////////////////////////////////////////
////////////////////											////////////////////////
////////////////////	Rigid types specialization (float)		////////////////////////
////////////////////											////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

#ifndef SOFA_DOUBLE

template <>
bool SphereROI<Rigid3fTypes>::isPointInSphere(const Vec3& c, const Real& r, const Coord& p)
{
	if((p.getCenter()-c).norm() > r)
		return false;
	else
		return true;
}

template <>
bool SphereROI<Rigid3fTypes>::isPointInSphere(const PointID& pid, const Real& r, const Coord& p)
{
	const VecCoord x0 = f_X0.getValue();
	CPos c =  x0[pid].getCenter();
	return ( isPointInSphere(c, r, p) );
}

template <>
bool SphereROI<Rigid3fTypes>::isEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<2; ++i)
	{
		Coord p = (*x0)[edge[i]];

		if((p.getCenter()-c).norm() > r)
			return false;
	}
	return true;
}

template <>
bool SphereROI<Rigid3fTypes>::isPartOfEdgeInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Edge& edge)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<2; ++i)
	{
		Coord p = (*x0)[edge[i]];

		if((p.getCenter()-c).norm() < r)
			return true;
	}
	return false;
}

template <>
bool SphereROI<Rigid3fTypes>::isTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<3; ++i)
	{
		Coord p = (*x0)[triangle[i]];

		if((p.getCenter()-c).norm() > r)
			return false;
	}
	return true;
}

template <>
bool SphereROI<Rigid3fTypes>::isPartOfTriangleInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Triangle& triangle)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<3; ++i)
	{
		Coord p = (*x0)[triangle[i]];

		if((p.getCenter()-c).norm() < r)
			return true;
	}
	return false;
}

template <>
bool SphereROI<Rigid3fTypes>::isQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<4; ++i)
	{
		Coord p = (*x0)[quad[i]];

		if((p.getCenter()-c).norm() > r)
			return false;
	}
	return true;
}

template <>
bool SphereROI<Rigid3fTypes>::isPartOfQuadInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Quad& quad)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<4; ++i)
	{
		Coord p = (*x0)[quad[i]];

		if((p.getCenter()-c).norm() < r)
			return true;
	}
	return false;
}

template <>
bool SphereROI<Rigid3fTypes>::isTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<4; ++i)
	{
		Coord p = (*x0)[tetrahedron[i]];

		if((p.getCenter()-c).norm() > r)
			return false;
	}
	return true;
}

template <>
bool SphereROI<Rigid3fTypes>::isPartOfTetrahedronInSphere(const Vec3& c, const Real& r, const sofa::core::topology::BaseMeshTopology::Tetra& tetrahedron)
{
	const VecCoord* x0 = &f_X0.getValue();
	for (unsigned int i=0; i<4; ++i)
	{
		Coord p = (*x0)[tetrahedron[i]];

		if((p.getCenter()-c).norm() < r)
			return true;
	}
	return false;
}

template <>
void SphereROI<Rigid3fTypes>::update()
{
	cleanDirty();

	const helper::vector<Vec3>& cen = (centers.getValue());
	const helper::vector<Real>& rad = (radii.getValue());

	if (cen.empty())
		return;

	if (cen.size() != rad.size())
	{
		serr << "WARNING: number of sphere centers and radius doesn't match." <<sendl;
		return;
	}

	Real eAngle = edgeAngle.getValue();
	Real tAngle = triAngle.getValue();
	Vec3 dir = direction.getValue();
	Vec3 norm = normal.getValue();

	if (eAngle>0)
		dir.normalize();

	if (tAngle>0)
		norm.normalize();

	// Read accessor for input topology
	helper::ReadAccessor< Data<helper::vector<Edge> > > edges = f_edges;
	helper::ReadAccessor< Data<helper::vector<Triangle> > > triangles = f_triangles;
	helper::ReadAccessor< Data<helper::vector<Quad> > > quads = f_quads;
	helper::ReadAccessor< Data<helper::vector<Tetra> > > tetrahedra = f_tetrahedra;

    // Write accessor for topological element indices in SPHERE
    SetIndex& indices = *(f_indices.beginEdit());
    SetIndex& edgeIndices = *(f_edgeIndices.beginEdit());
    SetIndex& triangleIndices = *(f_triangleIndices.beginEdit());
    SetIndex& quadIndices = *(f_quadIndices.beginEdit());
    SetIndex& tetrahedronIndices = *f_tetrahedronIndices.beginEdit();

    // Write accessor for toplogical element in SPHERE
    helper::WriteAccessor< Data<VecCoord > > pointsInROI = f_pointsInROI;
    helper::WriteAccessor< Data<helper::vector<Edge> > > edgesInROI = f_edgesInROI;
    helper::WriteAccessor< Data<helper::vector<Triangle> > > trianglesInROI = f_trianglesInROI;
    helper::WriteAccessor< Data<helper::vector<Quad> > > quadsInROI = f_quadsInROI;
    helper::WriteAccessor< Data<helper::vector<Tetra> > > tetrahedraInROI = f_tetrahedraInROI;

    // Write accessor for topological element indices in SPHERE
    SetIndex& indicesOut = *(f_indicesOut.beginEdit());
    SetIndex& edgeIndicesOut = *(f_edgeIndicesOut.beginEdit());
    SetIndex& triangleIndicesOut = *(f_triangleIndicesOut.beginEdit());
    SetIndex& quadIndicesOut = *(f_quadIndicesOut.beginEdit());
    SetIndex& tetrahedronIndicesOut = *f_tetrahedronIndicesOut.beginEdit();

    // Write accessor for toplogical element in SPHERE
    helper::WriteAccessor< Data<VecCoord > > pointsOutROI = f_pointsOutROI;
    helper::WriteAccessor< Data<helper::vector<Edge> > > edgesOutROI = f_edgesOutROI;
    helper::WriteAccessor< Data<helper::vector<Triangle> > > trianglesOutROI = f_trianglesOutROI;
    helper::WriteAccessor< Data<helper::vector<Quad> > > quadsOutROI = f_quadsOutROI;
    helper::WriteAccessor< Data<helper::vector<Tetra> > > tetrahedraOutROI = f_tetrahedraOutROI;

    // Clear lists
    indices.clear();
    edgeIndices.clear();
    triangleIndices.clear();
    quadIndices.clear();
    tetrahedronIndices.clear();

    pointsInROI.clear();
    edgesInROI.clear();
    trianglesInROI.clear();
    quadsInROI.clear();
    tetrahedraInROI.clear();

    indicesOut.clear();
    edgeIndicesOut.clear();
    triangleIndicesOut.clear();
    quadIndicesOut.clear();
    tetrahedronIndicesOut.clear();

    pointsOutROI.clear();
    edgesOutROI.clear();
    trianglesOutROI.clear();
    quadsOutROI.clear();
    tetrahedraOutROI.clear();

	const VecCoord* x0 = &f_X0.getValue();

    bool strictlyInROI = f_strictlyInROI.getValue();

    if (strictlyInROI)
    {
	    //Points
	    for( unsigned i=0; i<x0->size(); ++i )
	    {
		    for (unsigned int j=0; j<cen.size(); ++j)
		    {
			    if (isPointInSphere(cen[j], rad[j], (*x0)[i]))
			    {
				    indices.push_back(i);
				    pointsInROI.push_back((*x0)[i]);
				    break;
			    }
			    else
                {
                    indicesOut.push_back(i);
                    pointsOutROI.push_back((*x0)[i]);
                }
		    }
	    }

	    //Edges
	    if (f_computeEdges.getValue())
	    {
		    for(unsigned int i=0 ; i<edges.size() ; i++)
		    {
			    Edge edge = edges[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isEdgeInSphere(cen[j], rad[j], edge))
				    {
					    if (eAngle > 0)
					    {
						    Vec3 n = (*x0)[edge[1]].getCenter()-(*x0)[edge[0]].getCenter();
						    n.normalize();
						    if (fabs(dot(n,dir)) < fabs(cos(eAngle*M_PI/180.0))) continue;
					    }
					    edgeIndices.push_back(i);
					    edgesInROI.push_back(edge);
					    break;
				    }
                    else
                    {
                        edgeIndicesOut.push_back(i);
                        edgesOutROI.push_back(edge);
                    }
			    }
		    }
	    }

	    //Triangles
	    if (f_computeTriangles.getValue())
	    {
		    for(unsigned int i=0 ; i<triangles.size() ; i++)
		    {
			    Triangle tri = triangles[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isTriangleInSphere(cen[j], rad[j], tri))
				    {
					    if (tAngle > 0)
					    {
						    Vec3 n = cross((*x0)[tri[2]].getCenter()-(*x0)[tri[0]].getCenter(), (*x0)[tri[1]].getCenter()-(*x0)[tri[0]].getCenter());
						    n.normalize();
						    if (dot(n,norm) < cos(tAngle*M_PI/180.0)) continue;
					    }
					    triangleIndices.push_back(i);
					    trianglesInROI.push_back(tri);
					    break;
				    }
                    else
                    {
                        triangleIndicesOut.push_back(i);
                        trianglesOutROI.push_back(tri);
                    }
			    }
		    }
	    }

	    //Quads
	    if (f_computeQuads.getValue())
	    {
		    for(unsigned int i=0 ; i<quads.size() ; i++)
		    {
			    Quad qua = quads[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isQuadInSphere(cen[j], rad[j], qua))
				    {
					    quadIndices.push_back(i);
					    quadsInROI.push_back(qua);
					    break;
				    }
                    else
                    {
                        quadIndicesOut.push_back(i);
                        quadsOutROI.push_back(qua);
                    }
			    }
		    }
	    }

	    //Tetrahedra
	    if (f_computeTetrahedra.getValue())
	    {
		    for(unsigned int i=0 ; i<tetrahedra.size() ; i++)
		    {
			    Tetra t = tetrahedra[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isTetrahedronInSphere(cen[j], rad[j], t))
				    {
					    //tAngle > 0 ??
					    tetrahedronIndices.push_back(i);
					    tetrahedraInROI.push_back(t);
					    break;
				    }
                    else
                    {
                        tetrahedronIndicesOut.push_back(i);
                        tetrahedraOutROI.push_back(t);
                    }
			    }
		    }
	    }
    }
    else
    {
	    //Points
	    for( unsigned i=0; i<x0->size(); ++i )
	    {
		    for (unsigned int j=0; j<cen.size(); ++j)
		    {
			    if (isPointInSphere(cen[j], rad[j], (*x0)[i]))
			    {
				    indices.push_back(i);
				    pointsInROI.push_back((*x0)[i]);
				    break;
			    }
			    else
                {
                    indicesOut.push_back(i);
                    pointsOutROI.push_back((*x0)[i]);
                }
		    }
	    }

	    //Edges
	    if (f_computeEdges.getValue())
	    {
		    for(unsigned int i=0 ; i<edges.size() ; i++)
		    {
			    Edge edge = edges[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isPartOfEdgeInSphere(cen[j], rad[j], edge))
				    {
					    if (eAngle > 0)
					    {
						    Vec3 n = (*x0)[edge[1]].getCenter()-(*x0)[edge[0]].getCenter();
						    n.normalize();
						    if (fabs(dot(n,dir)) < fabs(cos(eAngle*M_PI/180.0))) continue;
					    }
					    edgeIndices.push_back(i);
					    edgesInROI.push_back(edge);
					    break;
				    }
                    else
                    {
                        edgeIndicesOut.push_back(i);
                        edgesOutROI.push_back(edge);
                    }
			    }
		    }
	    }

	    //Triangles
	    if (f_computeTriangles.getValue())
	    {
		    for(unsigned int i=0 ; i<triangles.size() ; i++)
		    {
			    Triangle tri = triangles[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isPartOfTriangleInSphere(cen[j], rad[j], tri))
				    {
					    if (tAngle > 0)
					    {
						    Vec3 n = cross((*x0)[tri[2]].getCenter()-(*x0)[tri[0]].getCenter(), (*x0)[tri[1]].getCenter()-(*x0)[tri[0]].getCenter());
						    n.normalize();
						    if (dot(n,norm) < cos(tAngle*M_PI/180.0)) continue;
					    }
					    triangleIndices.push_back(i);
					    trianglesInROI.push_back(tri);
					    break;
				    }
                    else
                    {
                        triangleIndicesOut.push_back(i);
                        trianglesOutROI.push_back(tri);
                    }
			    }
		    }
	    }

	    //Quads
	    if (f_computeQuads.getValue())
	    {
		    for(unsigned int i=0 ; i<quads.size() ; i++)
		    {
			    Quad qua = quads[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isPartOfQuadInSphere(cen[j], rad[j], qua))
				    {
					    quadIndices.push_back(i);
					    quadsInROI.push_back(qua);
					    break;
				    }
                    else
                    {
                        quadIndicesOut.push_back(i);
                        quadsOutROI.push_back(qua);
                    }
			    }
		    }
	    }

	    //Tetrahedra
	    if (f_computeTetrahedra.getValue())
	    {
		    for(unsigned int i=0 ; i<tetrahedra.size() ; i++)
		    {
			    Tetra t = tetrahedra[i];
			    for (unsigned int j=0; j<cen.size(); ++j)
			    {
				    if (isPartOfTetrahedronInSphere(cen[j], rad[j], t))
				    {
					    //tAngle > 0 ??
					    tetrahedronIndices.push_back(i);
					    tetrahedraInROI.push_back(t);
					    break;
				    }
                    else
                    {
                        tetrahedronIndicesOut.push_back(i);
                        tetrahedraOutROI.push_back(t);
                    }
			    }
		    }
	    }
    }

    f_indices.endEdit();
    f_edgeIndices.endEdit();
    f_triangleIndices.endEdit();
    f_quadIndices.endEdit();
    f_tetrahedronIndices.endEdit();

    f_indicesOut.endEdit();
    f_edgeIndicesOut.endEdit();
    f_triangleIndicesOut.endEdit();
    f_quadIndicesOut.endEdit();
    f_tetrahedronIndicesOut.endEdit();
}

#endif //#ifndef SOFA_DOUBLE


} // namespace engine

} // namespace component

} // namespace sofa

#endif
