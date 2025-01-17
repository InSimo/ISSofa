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
#ifndef SOFA_COMPONENT_ENGINE_BASEBOXROI_CPP
#define SOFA_COMPONENT_ENGINE_BASEBOXROI_CPP

#include <SofaEngine/BaseBoxROI.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/gl/BasicShapes.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/BoundingBox.h>
#include <limits>


namespace sofa
{

namespace component
{

namespace engine
{

template <class DataTypes, class BoxType>
BaseBoxROI<DataTypes, BoxType>::BaseBoxROI()
    : boxes(initData (&boxes, "box", "") )
    , f_computeEdges( initData(&f_computeEdges, true,"computeEdges","If true, will compute edge list and index list inside the ROI.") )
    , f_computeTriangles( initData(&f_computeTriangles, true,"computeTriangles","If true, will compute triangle list and index list inside the ROI.") )
    , f_computeTetrahedra( initData(&f_computeTetrahedra, true,"computeTetrahedra","If true, will compute tetrahedra list and index list inside the ROI.") )
    , f_computeHexahedra( initData(&f_computeHexahedra, true,"computeHexahedra","If true, will compute hexahedra list and index list inside the ROI.") )
    , f_computeQuad( initData(&f_computeQuad, true,"computeQuad","If true, will compute quad list and index list inside the ROI.") )
    , f_X0(initData (&f_X0, "position", "Rest position coordinates of the degrees of freedom") )
    , f_edges(initData (&f_edges, "edges", "Edge Topology") )
    , f_triangles(initData (&f_triangles, "triangles", "Triangle Topology") )
    , f_tetrahedra(initData (&f_tetrahedra, "tetrahedra", "Tetrahedron Topology") )
    , f_hexahedra(initData (&f_hexahedra, "hexahedra", "Hexahedron Topology") )
    , f_quad(initData (&f_quad, "quad", "Quad Topology") )
    , f_indices( initData(&f_indices,"indices","Indices of the points contained in the ROI") )
    , f_edgeIndices( initData(&f_edgeIndices,"edgeIndices","Indices of the edges contained in the ROI") )
    , f_triangleIndices( initData(&f_triangleIndices,"triangleIndices","Indices of the triangles contained in the ROI") )
    , f_tetrahedronIndices( initData(&f_tetrahedronIndices,"tetrahedronIndices","Indices of the tetrahedra contained in the ROI") )
    , f_hexahedronIndices( initData(&f_hexahedronIndices,"hexahedronIndices","Indices of the hexahedra contained in the ROI") )
    , f_quadIndices( initData(&f_quadIndices,"quadIndices","Indices of the quad contained in the ROI") )
    , f_pointsInROI( initData(&f_pointsInROI,"pointsInROI","Points contained in the ROI") )
    , f_edgesInROI( initData(&f_edgesInROI,"edgesInROI","Edges contained in the ROI") )
    , f_trianglesInROI( initData(&f_trianglesInROI,"trianglesInROI","Triangles contained in the ROI") )
    , f_tetrahedraInROI( initData(&f_tetrahedraInROI,"tetrahedraInROI","Tetrahedra contained in the ROI") )
    , f_hexahedraInROI( initData(&f_hexahedraInROI,"hexahedraInROI","Hexahedra contained in the ROI") )
    , f_quadInROI( initData(&f_quadInROI,"quadInROI","Quad contained in the ROI") )
    , f_nbIndices( initData(&f_nbIndices,"nbIndices", "Number of selected indices") )
    , f_pointsOutROI( initData(&f_pointsOutROI,"pointsOutROI","Points contained out of the ROI") )
    , f_edgesOutROI( initData(&f_edgesOutROI,"edgesOutROI","Edges contained out of the ROI") )
    , f_trianglesOutROI( initData(&f_trianglesOutROI,"trianglesOutROI","Triangles contained out of the ROI") )
    , f_tetrahedraOutROI( initData(&f_tetrahedraOutROI,"tetrahedraOutROI","Tetrahedra contained out of the ROI") )
    , f_hexahedraOutROI( initData(&f_hexahedraOutROI,"hexahedraOutROI","Hexahedra contained out of the ROI") )
    , f_quadOutROI( initData(&f_quadOutROI,"quadOutROI","Quad contained out of the ROI") )
    , p_drawBoxes( initData(&p_drawBoxes,false,"drawBoxes","Draw Box(es)") )
    , p_drawPoints( initData(&p_drawPoints,false,"drawPoints","Draw Points") )
    , p_drawEdges( initData(&p_drawEdges,false,"drawEdges","Draw Edges") )
    , p_drawTriangles( initData(&p_drawTriangles,false,"drawTriangles","Draw Triangles") )
    , p_drawTetrahedra( initData(&p_drawTetrahedra,false,"drawTetrahedra","Draw Tetrahedra") )
    , p_drawHexahedra( initData(&p_drawHexahedra,false,"drawHexahedra","Draw Tetrahedra") )
    , p_drawQuads( initData(&p_drawQuads,false,"drawQuads","Draw Quads") )
    , p_drawSize( initData(&p_drawSize,0.0,"drawSize","rendering size for box and topological elements") )
{
    f_computeEdges.setGroup("Compute modes");
    f_computeTriangles.setGroup("Compute modes");
    f_computeTetrahedra.setGroup("Compute modes");
    f_computeHexahedra.setGroup("Compute modes");
    f_computeQuad.setGroup("Compute modes");

    //Adding alias to handle old BoxROI input/output
    addAlias(&f_pointsInROI,"pointsInBox");
    addAlias(&f_edgesInROI,"edgesInBox");
    addAlias(&f_trianglesInROI,"f_trianglesInBox");
    addAlias(&f_tetrahedraInROI,"f_tetrahedraInBox");
    addAlias(&f_hexahedraInROI,"f_tetrahedraInBox");
    addAlias(&f_quadInROI,"f_quadInBOX");
    addAlias(&f_X0,"rest_position");

    //Adding alias to handle TrianglesInBoxROI input/output
    addAlias(&p_drawBoxes,"isVisible");

    f_indices.beginEdit()->push_back(0);
    f_indices.endEdit();
}


template <class DataTypes, class BoxType>
void BaseBoxROI<DataTypes, BoxType>::init()
{
    //cerr<<"BoxROI<DataTypes>::init() is called "<<endl;
    if (!f_X0.isSet())
    {
        //cerr<<"BoxROI<DataTypes>::init() f_X0 is not set "<<endl;
        sofa::core::behavior::BaseMechanicalState* mstate = NULL;
        this->getContext()->get(mstate,sofa::core::objectmodel::BaseContext::Local);
        if (mstate)
        {
            sofa::core::objectmodel::BaseData* parent = mstate->findData("rest_position");
            if (parent)
            {
                f_X0.setParent(parent);
                f_X0.setReadOnly(true);
            }
        }
        else
        {
            sofa::core::loader::MeshLoader* loader = NULL;
            this->getContext()->get(loader,sofa::core::objectmodel::BaseContext::Local);
            if (loader)
            {
                sofa::core::objectmodel::BaseData* parent = loader->findData("position");
                if (parent)
                {
                    f_X0.setParent(parent);
                    f_X0.setReadOnly(true);
                }
            }
            else   // no local state, no loader => find upward
            {
                this->getContext()->get(mstate,sofa::core::objectmodel::BaseContext::SearchUp);
                assert(mstate && "BoxROI needs a mstate");
                sofa::core::objectmodel::BaseData* parent = mstate->findData("rest_position");
                assert(parent && "BoxROI needs a state with a rest_position Data");
                f_X0.setParent(parent);
                f_X0.setReadOnly(true);
            }
        }
    }
    if (!f_edges.isSet() || !f_triangles.isSet() || !f_tetrahedra.isSet() || !f_hexahedra.isSet() || !f_quad.isSet() )
    {
        sofa::core::topology::BaseMeshTopology* topology;
        this->getContext()->get(topology,sofa::core::objectmodel::BaseContext::Local);
        if (topology)
        {
            if (!f_edges.isSet() && f_computeEdges.getValue())
            {
                sofa::core::objectmodel::BaseData* eparent = topology->findData("edges");
                if (eparent)
                {
                    f_edges.setParent(eparent);
                    f_edges.setReadOnly(true);
                }
            }
            if (!f_triangles.isSet() && f_computeTriangles.getValue())
            {
                sofa::core::objectmodel::BaseData* tparent = topology->findData("triangles");
                if (tparent)
                {
                    f_triangles.setParent(tparent);
                    f_triangles.setReadOnly(true);
                }
            }
            if (!f_tetrahedra.isSet() && f_computeTetrahedra.getValue())
            {
                sofa::core::objectmodel::BaseData* tparent = topology->findData("tetrahedra");
                if (tparent)
                {
                    f_tetrahedra.setParent(tparent);
                    f_tetrahedra.setReadOnly(true);
                }
            }
            if (!f_hexahedra.isSet() && f_computeHexahedra.getValue())
            {
                sofa::core::objectmodel::BaseData* tparent = topology->findData("hexahedra");
                if (tparent)
                {
                    f_hexahedra.setParent(tparent);
                    f_hexahedra.setReadOnly(true);
                }
            }
            if (!f_quad.isSet() && f_computeQuad.getValue())
            {
                sofa::core::objectmodel::BaseData* tparent = topology->findData("quads");
                if (tparent)
                {
                    f_quad.setParent(tparent);
                    f_quad.setReadOnly(true);
                }
            }

        }
    }

    addInput(&f_X0);
    addInput(&f_edges);
    addInput(&f_triangles);
    addInput(&f_tetrahedra);
    addInput(&f_hexahedra);
    addInput(&f_quad);

    addOutput(&f_indices);
    addOutput(&f_edgeIndices);
    addOutput(&f_triangleIndices);
    addOutput(&f_tetrahedronIndices);
    addOutput(&f_hexahedronIndices);
    addOutput(&f_quadIndices);
    addOutput(&f_pointsInROI);
    addOutput(&f_edgesInROI);
    addOutput(&f_trianglesInROI);
    addOutput(&f_tetrahedraInROI);
    addOutput(&f_hexahedraInROI);
    addOutput(&f_quadInROI);
    addOutput(&f_nbIndices);
    addOutput(&f_pointsOutROI);
    addOutput(&f_edgesOutROI);
    addOutput(&f_trianglesOutROI);
    addOutput(&f_tetrahedraOutROI);
    addOutput(&f_hexahedraOutROI);
    addOutput(&f_quadOutROI);
    setDirtyValue();
}

template <class DataTypes, class BoxType>
void BaseBoxROI<DataTypes, BoxType>::reinit()
{
    Inherit1::reinit();
    setDirtyValue();
    this->requestUpdate();
}

template <class DataTypes, class BoxType>
void BaseBoxROI<DataTypes, BoxType>::update()
{
    cleanDirty();

    this->updateBoxes();

    helper::ReadAccessor< Data<Boxes> > vb = boxes;

    // Read accessor for input topology
    helper::ReadAccessor< Data<helper::vector<Edge> > > edges = f_edges;
    helper::ReadAccessor< Data<helper::vector<Triangle> > > triangles = f_triangles;
    helper::ReadAccessor< Data<helper::vector<Tetra> > > tetrahedra = f_tetrahedra;
    helper::ReadAccessor< Data<helper::vector<Hexa> > > hexahedra = f_hexahedra;
    helper::ReadAccessor< Data<helper::vector<Quad> > > quad = f_quad;

    // Write accessor for topological element indices in BOX
    SetIndex& indices = *f_indices.beginEdit();
    SetIndex& edgeIndices = *f_edgeIndices.beginEdit();
    SetIndex& triangleIndices = *f_triangleIndices.beginEdit();
    SetIndex& tetrahedronIndices = *f_tetrahedronIndices.beginEdit();
    SetIndex& hexahedronIndices = *f_hexahedronIndices.beginEdit();
    SetIndex& quadIndices = *f_quadIndices.beginEdit();

    // Write accessor for toplogical element in BOX
    helper::WriteOnlyAccessor< Data<VecCoord > > pointsInROI = f_pointsInROI;
    helper::WriteOnlyAccessor< Data<helper::vector<Edge> > > edgesInROI = f_edgesInROI;
    helper::WriteOnlyAccessor< Data<helper::vector<Triangle> > > trianglesInROI = f_trianglesInROI;
    helper::WriteOnlyAccessor< Data<helper::vector<Tetra> > > tetrahedraInROI = f_tetrahedraInROI;
    helper::WriteOnlyAccessor< Data<helper::vector<Hexa> > > hexahedraInROI = f_hexahedraInROI;
    helper::WriteOnlyAccessor< Data<helper::vector<Quad> > > quadInROI = f_quadInROI;

    // Write accessor for toplogical element out of the BOX
    helper::WriteAccessor< Data<VecCoord > > pointsOutROI = f_pointsOutROI;
    helper::WriteAccessor< Data<helper::vector<Edge> > > edgesOutROI = f_edgesOutROI;
    helper::WriteAccessor< Data<helper::vector<Triangle> > > trianglesOutROI = f_trianglesOutROI;
    helper::WriteAccessor< Data<helper::vector<Tetra> > > tetrahedraOutROI = f_tetrahedraOutROI;
    helper::WriteAccessor< Data<helper::vector<Hexa> > > hexahedraOutROI = f_hexahedraOutROI;
    helper::WriteAccessor< Data<helper::vector<Quad> > > quadOutROI = f_quadOutROI;


    // Clear lists
    indices.clear();
    edgeIndices.clear();
    triangleIndices.clear();
    tetrahedronIndices.clear();
    hexahedronIndices.clear();
    quadIndices.clear();

    pointsInROI.clear();
    edgesInROI.clear();
    trianglesInROI.clear();
    tetrahedraInROI.clear();
    hexahedraInROI.clear();
    quadInROI.clear();

    pointsOutROI.clear();
    edgesOutROI.clear();
    trianglesOutROI.clear();
    tetrahedraOutROI.clear();
    hexahedraOutROI.clear();
    quadOutROI.clear();

    const VecCoord* x0 = &f_X0.getValue();

    //Points
    for( unsigned i=0; i<x0->size(); ++i )
    {
        bool in = false;
        for (unsigned int bi=0; !in && bi<vb.size(); ++bi)
        {
            in = isPointInBox(i, vb[bi]);
        }
        if (in)
        {
            indices.push_back(i);
            pointsInROI.push_back((*x0)[i]);
            //sout<<"\nBoxROI<DataTypes>::update, add index "<< i << sendl;
        }
        else
        {
            pointsOutROI.push_back((*x0)[i]);
        }
    }

    //Edges
    if (f_computeEdges.getValue())
    {
        for(unsigned int i=0 ; i<edges.size() ; i++)
        {
            Edge e = edges[i];
            bool in = false;
            for (unsigned int bi=0; !in && bi<vb.size(); ++bi)
            {
                in = isEdgeInBox(e, vb[bi]);
            }
            if (in)
            {
                edgeIndices.push_back(i);
                edgesInROI.push_back(e);
            }
            else
            {
                edgesOutROI.push_back(e);
            }
        }
    }

    //Triangles
    if (f_computeTriangles.getValue())
    {
        for(unsigned int i=0 ; i<triangles.size() ; i++)
        {
            Triangle t = triangles[i];
            bool in = false;
            for (unsigned int bi=0; !in && bi<vb.size(); ++bi)
            {
                in = isTriangleInBox(t, vb[bi]);
            }
            if (in)
            {
                triangleIndices.push_back(i);
                trianglesInROI.push_back(t);
            }
            else
            {
                trianglesOutROI.push_back(t);
            }
        }
    }

    //Tetrahedra
    if (f_computeTetrahedra.getValue())
    {
        for(unsigned int i=0 ; i<tetrahedra.size() ; i++)
        {
            Tetra t = tetrahedra[i];
            bool in = false;
            for (unsigned int bi=0; !in && bi<vb.size(); ++bi)
            {
                in = isTetrahedronInBox(t, vb[bi]);
            }
            if (in)
            {
                tetrahedronIndices.push_back(i);
                tetrahedraInROI.push_back(t);
            }
            else
            {
                tetrahedraOutROI.push_back(t);
            }
        }
    }

    //Hexahedra
    if (f_computeHexahedra.getValue())
    {
        for(unsigned int i=0 ; i<hexahedra.size() ; i++)
        {
            Hexa t = hexahedra[i];
            bool in = false;
            for (unsigned int bi=0; !in && bi<vb.size(); ++bi)
            {
                in = isHexahedronInBox(t, vb[bi]);
            }
            if (in)
            {
                hexahedronIndices.push_back(i);
                hexahedraInROI.push_back(t);
            }
            else
            {
                hexahedraOutROI.push_back(t);
            }
        }
    }

    //Quads
    if (f_computeQuad.getValue())
    {
        for(unsigned int i=0 ; i<quad.size() ; i++)
        {
            Quad q = quad[i];
            bool in = false;
            for (unsigned int bi=0; !in && bi<vb.size(); ++bi)
            {
                in = isQuadInBox(q, vb[bi]);
            }
            if (in)
            {
                quadIndices.push_back(i);
                quadInROI.push_back(q);
            }
            else
            {
                quadOutROI.push_back(q);
            }
        }
    }

    f_nbIndices.setValue(indices.size());

    f_indices.endEdit();
    f_edgeIndices.endEdit();
    f_triangleIndices.endEdit();
    f_tetrahedronIndices.endEdit();
    f_hexahedronIndices.endEdit();
    f_quadIndices.endEdit();
}


template <class DataTypes, class BoxType>
bool BaseBoxROI<DataTypes, BoxType>::isPointInBox(const PointID& pid, const BoxType& b)
{
    const VecCoord* x0 = &f_X0.getValue();
    CPos p =  DataTypes::getCPos((*x0)[pid]);

    return ( isPointInBox(p,b) );
}

template <class DataTypes, class BoxType>
bool BaseBoxROI<DataTypes, BoxType>::isEdgeInBox(const Edge& e, const BoxType& b)
{
    const VecCoord* x0 = &f_X0.getValue();
    CPos p0 =  DataTypes::getCPos((*x0)[e[0]]);
    CPos p1 =  DataTypes::getCPos((*x0)[e[1]]);
    CPos c = (p1+p0)*0.5;

    return isPointInBox(c,b);
}

template <class DataTypes, class BoxType>
bool BaseBoxROI<DataTypes, BoxType>::isTriangleInBox(const Triangle& t, const BoxType& b)
{
    const VecCoord* x0 = &f_X0.getValue();
    CPos p0 =  DataTypes::getCPos((*x0)[t[0]]);
    CPos p1 =  DataTypes::getCPos((*x0)[t[1]]);
    CPos p2 =  DataTypes::getCPos((*x0)[t[2]]);
    CPos c = (p2+p1+p0)/3.0;

    return (isPointInBox(c,b));
}

template <class DataTypes, class BoxType>
bool BaseBoxROI<DataTypes, BoxType>::isTetrahedronInBox(const Tetra &t, const BoxType& b)
{
    const VecCoord* x0 = &f_X0.getValue();
    CPos p0 =  DataTypes::getCPos((*x0)[t[0]]);
    CPos p1 =  DataTypes::getCPos((*x0)[t[1]]);
    CPos p2 =  DataTypes::getCPos((*x0)[t[2]]);
    CPos p3 =  DataTypes::getCPos((*x0)[t[3]]);
    CPos c = (p3+p2+p1+p0)/4.0;

    return (isPointInBox(c,b));
}

template <class DataTypes, class BoxType>
bool BaseBoxROI<DataTypes, BoxType>::isHexahedronInBox(const Hexa &t, const BoxType& b)
{
    const VecCoord* x0 = &f_X0.getValue();
    CPos p0 =  DataTypes::getCPos((*x0)[t[0]]);
    CPos p1 =  DataTypes::getCPos((*x0)[t[1]]);
    CPos p2 =  DataTypes::getCPos((*x0)[t[2]]);
    CPos p3 =  DataTypes::getCPos((*x0)[t[3]]);
    CPos p4 =  DataTypes::getCPos((*x0)[t[4]]);
    CPos p5 =  DataTypes::getCPos((*x0)[t[5]]);
    CPos p6 =  DataTypes::getCPos((*x0)[t[6]]);
    CPos p7 =  DataTypes::getCPos((*x0)[t[7]]);
    CPos c = (p7+p6+p5+p4+p3+p2+p1+p0)/8.0;

    return (isPointInBox(c,b));
}

template <class DataTypes, class BoxType>
bool BaseBoxROI<DataTypes, BoxType>::isQuadInBox(const Quad& q, const BoxType& b)
{
    const VecCoord* x0 = &f_X0.getValue();
    CPos p0 =  DataTypes::getCPos((*x0)[q[0]]);
    CPos p1 =  DataTypes::getCPos((*x0)[q[1]]);
    CPos p2 =  DataTypes::getCPos((*x0)[q[2]]);
    CPos p3 =  DataTypes::getCPos((*x0)[q[3]]);
    CPos c = (p3+p2+p1+p0)/4.0;

    return (isPointInBox(c,b));
}


template <class DataTypes, class BoxType>
void BaseBoxROI<DataTypes, BoxType>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels() && !this->p_drawSize.getValue())
        return;

    const VecCoord* x0 = &f_X0.getValue();
    sofa::defaulttype::Vec4f color = sofa::defaulttype::Vec4f(1.0f, 0.4f, 0.4f, 1.0f);

    ///draw the boxes
    if( p_drawBoxes.getValue())
    {
        this->drawBoxes(vparams);
    }

    const unsigned int max_spatial_dimensions = std::min((unsigned int)3,(unsigned int)DataTypes::spatial_dimensions);

    ///draw points in ROI
    if( p_drawPoints.getValue())
    {
        float pointsWidth = p_drawSize.getValue() ? (float)p_drawSize.getValue() : 1;
        vparams->drawTool()->setLightingEnabled(false);
        sofa::helper::vector<sofa::defaulttype::Vector3> vertices;
        helper::ReadAccessor< Data<VecCoord > > pointsInROI = f_pointsInROI;
        for (unsigned int i=0; i<pointsInROI.size() ; ++i)
        {
            CPos p = DataTypes::getCPos(pointsInROI[i]);
            sofa::defaulttype::Vector3 pv;
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
        }
        vparams->drawTool()->drawPoints(vertices, pointsWidth, color);
    }

    ///draw edges in ROI
    if( p_drawEdges.getValue())
    {
        vparams->drawTool()->setLightingEnabled(false);
        float linesWidth = p_drawSize.getValue() ? (float)p_drawSize.getValue() : 1;
        sofa::helper::vector<sofa::defaulttype::Vector3> vertices;
        helper::ReadAccessor< Data<helper::vector<Edge> > > edgesInROI = f_edgesInROI;
        for (unsigned int i=0; i<edgesInROI.size() ; ++i)
        {
            Edge e = edgesInROI[i];
            for (unsigned int j=0 ; j<2 ; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[e[j]]);
                sofa::defaulttype::Vector3 pv;
                for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                    pv[j] = p[j];
                vertices.push_back( pv );
            }
        }
        vparams->drawTool()->drawLines(vertices, linesWidth, color);
    }

    ///draw triangles in ROI
    if( p_drawTriangles.getValue())
    {
        vparams->drawTool()->setLightingEnabled(false);
        sofa::helper::vector<sofa::defaulttype::Vector3> vertices;
        helper::ReadAccessor< Data<helper::vector<Triangle> > > trianglesInROI = f_trianglesInROI;
        for (unsigned int i=0; i<trianglesInROI.size() ; ++i)
        {
            Triangle t = trianglesInROI[i];
            for (unsigned int j=0 ; j<3 ; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[t[j]]);
                sofa::defaulttype::Vector3 pv;
                for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                    pv[j] = p[j];
                vertices.push_back( pv );
            }
        }
        vparams->drawTool()->drawTriangles(vertices, color);
    }

    ///draw tetrahedra in ROI
    if( p_drawTetrahedra.getValue())
    {
        vparams->drawTool()->setLightingEnabled(false);
        float linesWidth = p_drawSize.getValue() ? (float)p_drawSize.getValue() : 1;
        sofa::helper::vector<sofa::defaulttype::Vector3> vertices;
        helper::ReadAccessor< Data<helper::vector<Tetra> > > tetrahedraInROI = f_tetrahedraInROI;
        for (unsigned int i=0; i<tetrahedraInROI.size() ; ++i)
        {
            Tetra t = tetrahedraInROI[i];
            for (unsigned int j=0 ; j<4 ; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[t[j]]);
                sofa::defaulttype::Vector3 pv;
                for( unsigned int k=0 ; k<max_spatial_dimensions ; ++k )
                    pv[k] = p[k];
                vertices.push_back( pv );

                p = DataTypes::getCPos((*x0)[t[(j+1)%4]]);
                for( unsigned int k=0 ; k<max_spatial_dimensions ; ++k )
                    pv[k] = p[k];
                vertices.push_back( pv );
            }

            CPos p = DataTypes::getCPos((*x0)[t[0]]);
            sofa::defaulttype::Vector3 pv;
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[2]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[1]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[3]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
        }
        vparams->drawTool()->drawLines(vertices, linesWidth, color);
    }

    ///draw hexahedra in ROI
    if( p_drawHexahedra.getValue())
    {
        vparams->drawTool()->setLightingEnabled(false);
        float linesWidth = p_drawSize.getValue() ? (float)p_drawSize.getValue() : 1;
        sofa::helper::vector<sofa::defaulttype::Vector3> vertices;
        helper::ReadAccessor< Data<helper::vector<Hexa> > > hexahedraInROI = f_hexahedraInROI;
        for (unsigned int i=0; i<hexahedraInROI.size() ; ++i)
        {
            Hexa t = hexahedraInROI[i];
            for (unsigned int j=0 ; j<8 ; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[t[j]]);
                sofa::defaulttype::Vector3 pv;
                for( unsigned int k=0 ; k<max_spatial_dimensions ; ++k )
                    pv[k] = p[k];
                vertices.push_back( pv );

                p = DataTypes::getCPos((*x0)[t[(j+1)%4]]);
                for( unsigned int k=0 ; k<max_spatial_dimensions ; ++k )
                    pv[k] = p[k];
                vertices.push_back( pv );
            }

            CPos p = DataTypes::getCPos((*x0)[t[0]]);
            sofa::defaulttype::Vector3 pv;
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[2]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[1]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[3]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[4]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[5]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[6]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
            p = DataTypes::getCPos((*x0)[t[7]]);
            for( unsigned int j=0 ; j<max_spatial_dimensions ; ++j )
                pv[j] = p[j];
            vertices.push_back( pv );
        }
        vparams->drawTool()->drawLines(vertices, linesWidth, color);
    }

    ///draw quads in ROI
    if( p_drawQuads.getValue())
    {
        vparams->drawTool()->setLightingEnabled(false);
        float linesWidth = p_drawSize.getValue() ? (float)p_drawSize.getValue() : 1;
        sofa::helper::vector<sofa::defaulttype::Vector3> vertices;
        helper::ReadAccessor<Data<helper::vector<Quad> > > quadsInROI = f_quadInROI;
        for (unsigned i=0; i<quadsInROI.size(); ++i)
        {
            Quad q = quadsInROI[i];
            for (unsigned j=0; j<4; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[q[j]]);
                sofa::defaulttype::Vector3 pv;
                for (unsigned k=0; k<max_spatial_dimensions; k++)
                    pv[k] = p[k];
                vertices.push_back(pv);
            }
            for (unsigned j=0; j<4; j++)
            {
                CPos p = DataTypes::getCPos((*x0)[q[(j+1)%4]]);
                sofa::defaulttype::Vector3 pv;
                for (unsigned k=0; k<max_spatial_dimensions; k++)
                    pv[k] = p[k];
                vertices.push_back(pv);
            }

        }
        vparams->drawTool()->drawLines(vertices,linesWidth,color);
    }
}


} // namespace engine

} // namespace component

} // namespace sofa

#endif
