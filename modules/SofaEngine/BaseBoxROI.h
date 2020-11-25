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
#ifndef SOFA_COMPONENT_ENGINE_BASEBOXROI_H
#define SOFA_COMPONENT_ENGINE_BASEBOXROI_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/defaulttype/Vec.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/loader/MeshLoader.h>
#include <sofa/SofaGeneral.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa
{

namespace component
{

namespace engine
{

/**
 * Base class to find all the points/edges/triangles/tetrahedra located inside a given box.
 */
template <class DataTypes, class BoxType>
class BaseBoxROI : public core::DataEngine
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((BaseBoxROI<DataTypes,BoxType>),((core::DataEngine)));
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef defaulttype::vector<BoxType> Boxes;
    typedef core::topology::BaseMeshTopology::SetIndex SetIndex;
    typedef typename DataTypes::CPos CPos;

    typedef unsigned int PointID;
    typedef core::topology::BaseMeshTopology::Edge Edge;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;
    typedef core::topology::BaseMeshTopology::Tetra Tetra;
    typedef core::topology::BaseMeshTopology::Hexa Hexa;
    typedef core::topology::BaseMeshTopology::Quad Quad;

protected:
    BaseBoxROI();

    ~BaseBoxROI() {}

public:
    void init();

    void reinit();

    void update();

    virtual void updateBoxes() = 0;

    virtual void computeBBox(const core::ExecParams*  /*params*/ ) {}

    void draw(const core::visual::VisualParams*);

    virtual void drawBoxes(const core::visual::VisualParams* vparams) = 0;

protected:
    virtual bool isPointInBox(const CPos& p, const BoxType& b) = 0;
    
    bool isPointInBox(const PointID& pid, const BoxType& b);
    bool isEdgeInBox(const Edge& e, const BoxType& b);
    bool isTriangleInBox(const Triangle& t, const BoxType& b);
    bool isTetrahedronInBox(const Tetra& t, const BoxType& b);
    bool isHexahedronInBox(const Hexa& t, const BoxType& b);
    bool isQuadInBox(const Quad& q, const BoxType& b);

public:
    // Boxes
    Data<Boxes> boxes;

    // Modes
    Data<bool> f_computeEdges;
    Data<bool> f_computeTriangles;
    Data<bool> f_computeTetrahedra;
    Data<bool> f_computeHexahedra;
    Data<bool> f_computeQuad;

    //Inputs (Data to sort)
    Data<VecCoord> f_X0;
    Data<helper::vector<Edge> > f_edges;
    Data<helper::vector<Triangle> > f_triangles;
    Data<helper::vector<Tetra> > f_tetrahedra;
    Data<helper::vector<Hexa> > f_hexahedra;
    Data<helper::vector<Quad> > f_quad;

    //Output (Sorted data)
    Data<SetIndex> f_indices;
    Data<SetIndex> f_edgeIndices;
    Data<SetIndex> f_triangleIndices;
    Data<SetIndex> f_tetrahedronIndices;
    Data<SetIndex> f_hexahedronIndices;
    Data<SetIndex> f_quadIndices;
    Data<VecCoord > f_pointsInROI;
    Data<helper::vector<Edge> > f_edgesInROI;
    Data<helper::vector<Triangle> > f_trianglesInROI;
    Data<helper::vector<Tetra> > f_tetrahedraInROI;
    Data<helper::vector<Hexa> > f_hexahedraInROI;
    Data<helper::vector<Quad> > f_quadInROI;
    Data< unsigned int > f_nbIndices;
    Data<VecCoord > f_pointsOutROI;
    Data<helper::vector<Edge> > f_edgesOutROI;
    Data<helper::vector<Triangle> > f_trianglesOutROI;
    Data<helper::vector<Tetra> > f_tetrahedraOutROI;
    Data<helper::vector<Hexa> > f_hexahedraOutROI;
    Data<helper::vector<Quad> > f_quadOutROI;

    // Draw parameters
    Data<bool> p_drawBoxes;
    Data<bool> p_drawPoints;
    Data<bool> p_drawEdges;
    Data<bool> p_drawTriangles;
    Data<bool> p_drawTetrahedra;
    Data<bool> p_drawHexahedra;
    Data<bool> p_drawQuads;
    Data<double> p_drawSize;
};


} // namespace engine

} // namespace component

} // namespace sofa

#endif
