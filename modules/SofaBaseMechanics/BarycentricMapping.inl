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
#ifndef SOFA_COMPONENT_MAPPING_BARYCENTRICMAPPING_INL
#define SOFA_COMPONENT_MAPPING_BARYCENTRICMAPPING_INL

#include <SofaBaseMechanics/BarycentricMapping.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/Mapping.inl>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <SofaBaseTopology/RegularGridTopology.h>
#include <SofaBaseTopology/SparseGridTopology.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/QuadSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/HexahedronSetTopologyContainer.h>
#include <SofaBaseTopology/EdgeSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>
#include <SofaBaseTopology/QuadSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TetrahedronSetGeometryAlgorithms.h>
#include <SofaBaseTopology/HexahedronSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TopologyData.inl>

#include <sofa/helper/vector.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/system/config.h>

#include <sofa/simulation/common/Simulation.h>

#include <algorithm>
#include <iostream>

//#include <sofa/component/collision/MeshIntTool.h>


namespace sofa
{

namespace component
{

namespace mapping
{

template <class TIn, class TOut>
BarycentricMapping<TIn, TOut>::BarycentricMapping()
    : Inherit()
    , mapper(initLink("mapper","Internal mapper created depending on the type of topology"))
    , useRestPosition(core::objectmodel::Base::initData(&useRestPosition, false, "useRestPosition", "Use the rest position of the input and output models to initialize the mapping"))
    , d_handleTopologyChange(core::objectmodel::Base::initData(&d_handleTopologyChange, true, "handleTopologyChange", "Enable (partial) support of topological changes (disable if another component takes care of this)"))
#ifdef SOFA_DEV
    , sleeping(core::objectmodel::Base::initData(&sleeping, false, "sleeping", "is the mapping sleeping (not computed)"))
#endif
{
}

template <class TIn, class TOut>
BarycentricMapping<TIn, TOut>::BarycentricMapping(core::State<In>* from, core::State<Out>* to, typename Mapper::SPtr mapper)
    : Inherit ( from, to )
    , mapper(initLink("mapper","Internal mapper created depending on the type of topology"), mapper)
    , useRestPosition(core::objectmodel::Base::initData(&useRestPosition, false, "useRestPosition", "Use the rest position of the input and output models to initialize the mapping"))
    , d_handleTopologyChange(core::objectmodel::Base::initData(&d_handleTopologyChange, true, "handleTopologyChange", "Enable (partial) support of topological changes (disable if another component takes care of this)"))
#ifdef SOFA_DEV
    , sleeping(core::objectmodel::Base::initData(&sleeping, false, "sleeping", "is the mapping sleeping (not computed)"))
#endif
{
    if (mapper)
        this->addSlave(mapper.get());
}

template <class TIn, class TOut>
BarycentricMapping<TIn, TOut>::BarycentricMapping (core::State<In>* from, core::State<Out>* to, BaseMeshTopology * topology )
    : Inherit ( from, to )
    , mapper (initLink("mapper","Internal mapper created depending on the type of topology"))
    , useRestPosition(core::objectmodel::Base::initData(&useRestPosition, false, "useRestPosition", "Use the rest position of the input and output models to initialize the mapping"))
    , d_handleTopologyChange(core::objectmodel::Base::initData(&d_handleTopologyChange, true, "handleTopologyChange", "Enable (partial) support of topological changes (disable if another component takes care of this)"))
#ifdef SOFA_DEV
    , sleeping(core::objectmodel::Base::initData(&sleeping, false, "sleeping", "is the mapping sleeping (not computed)"))
#endif
{
    if (topology)
    {
        createMapperFromTopology ( topology );
    }
}

template <class TIn, class TOut>
BarycentricMapping<TIn, TOut>::~BarycentricMapping()
{
}

template <class In, class Out>
void BarycentricMapperRegularGridTopology<In,Out>::clear ( int reserve )
{
    updateJ = true;
    map.clear();
    if ( reserve>0 ) map.reserve ( reserve );
}

template <class In, class Out>
int BarycentricMapperRegularGridTopology<In,Out>::addPointInCube ( const int cubeIndex, const SReal* baryCoords )
{
    map.resize ( map.size() +1 );
    CubeData& data = *map.rbegin();
    data.in_index = cubeIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    data.baryCoords[2] = ( Real ) baryCoords[2];
    return map.size()-1;
}

template <class In, class Out>
void BarycentricMapperRegularGridTopology<In,Out>::init ( const typename Out::VecCoord& out, const typename In::VecCoord& /*in*/ )
{
    //if ( map.size() != 0 ) return;
    updateJ = true;

    int outside = 0;
    clear ( out.size() );
    if ( fromTopology->isVolume() )
    {
        for ( unsigned int i=0; i<out.size(); i++ )
        {
            sofa::defaulttype::Vector3 coefs;
            int cube = fromTopology->findCube ( sofa::defaulttype::Vector3 ( Out::getCPos(out[i]) ), coefs[0], coefs[1], coefs[2] );
            if ( cube==-1 )
            {
                ++outside;
                cube = fromTopology->findNearestCube ( sofa::defaulttype::Vector3 ( Out::getCPos(out[i]) ), coefs[0], coefs[1], coefs[2] );
            }

            this->addPointInCube ( cube, coefs.ptr() );
        }
    }
}

template <class In, class Out>
void BarycentricMapperSparseGridTopology<In,Out>::clear ( int reserve )
{
    updateJ = true;
    map.clear();
    if ( reserve>0 ) map.reserve ( reserve );
}

template <class In, class Out>
int BarycentricMapperSparseGridTopology<In,Out>::addPointInCube ( const int cubeIndex, const SReal* baryCoords )
{
    map.resize ( map.size() +1 );
    CubeData& data = *map.rbegin();
    data.in_index = cubeIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    data.baryCoords[2] = ( Real ) baryCoords[2];
    return map.size()-1;
}

template <class In, class Out>
void BarycentricMapperSparseGridTopology<In,Out>::init ( const typename Out::VecCoord& out, const typename In::VecCoord& /*in*/ )
{
    if ( this->map.size() != 0 ) return;
    updateJ = true;
    int outside = 0;
    clear ( out.size() );

    if ( fromTopology->isVolume() )
    {
        for ( unsigned int i=0; i<out.size(); i++ )
        {
            sofa::defaulttype::Vector3 coefs;
            int cube = fromTopology->findCube ( sofa::defaulttype::Vector3 ( Out::getCPos(out[i]) ), coefs[0], coefs[1], coefs[2] );
            if ( cube==-1 )
            {
                ++outside;
                cube = fromTopology->findNearestCube ( sofa::defaulttype::Vector3 ( Out::getCPos(out[i]) ), coefs[0], coefs[1], coefs[2] );
            }
            sofa::defaulttype::Vector3 baryCoords = coefs;
            this->addPointInCube ( cube, baryCoords.ptr() );
        }
    }
}

template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::clear1d ( int reserve )
{
    updateJ = true;
    map1d.clear(); if ( reserve>0 ) map1d.reserve ( reserve );
}

template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::clear2d ( int reserve )
{
    updateJ = true;
    map2d.clear(); if ( reserve>0 ) map2d.reserve ( reserve );
}

template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::clear3d ( int reserve )
{
    updateJ = true;
    map3d.clear(); if ( reserve>0 ) map3d.reserve ( reserve );
}

template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::clear ( int reserve )
{
    updateJ = true;
    map1d.clear(); if ( reserve>0 ) map1d.reserve ( reserve );
    map2d.clear(); if ( reserve>0 ) map2d.reserve ( reserve );
    map3d.clear(); if ( reserve>0 ) map3d.reserve ( reserve );
}

template <class In, class Out>
int BarycentricMapperMeshTopology<In,Out>::addPointInLine ( const int lineIndex, const SReal* baryCoords )
{
    map1d.resize ( map1d.size() +1 );
    MappingData1D& data = *map1d.rbegin();
    data.in_index = lineIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    return map1d.size()-1;
}

template <class In, class Out>
int BarycentricMapperMeshTopology<In,Out>::addPointInTriangle ( const int triangleIndex, const SReal* baryCoords )
{
    map2d.resize ( map2d.size() +1 );
    MappingData2D& data = *map2d.rbegin();
    data.in_index = triangleIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    return map2d.size()-1;
}

template <class In, class Out>
int BarycentricMapperMeshTopology<In,Out>::addPointInQuad ( const int quadIndex, const SReal* baryCoords )
{
    map2d.resize ( map2d.size() +1 );
    MappingData2D& data = *map2d.rbegin();
    data.in_index = quadIndex + this->fromTopology->getNbTriangles();
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    return map2d.size()-1;
}

template <class In, class Out>
int BarycentricMapperMeshTopology<In,Out>::addPointInTetra ( const int tetraIndex, const SReal* baryCoords )
{
    map3d.resize ( map3d.size() +1 );
    MappingData3D& data = *map3d.rbegin();
    data.in_index = tetraIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    data.baryCoords[2] = ( Real ) baryCoords[2];
    return map3d.size()-1;
}

template <class In, class Out>
int BarycentricMapperMeshTopology<In,Out>::addPointInCube ( const int cubeIndex, const SReal* baryCoords )
{
    map3d.resize ( map3d.size() +1 );
    MappingData3D& data = *map3d.rbegin();
    data.in_index = cubeIndex + this->fromTopology->getNbTetrahedra();
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    data.baryCoords[2] = ( Real ) baryCoords[2];
    return map3d.size()-1;
}

template <class In, class Out>
int BarycentricMapperMeshTopology<In,Out>::createPointInLine ( const typename Out::Coord& p, int lineIndex, const typename In::VecCoord* points )
{
    SReal baryCoords[1];
    const sofa::core::topology::BaseMeshTopology::Line& elem = this->fromTopology->getLine ( lineIndex );
    const typename In::CPos p0 = In::getCPos( ( *points ) [elem[0]] );
    const typename In::CPos pA = In::getCPos( ( *points ) [elem[1]] ) - p0;
    typename In::CPos pos = Out::getCPos(p) - p0;
    baryCoords[0] = ( pos*pA ) / pA.norm2() ;
    return this->addPointInLine ( lineIndex, baryCoords );
}

template <class In, class Out>
int BarycentricMapperMeshTopology<In,Out>::createPointInTriangle ( const typename Out::Coord& p, int triangleIndex, const typename In::VecCoord* points )
{
    SReal baryCoords[2];
    const sofa::core::topology::BaseMeshTopology::Triangle& elem = this->fromTopology->getTriangle ( triangleIndex );

    const typename In::CPos & p1 = In::getCPos( ( *points ) [elem[0]] );
    const typename In::CPos & p2 = In::getCPos( ( *points ) [elem[1]] );
    const typename In::CPos & p3 = In::getCPos( ( *points ) [elem[2]] );
    const typename In::CPos &  to_be_projected = Out::getCPos(p) ;

    const typename In::CPos AB = p2-p1;
    const typename In::CPos AC = p3-p1;
    const typename In::CPos AQ = to_be_projected -p1;
    sofa::defaulttype::Mat<2,2,typename In::Real> A;
    sofa::defaulttype::Vec<2,typename In::Real> b;
    A[0][0] = AB*AB;
    A[1][1] = AC*AC;
    A[0][1] = A[1][0] = AB*AC;
    b[0] = AQ*AB;
    b[1] = AQ*AC;
    const typename In::Real det = sofa::defaulttype::determinant(A);

    baryCoords[0] = (b[0]*A[1][1] - b[1]*A[0][1])/det;
    baryCoords[1]  = (b[1]*A[0][0] - b[0]*A[1][0])/det;

    if (baryCoords[0] < 0 || baryCoords[1] < 0 || baryCoords[0] + baryCoords[1] > 1)
    {
        // nearest point is on an edge or corner
        // barycentric coordinate on AB
        SReal pAB = b[0] / A[0][0]; // AQ*AB / AB*AB
        // barycentric coordinate on AC
        SReal pAC = b[1] / A[1][1]; // AQ*AC / AB*AB
        if (pAB < 0 && pAC < 0)
        {
            // closest point is A
            baryCoords[0] = 0.0;
            baryCoords[1] = 0.0;
        }
        else if (pAB < 1 && baryCoords[1] < 0)
        {
            // closest point is on AB
            baryCoords[0] = pAB;
            baryCoords[1] = 0.0;
        }
        else if (pAC < 1 && baryCoords[0] < 0)
        {
            // closest point is on AC
            baryCoords[0] = 0.0;
            baryCoords[1] = pAC;
        }
        else
        {
            // barycentric coordinate on BC
            // BQ*BC / BC*BC = (AQ-AB)*(AC-AB) / (AC-AB)*(AC-AB) = (AQ*AC-AQ*AB + AB*AB-AB*AC) / (AB*AB+AC*AC-2AB*AC)
            SReal pBC = (b[1] - b[0] + A[0][0] - A[0][1]) / (A[0][0] + A[1][1] - 2*A[0][1]); // BQ*BC / BC*BC
            if (pBC < 0)
            {
                // closest point is B
                baryCoords[0] = 1.0;
                baryCoords[1] = 0.0;
            }
            else if (pBC > 1)
            {
                // closest point is C
                baryCoords[0] = 0.0;
                baryCoords[1] = 1.0;
            }
            else
            {
                // closest point is on BC
                baryCoords[0] = 1.0-pBC;
                baryCoords[1] = pBC;
            }
        }
    }

//    std::cout<<"barycoords : "<<baryCoords[0]<<" "<<baryCoords[1]<<std::endl;
//    const sofa::core::topology::BaseMeshTopology::Triangle& elem = this->fromTopology->getTriangle ( triangleIndex );
//    const typename In::Coord p0 = ( *points ) [elem[0]];
//    const typename In::Coord pA = ( *points ) [elem[1]] - p0;
//    const typename In::Coord pB = ( *points ) [elem[2]] - p0;
//    // First project to plane
//    typename In::Coord normal = cross ( pA, pB );
//    Real norm2 = normal.norm2();
//    typename In::Coord pos = Out::getCPos(p) - p0;
//    pos -= normal* ( ( pos*normal ) /norm2 );
//    baryCoords[0] = ( Real ) sqrt ( cross ( pB, pos ).norm2() / norm2 );
//    baryCoords[1] = ( Real ) sqrt ( cross ( pA, pos ).norm2() / norm2 );

//    typename In::Coord result = p0 * (1-baryCoords[0]-baryCoords[1]) + ( *points ) [elem[1]] * (baryCoords[0]) + ( *points ) [elem[2]] * (baryCoords[1]);
//    std::cout<<"Result "<<result<<std::endl;

    return this->addPointInTriangle ( triangleIndex, baryCoords );
}

template <class In, class Out>
int BarycentricMapperMeshTopology<In,Out>::createPointInQuad ( const typename Out::Coord& p, int quadIndex, const typename In::VecCoord* points )
{
    SReal baryCoords[2];
    const sofa::core::topology::BaseMeshTopology::Quad& elem = this->fromTopology->getQuad ( quadIndex );
    const typename In::CPos p0 = In::getCPos( ( *points ) [elem[0]] );
    const typename In::CPos pA = In::getCPos( ( *points ) [elem[1]] ) - p0;
    const typename In::CPos pB = In::getCPos( ( *points ) [elem[3]] ) - p0;
    typename In::CPos pos = Out::getCPos(p) - p0;
    sofa::defaulttype::Mat<3,3,typename In::Real> m,mt,base;
    m[0] = pA;
    m[1] = pB;
    m[2] = cross ( pA, pB );
    mt.transpose ( m );
    base.invert ( mt );
    typename In::CPos base0 = base[0];
    typename In::CPos base1 = base[1];
    baryCoords[0] = base0 * pos;
    baryCoords[1] = base1 * pos;
    return this->addPointInQuad ( quadIndex, baryCoords );
}

template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::init ( const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    int outside = 0;
    updateJ = true;

    const sofa::core::topology::BaseMeshTopology::SeqTetrahedra& tetrahedra = this->fromTopology->getTetrahedra();
#ifdef SOFA_NEW_HEXA
    const sofa::core::topology::BaseMeshTopology::SeqHexahedra& cubes = this->fromTopology->getHexahedra();
#else
    const sofa::core::topology::BaseMeshTopology::SeqCubes& cubes = this->fromTopology->getCubes();
#endif
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& triangles = this->fromTopology->getTriangles();
    const sofa::core::topology::BaseMeshTopology::SeqQuads& quads = this->fromTopology->getQuads();
    sofa::helper::vector<sofa::defaulttype::Matrix3> bases;
    sofa::helper::vector<sofa::defaulttype::Vector3> centers;
    if ( tetrahedra.empty() && cubes.empty() )
    {
        if ( triangles.empty() && quads.empty() )
        {
            //no 3D elements, nor 2D elements -> map on 1D elements

            const sofa::core::topology::BaseMeshTopology::SeqEdges& edges = this->fromTopology->getEdges();
            if ( edges.empty() ) return;

            clear1d ( out.size() );

            sofa::helper::vector< SReal >   lengthEdges;
            sofa::helper::vector< sofa::defaulttype::Vector3 > unitaryVectors;

            unsigned int e;
            for ( e=0; e<edges.size(); e++ )
            {
                lengthEdges.push_back ( ( In::getCPos( in[edges[e][1]] - in[edges[e][0]] ) ).norm() );

                sofa::defaulttype::Vector3 V12 = ( In::getCPos( in[edges[e][1]] - in[edges[e][0]] ) );
                V12.normalize();
                unitaryVectors.push_back ( V12 );
            }

            for ( unsigned int i=0; i<out.size(); i++ )
            {
                SReal coef=0;
                for ( e=0; e<edges.size(); e++ )
                {
                    SReal lengthEdge = lengthEdges[e];
                    sofa::defaulttype::Vector3 V12 =unitaryVectors[e];

                    coef = ( V12 ) * sofa::defaulttype::Vector3 ( Out::getCPos(out[i])-In::getCPos(in[edges[e][0]]) ) /lengthEdge;
                    if ( coef >= 0 && coef <= 1 )
                    {
                        addPointInLine ( e,&coef );
                        break;
                    }

                }
                //If no good coefficient has been found, we add to the last element
                if ( e == edges.size() ) addPointInLine ( edges.size()-1,&coef );

            }
        }
        else
        {
            // no 3D elements -> map on 2D elements
            clear2d ( out.size() ); // reserve space for 2D mapping
            int c0 = triangles.size();
            bases.resize ( triangles.size() +quads.size() );
            centers.resize ( triangles.size() +quads.size() );
            for ( unsigned int t = 0; t < triangles.size(); t++ )
            {
                sofa::defaulttype::Mat3x3d m,mt;
                m[0] = In::getCPos(in[triangles[t][1]] - in[triangles[t][0]]);
                m[1] = In::getCPos(in[triangles[t][2]] - in[triangles[t][0]]);
                m[2] = cross ( m[0],m[1] );
                mt.transpose ( m );
                bases[t].invert ( mt );
                centers[t] = In::getCPos(in[triangles[t][0]] + in[triangles[t][1]] + in[triangles[t][2]])/3;
            }
            for ( unsigned int c = 0; c < quads.size(); c++ )
            {
                sofa::defaulttype::Mat3x3d m,mt;
                m[0] = In::getCPos(in[quads[c][1]] - in[quads[c][0]]);
                m[1] = In::getCPos(in[quads[c][3]] - in[quads[c][0]]);
                m[2] = cross ( m[0],m[1] );
                mt.transpose ( m );
                bases[c0+c].invert ( mt );
                centers[c0+c] = In::getCPos( in[quads[c][0]] + in[quads[c][1]] + in[quads[c][2]] + in[quads[c][3]] )*0.25;
            }
            for ( unsigned int i=0; i<out.size(); i++ )
            {
                sofa::defaulttype::Vector3 pos = Out::getCPos(out[i]);
                sofa::defaulttype::Vector3 coefs;
                int index = -1;
                double distance = 1e10;
                for ( unsigned int t = 0; t < triangles.size(); t++ )
                {
                    sofa::defaulttype::Vec3d v = bases[t] * ( pos - In::getCPos(in[triangles[t][0]]) );
                    double d = std::max ( std::max ( -v[0],-v[1] ),std::max ( ( v[2]<0?-v[2]:v[2] )-0.01,v[0]+v[1]-1 ) );
                    if ( d>0 ) d = ( pos-centers[t] ).norm2();
                    if ( d<distance ) { coefs = v; distance = d; index = t; }
                }
                for ( unsigned int c = 0; c < quads.size(); c++ )
                {
                    sofa::defaulttype::Vec3d v = bases[c0+c] * ( pos - In::getCPos(in[quads[c][0]]) );
                    double d = std::max ( std::max ( -v[0],-v[1] ),std::max ( std::max ( v[1]-1,v[0]-1 ),std::max ( v[2]-0.01,-v[2]-0.01 ) ) );
                    if ( d>0 ) d = ( pos-centers[c0+c] ).norm2();
                    if ( d<distance ) { coefs = v; distance = d; index = c0+c; }
                }
                if ( distance>0 )
                {
                    ++outside;
                }
                if ( index < c0 )
                    addPointInTriangle ( index, coefs.ptr() );
                else
                    addPointInQuad ( index-c0, coefs.ptr() );
            }
        }
    }
    else
    {
        clear3d ( out.size() ); // reserve space for 3D mapping
        int c0 = tetrahedra.size();
        bases.resize ( tetrahedra.size() +cubes.size() );
        centers.resize ( tetrahedra.size() +cubes.size() );
        for ( unsigned int t = 0; t < tetrahedra.size(); t++ )
        {
            sofa::defaulttype::Mat3x3d m,mt;
            m[0] = In::getCPos(in[tetrahedra[t][1]] - in[tetrahedra[t][0]]);
            m[1] = In::getCPos(in[tetrahedra[t][2]] - in[tetrahedra[t][0]]);
            m[2] = In::getCPos(in[tetrahedra[t][3]] - in[tetrahedra[t][0]]);
            mt.transpose ( m );
            bases[t].invert ( mt );
            centers[t] = In::getCPos( in[tetrahedra[t][0]] + in[tetrahedra[t][1]] + in[tetrahedra[t][2]] + in[tetrahedra[t][3]]) *0.25;
            //sout << "Tetra "<<t<<" center="<<centers[t]<<" base="<<m<<sendl;
        }
        for ( unsigned int c = 0; c < cubes.size(); c++ )
        {
            sofa::defaulttype::Mat3x3d m,mt;
            m[0] = In::getCPos(in[cubes[c][1]] - in[cubes[c][0]]);
#ifdef SOFA_NEW_HEXA
            m[1] = In::getCPos(in[cubes[c][3]] - in[cubes[c][0]]);
#else
            m[1] = In::getCPos(in[cubes[c][2]] - in[cubes[c][0]]);
#endif
            m[2] = In::getCPos(in[cubes[c][4]] - in[cubes[c][0]]);
            mt.transpose ( m );
            bases[c0+c].invert ( mt );
            centers[c0+c] = In::getCPos(in[cubes[c][0]] + in[cubes[c][1]] + in[cubes[c][2]] + in[cubes[c][3]] + in[cubes[c][4]] + in[cubes[c][5]] + in[cubes[c][6]] + in[cubes[c][7]] )*0.125;
        }
        for ( unsigned int i=0; i<out.size(); i++ )
        {
            sofa::defaulttype::Vector3 pos = Out::getCPos(out[i]);
            sofa::defaulttype::Vector3 coefs;
            int index = -1;
            double distance = 1e10;
            for ( unsigned int t = 0; t < tetrahedra.size(); t++ )
            {
                sofa::defaulttype::Vector3 v = bases[t] * ( pos - In::getCPos(in[tetrahedra[t][0]]) );
                double d = std::max ( std::max ( -v[0],-v[1] ),std::max ( -v[2],v[0]+v[1]+v[2]-1 ) );
                if ( d>0 ) d = ( pos-centers[t] ).norm2();
                if ( d<distance ) { coefs = v; distance = d; index = t; }
            }
            for ( unsigned int c = 0; c < cubes.size(); c++ )
            {
                sofa::defaulttype::Vector3 v = bases[c0+c] * ( pos - In::getCPos(in[cubes[c][0]]) );
                double d = std::max ( std::max ( -v[0],-v[1] ),std::max ( std::max ( -v[2],v[0]-1 ),std::max ( v[1]-1,v[2]-1 ) ) );
                if ( d>0 ) d = ( pos-centers[c0+c] ).norm2();
                if ( d<distance ) { coefs = v; distance = d; index = c0+c; }
            }
            if ( distance>0 )
            {
                ++outside;
            }
            if ( index < c0 )
                addPointInTetra ( index, coefs.ptr() );
            else
                addPointInCube ( index-c0, coefs.ptr() );
        }
    }
}





template <class In, class Out>
void BarycentricMapperEdgeSetTopology<In,Out>::clear ( int reserve )
{
    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    vectorData.clear(); if ( reserve>0 ) vectorData.reserve ( reserve );
    map.endEdit();
}


template <class In, class Out>
int BarycentricMapperEdgeSetTopology<In,Out>::addPointInLine ( const int edgeIndex, const SReal* baryCoords )
{
    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    vectorData.resize ( map.getValue().size() +1 );
    map.endEdit();
    MappingData& data = *vectorData.rbegin();
    data.in_index = edgeIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    return map.getValue().size()-1;
}

template <class In, class Out>
int BarycentricMapperEdgeSetTopology<In,Out>::createPointInLine ( const typename Out::Coord& p, int edgeIndex, const typename In::VecCoord* points )
{
    SReal baryCoords[1];
    const topology::Edge& elem = this->fromTopology->getEdge ( edgeIndex );
    const typename In::CPos p0 = In::getCPos( ( *points ) [elem[0]] );
    const typename In::CPos pA = In::getCPos( ( *points ) [elem[1]] ) - p0;
    typename In::CPos pos = Out::getCPos(p) - p0;
    baryCoords[0] = dot ( pA, pos ) /dot ( pA, pA );
    return this->addPointInLine ( edgeIndex, baryCoords );
}

template <class In, class Out>
void BarycentricMapperEdgeSetTopology<In,Out>::init ( const typename Out::VecCoord& /*out*/, const typename In::VecCoord& /*in*/ )
{
    _fromContainer->getContext()->get ( _fromGeomAlgo );
}

template <class In, class Out>
void BarycentricMapperEdgeSetTopology<In,Out>::initTopologyChange()
{
    if (this->toTopology)
    {
        map.createTopologicalEngine(this->toTopology);
        map.registerTopologicalData();
    }
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In,Out>::clear ( int reserve )
{
    helper::WriteAccessor<Data<sofa::helper::vector<MappingData> > > vectorData = map;
    vectorData.clear(); if ( reserve>0 ) vectorData.reserve ( reserve );

    helper::WriteAccessor<Data<VecBaryTriangleInfo> > vBaryTriangleInfo = d_vBaryTriangleInfo;
    vBaryTriangleInfo.clear();

    m_dirtyPoints.clear();
}

template <class In, class Out>
int BarycentricMapperTriangleSetTopology<In,Out>::addPointInTriangle ( const int triangleIndex, const SReal* baryCoords )
{
    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    vectorData.resize ( map.getValue().size() +1 );
    MappingData& data = *vectorData.rbegin();
    map.endEdit();
    data.in_index = triangleIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    return map.getValue().size()-1;
}

template <class In, class Out>
int BarycentricMapperTriangleSetTopology<In,Out>::setPointInTriangle ( const int pointIndex, const int triangleIndex, const SReal* baryCoords )
{
    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    vectorData.resize ( map.getValue().size() );
    MappingData& data = vectorData[pointIndex];
    map.endEdit();
    data.in_index = triangleIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    return pointIndex;
}

template <class In, class Out>
int BarycentricMapperTriangleSetTopology<In,Out>::createPointInTriangle ( const typename Out::Coord& p, int triangleIndex, const typename In::VecCoord* points )
{
    SReal baryCoords[2];
    const topology::Triangle& elem = this->fromTopology->getTriangle ( triangleIndex );
    const typename In::CPos p0 = In::getCPos( ( *points ) [elem[0]] );
    const typename In::CPos pA = In::getCPos( ( *points ) [elem[1]] ) - p0;
    const typename In::CPos pB = In::getCPos( ( *points ) [elem[2]] ) - p0;
    typename In::CPos pos = Out::getCPos(p) - p0;
    // First project to plane
    typename In::CPos normal = cross ( pA, pB);
    Real norm2 = normal.norm2();
    pos-= normal* ( ( pos*normal ) /norm2);

    baryCoords[0] = ( Real ) sqrt ( cross ( pB, pos ).norm2() / norm2 );
    baryCoords[1] = ( Real ) sqrt ( cross ( pA, pos ).norm2() / norm2 );
    return this->addPointInTriangle ( triangleIndex, baryCoords );
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In,Out>::init(const typename Out::VecCoord& out, const typename In::VecCoord& in)
{
    if (!m_fromContainer)
    {
        serr << "Link to the topology failed" << sendl;
        return;
    }

    if (!m_stateFrom)
    {
        serr << "Link to the mechanical state in failed" << sendl;
        return;
    }

    if (!m_stateTo)
    {
        serr << "Link to the mechanical state out failed" << sendl;
        return;
    }


    if (m_fromContainer)
    {       
        // initialize d_vBaryTriangleInfo
        clear(out.size());
        helper::WriteAccessor<Data<VecBaryTriangleInfo> > vBaryTriangleInfo = d_vBaryTriangleInfo;

        const sofa::helper::vector<topology::Triangle>& triangles = this->m_fromContainer->getTriangles();
        vBaryTriangleInfo.resize(triangles.size());

        sofa::helper::vector< unsigned int > ancestorsIndex;
        const sofa::helper::vector< double > ancestorsCoeff;

        for (unsigned int t = 0; t < triangles.size(); t++)
        {
            m_vBTInfoHandler->applyCreateFunction(t, vBaryTriangleInfo[t], triangles[t], ancestorsIndex, ancestorsCoeff);
        }

        for (unsigned int i = 0; i < out.size(); ++i) 
        {
            m_dirtyPoints.emplace(i);
        }

        projectDirtyPoints(out, in);
    }
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In, Out>::TriangleInfoHandler::applyCreateFunction(unsigned int /*t*/,
    BaryElementInfo& baryTriangleInfo,
    const Triangle & triangle,
    const sofa::helper::vector< unsigned int >& ancestors,
    const sofa::helper::vector< double >& /*coeffs*/)
{
    // get restPositions
    helper::ReadAccessor< Data<typename core::State< In >::VecCoord > >  pIn = obj->m_useRestPosition ?
            obj->m_stateFrom->read(sofa::core::ConstVecCoordId::restPosition()) : obj->m_stateFrom->read(sofa::core::ConstVecCoordId::position());

    sofa::defaulttype::Mat3x3d m;
    m[0] = In::getCPos(pIn[triangle[1]] - pIn[triangle[0]]);
    m[1] = In::getCPos(pIn[triangle[2]] - pIn[triangle[0]]);
    m[2] = cross(m[0], m[1]);
    sofa::defaulttype::Mat3x3d mTranspose;
    mTranspose.transpose(m);

    baryTriangleInfo.restBase.invert(mTranspose);
    baryTriangleInfo.restCenter = (In::getCPos(pIn[triangle[0]] + pIn[triangle[1]] + pIn[triangle[2]])) / 3;

    helper::WriteAccessor<Data<VecBaryTriangleInfo> > vBaryTriangleInfo = obj->d_vBaryTriangleInfo;

    for (unsigned int ancestor : ancestors)
    {
        assert(ancestor < vBaryTriangleInfo.size());
        BaryElementInfo& baryTriangleInfoA = vBaryTriangleInfo[ancestor];
        baryTriangleInfoA.vPointsIncluded.clear();
    }
}


template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In, Out>::TriangleInfoHandler::applyDestroyFunction(unsigned int /*t*/, BaryElementInfo& baryTriangleInfo)
{
    baryTriangleInfo.vPointsIncluded.clear();
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In, Out>::TriangleInfoHandler::swap(unsigned int i1, unsigned int i2)
{
    helper::ReadAccessor<Data<VecBaryTriangleInfo> > vBaryTriangleInfo = obj->d_vBaryTriangleInfo;

    helper::WriteAccessor<Data<sofa::helper::vector<MappingData> > > vectorData = obj->map;

    const BaryElementInfo& baryTriangleInfo1 = vBaryTriangleInfo[i1];
    const BaryElementInfo& baryTriangleInfo2 = vBaryTriangleInfo[i2];
    for (auto pointIncluded : baryTriangleInfo1.vPointsIncluded)
    {
        vectorData[pointIncluded].in_index = i2;
    }

    for (auto pointIncluded : baryTriangleInfo2.vPointsIncluded)
    {
        vectorData[pointIncluded].in_index = i1;
    }

    component::topology::TopologyDataHandler<Triangle, VecBaryTriangleInfo >::swap(i1, i2);
}

//template <class In, class Out>
//void BarycentricMapperTriangleSetTopology<In, Out>::TriangleInfoHandler::move( const sofa::helper::vector<unsigned int> &indexList,
//const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
//const sofa::helper::vector< sofa::helper::vector< double > >& coefs)
//{
//    //TO DO
//}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In, Out>::projectDirtyPoints(const typename Out::VecCoord& out, const typename In::VecCoord& in)
{
    if (m_dirtyPoints.empty() || !m_fromContainer) return;

    // get restPositions
    helper::ReadAccessor< Data<typename core::State< In >::VecCoord > >  pIn = m_stateFrom->read(sofa::core::ConstVecCoordId::restPosition());
    helper::ReadAccessor< Data<typename core::State< Out >::VecCoord> > pOut = m_stateTo->read(sofa::core::ConstVecCoordId::restPosition());

    // evaluate dirty projections : fill in d_vBaryTriangleInfo and map 
    helper::WriteAccessor<Data<VecBaryTriangleInfo> > vBaryTriangleInfo = d_vBaryTriangleInfo;
    helper::WriteAccessor<Data<sofa::helper::vector<MappingData> > > vectorData = map;
    vectorData.resize(out.size());

    const sofa::helper::vector<topology::Triangle>& triangles = this->fromTopology->getTriangles();
    
    assert(triangles.size() == vBaryTriangleInfo.size());

    for (auto dirtyPoint : m_dirtyPoints)
    {
        sofa::defaulttype::Vec3d pos = (m_useRestPosition && pOut.size() > 0) ? Out::getCPos(pOut[dirtyPoint]) : Out::getCPos(out[dirtyPoint]);
        sofa::defaulttype::Vector3 coefs;
        int index = -1;
        double distance = std::numeric_limits<double>::max();

        for (unsigned int t = 0; t < triangles.size(); t++)
        {
            sofa::defaulttype::Vec3d xTri = (m_useRestPosition && pIn.size() > 0) ? In::getCPos(pIn[triangles[t][0]]) : In::getCPos(in[triangles[t][0]]);

            const BaryElementInfo& baryTriangleInfo = vBaryTriangleInfo[t];
            const sofa::defaulttype::Mat3x3d& restBase = baryTriangleInfo.restBase;
            const sofa::defaulttype::Vector3& restCenter = baryTriangleInfo.restCenter;

            sofa::defaulttype::Vec3d v = restBase * (pos - xTri);
            double d = std::max(std::max(-v[0], -v[1]), std::max((v[2]<0 ? -v[2] : v[2]) - 0.01, v[0] + v[1] - 1));
            if (d>0) d = (pos - restCenter).norm2();
            if (d<distance) { coefs = v; distance = d; index = t; }
        }

        vBaryTriangleInfo[index].vPointsIncluded.emplace_back(dirtyPoint);

        assert(dirtyPoint < vectorData.size());
        MappingData& mapData = vectorData[dirtyPoint];
        mapData.in_index = index;
        mapData.baryCoords[0] = (Real)coefs[0];
        mapData.baryCoords[1] = (Real)coefs[1];
    }

    m_dirtyPoints.clear();
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In,Out>::initTopologyChange()
{
    if (m_toContainer)
    {
        map.createTopologicalEngine(m_toContainer);
        map.registerTopologicalData();
    }

    d_vBaryTriangleInfo.createTopologicalEngine(m_fromContainer, m_vBTInfoHandler.get());
    d_vBaryTriangleInfo.registerTopologicalData();
}

template <class In, class Out>
void BarycentricMapperQuadSetTopology<In,Out>::clear ( int reserve )
{
    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    vectorData.clear(); if ( reserve>0 ) vectorData.reserve ( reserve );
    map.beginEdit();
}

template <class In, class Out>
int BarycentricMapperQuadSetTopology<In,Out>::addPointInQuad ( const int quadIndex, const SReal* baryCoords )
{
    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    vectorData.resize ( map.getValue().size() +1 );
    MappingData& data = *vectorData.rbegin();
    map.endEdit();
    data.in_index = quadIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    return map.getValue().size()-1;
}

template <class In, class Out>
int BarycentricMapperQuadSetTopology<In,Out>::createPointInQuad ( const typename Out::Coord& p, int quadIndex, const typename In::VecCoord* points )
{
    SReal baryCoords[2];
    const topology::Quad& elem = this->fromTopology->getQuad ( quadIndex );
    const typename In::CPos p0 = In::getCPos( ( *points ) [elem[0]] );
    const typename In::CPos pA = In::getCPos( ( *points ) [elem[1]] ) - p0;
    const typename In::CPos pB = In::getCPos( ( *points ) [elem[3]] ) - p0;
    typename In::CPos pos = Out::getCPos(p) - p0;
    sofa::defaulttype::Mat<3,3,typename In::Real> m,mt,base;
    m[0] = pA;
    m[1] = pB;
    m[2] = cross ( m[0],m[1] );
    mt.transpose ( m );
    base.invert ( mt );
    typename In::CPos base0 = base[0];
    typename In::CPos base1 = base[1];
    baryCoords[0] = base0 * pos;
    baryCoords[1] = base1 * pos;
    return this->addPointInQuad ( quadIndex, baryCoords );
}

template <class In, class Out>
void BarycentricMapperQuadSetTopology<In,Out>::init ( const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    _fromContainer->getContext()->get ( _fromGeomAlgo );

    int outside = 0;
    const sofa::helper::vector<topology::Quad>& quads = this->fromTopology->getQuads();

    sofa::helper::vector< sofa::defaulttype::Matrix3> bases;
    sofa::helper::vector< sofa::defaulttype::Vector3> centers;

    clear ( out.size() );
    bases.resize ( quads.size() );
    centers.resize ( quads.size() );

    for ( unsigned int c = 0; c < quads.size(); c++ )
    {
        sofa::defaulttype::Mat3x3d m,mt;
        m[0] = In::getCPos(in[quads[c][1]])-In::getCPos(in[quads[c][0]]);
        m[1] = In::getCPos(in[quads[c][3]])-In::getCPos(in[quads[c][0]]);
        m[2] = cross ( m[0],m[1] );
        mt.transpose ( m );
        bases[c].invert ( mt );
        centers[c] = ( In::getCPos(in[quads[c][0]])+In::getCPos(in[quads[c][1]])+In::getCPos(in[quads[c][2]])+In::getCPos(in[quads[c][3]]) ) *0.25;
    }

    for ( unsigned int i=0; i<out.size(); i++ )
    {
        sofa::defaulttype::Vec3d pos = Out::getCPos(out[i]);
        sofa::defaulttype::Vector3 coefs;
        int index = -1;
        double distance = 1e10;
        for ( unsigned int c = 0; c < quads.size(); c++ )
        {
            sofa::defaulttype::Vec3d v = bases[c] * ( pos - In::getCPos(in[quads[c][0]]) );
            double d = std::max ( std::max ( -v[0],-v[1] ),std::max ( std::max ( v[1]-1,v[0]-1 ),std::max ( v[2]-0.01,-v[2]-0.01 ) ) );
            if ( d>0 ) d = ( pos-centers[c] ).norm2();
            if ( d<distance ) { coefs = v; distance = d; index = c; }
        }
        if ( distance>0 )
        {
            ++outside;
        }
        addPointInQuad ( index, coefs.ptr() );
    }
}

template <class In, class Out>
void BarycentricMapperQuadSetTopology<In,Out>::initTopologyChange()
{
    if (this->toTopology)
    {
        map.createTopologicalEngine(this->toTopology);
        map.registerTopologicalData();
    }
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::clear ( int reserve )
{
    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    vectorData.clear(); if ( reserve>0 ) vectorData.reserve ( reserve );
    map.endEdit();

    helper::WriteAccessor<Data<VecBaryTetraInfo> > vBaryTetraInfo = d_vBaryTetraInfo;
    vBaryTetraInfo.clear();

    m_dirtyPoints.clear();
}

//template <class In, class Out>
//int BarycentricMapperTetrahedronSetTopology<In,Out>::createPointInTetra(const typename Out::Coord& p, int index, const typename In::VecCoord* points)
//{
//  //TODO: add implementation
//}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::init (const typename Out::VecCoord& out, const typename In::VecCoord& in)
{
    m_fromContainer->getContext()->get(m_fromGeomAlgo);

    if (m_fromContainer)
    {
        clear(out.size());
        helper::WriteAccessor<Data<VecBaryTetraInfo> > vBaryTetraInfo = d_vBaryTetraInfo;

        const sofa::helper::vector<core::topology::Tetrahedron>& tetrahedra = this->m_fromContainer->getTetrahedra();
        vBaryTetraInfo.resize(tetrahedra.size());

        sofa::helper::vector<unsigned int> ancestorsIndex;
        const sofa::helper::vector<double> ancestorsCoeff;

        for (unsigned int t = 0; t < tetrahedra.size(); t++)
        {
            m_tetraInfoHandler.applyCreateFunction(t, vBaryTetraInfo[t], tetrahedra[t], ancestorsIndex, ancestorsCoeff);
        }

        for (unsigned int i = 0; i < out.size(); ++i)
        {
            m_dirtyPoints.emplace_back(i);
        }
        projectDirtyPoints(out, in);
    }
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::initTopologyChange()
{
    if (this->toTopology)
    {
        map.createTopologicalEngine(this->toTopology, &m_vertexInfoHandler);
        map.registerTopologicalData();
    }
    d_vBaryTetraInfo.createTopologicalEngine(m_fromContainer, &m_tetraInfoHandler);
    d_vBaryTetraInfo.registerTopologicalData();
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::TetraInfoHandler::applyCreateFunction(unsigned int /*t*/,
    BaryElementInfo& baryElementInfo,
    const Tetra& tetra,
    const sofa::helper::vector<unsigned int>& ancestors,
    const sofa::helper::vector<double>& /*coeffs*/)
{
    // Get restPositions
    helper::ReadAccessor<Data<typename core::State<In>::VecCoord>> pIn = obj->m_useRestPosition ?
            obj->m_stateFrom->read(sofa::core::ConstVecCoordId::restPosition()) : obj->m_stateFrom->read(sofa::core::ConstVecCoordId::position());

    sofa::defaulttype::Mat3x3d m;
    m[0] = In::getCPos(pIn[tetra[1]]) - In::getCPos(pIn[tetra[0]]);
    m[1] = In::getCPos(pIn[tetra[2]]) - In::getCPos(pIn[tetra[0]]);
    m[2] = In::getCPos(pIn[tetra[3]]) - In::getCPos(pIn[tetra[0]]);
    sofa::defaulttype::Mat3x3d mTranspose;
    mTranspose.transpose(m);

    baryElementInfo.restBase.invert(mTranspose);
    baryElementInfo.restCenter = (In::getCPos(pIn[tetra[0]]) + In::getCPos(pIn[tetra[1]]) + In::getCPos(pIn[tetra[2]]) + In::getCPos(pIn[tetra[3]])) * 0.25;

    helper::WriteAccessor<Data<VecBaryTetraInfo>> vBaryTetraInfo = obj->d_vBaryTetraInfo;
    helper::WriteAccessor<Data<sofa::helper::vector<MappingData>>> vectorData = obj->map;
    for (unsigned int ancestor : ancestors)
    {
        assert(ancestor < vBaryTetraInfo.size());
        BaryElementInfo& baryTriangleInfoA = vBaryTetraInfo[ancestor];
        obj->m_dirtyPoints.insert(obj->m_dirtyPoints.end(), baryTriangleInfoA.vPointsIncluded.begin(), baryTriangleInfoA.vPointsIncluded.end());
        baryTriangleInfoA.vPointsIncluded.clear();

        for (auto ptId : baryTriangleInfoA.vPointsIncluded)
        {
            vectorData[ptId].in_index = -1;
        }
    }
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::TetraInfoHandler::applyDestroyFunction(unsigned int /*t*/, BaryElementInfo& baryTetraInfo)
{
    helper::WriteAccessor<Data<sofa::helper::vector<MappingData>>> vectorData = obj->map;
    for (auto ptId : baryTetraInfo.vPointsIncluded)
    {
        vectorData[ptId].in_index = -1;
    }
    obj->m_dirtyPoints.insert(obj->m_dirtyPoints.end(), baryTetraInfo.vPointsIncluded.begin(), baryTetraInfo.vPointsIncluded.end());
    baryTetraInfo.vPointsIncluded.clear();
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::TetraInfoHandler::swap(unsigned int i1, unsigned int i2)
{
    helper::WriteAccessor<Data<VecBaryTetraInfo>> vBaryTetraInfo = obj->d_vBaryTetraInfo;
    helper::WriteAccessor<Data<sofa::helper::vector<MappingData>>> vectorData = obj->map;

    BaryElementInfo& baryTetraInfo1 = vBaryTetraInfo[i1];
    BaryElementInfo& baryTetraInfo2 = vBaryTetraInfo[i2];
    for (auto pointIncluded : baryTetraInfo1.vPointsIncluded)
    {
        vectorData[pointIncluded].in_index = -1; // Not i2 because i2 might be deleted
        obj->m_dirtyPoints.emplace_back(pointIncluded);
    }
    baryTetraInfo1.vPointsIncluded.clear();

    for (auto pointIncluded : baryTetraInfo2.vPointsIncluded)
    {
        vectorData[pointIncluded].in_index = -1; // Not i1 because i1 might be deleted
        obj->m_dirtyPoints.emplace_back(pointIncluded);
    }
    baryTetraInfo2.vPointsIncluded.clear();

    component::topology::TopologyDataHandler<Tetra, VecBaryTetraInfo >::swap(i1, i2);
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::VertexInfoHandler::applyCreateFunction(unsigned int t,
    MappingData& vertexInfo,
    const Point& /*vertex*/,
    const sofa::helper::vector<unsigned int>& /*ancestors*/,
    const sofa::helper::vector<double>& /*coeffs*/)
{
    obj->m_dirtyPoints.push_back(t);
    vertexInfo.in_index = -1;
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::VertexInfoHandler::applyDestroyFunction(unsigned int t, MappingData& vertexInfo)
{
    if (vertexInfo.in_index != -1)
    {
        helper::WriteAccessor<Data<VecBaryTetraInfo> > vBaryTetraInfo = obj->d_vBaryTetraInfo;
        auto it = std::find(vBaryTetraInfo[vertexInfo.in_index].vPointsIncluded.begin(), vBaryTetraInfo[vertexInfo.in_index].vPointsIncluded.end(), t);
        if (it != vBaryTetraInfo[vertexInfo.in_index].vPointsIncluded.end())
        {
            vBaryTetraInfo[vertexInfo.in_index].vPointsIncluded.erase(it);
        }
        vertexInfo.in_index = -1;
    }

    auto it = std::find(obj->m_dirtyPoints.begin(), obj->m_dirtyPoints.end(), t);
    if (it != obj->m_dirtyPoints.end())
    {
        obj->m_dirtyPoints.erase(it);
    }
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::VertexInfoHandler::swap(unsigned int i1, unsigned int i2)
{
    helper::WriteAccessor<Data<VecBaryTetraInfo>> vBaryTetraInfo = obj->d_vBaryTetraInfo;
    helper::WriteAccessor<Data<sofa::helper::vector<MappingData>>> vectorData = obj->map;

    if (vectorData[i1].in_index != -1)
    {
        auto it = std::find(vBaryTetraInfo[vectorData[i1].in_index].vPointsIncluded.begin(), vBaryTetraInfo[vectorData[i1].in_index].vPointsIncluded.end(), i1);
        if (it != vBaryTetraInfo[vectorData[i1].in_index].vPointsIncluded.end())
        {
            vBaryTetraInfo[vectorData[i1].in_index].vPointsIncluded.erase(it);
        }
        vBaryTetraInfo[vectorData[i1].in_index].vPointsIncluded.push_back(i2);
    }

    if (vectorData[i2].in_index != -1)
    {
        auto it = std::find(vBaryTetraInfo[vectorData[i2].in_index].vPointsIncluded.begin(), vBaryTetraInfo[vectorData[i2].in_index].vPointsIncluded.end(), i2);
        if (it != vBaryTetraInfo[vectorData[i2].in_index].vPointsIncluded.end())
        {
            vBaryTetraInfo[vectorData[i2].in_index].vPointsIncluded.erase(it);
        }
        vBaryTetraInfo[vectorData[i2].in_index].vPointsIncluded.push_back(i1);
    }

    auto it = std::find(obj->m_dirtyPoints.begin(), obj->m_dirtyPoints.end(), i1);
    if (it != obj->m_dirtyPoints.end())
    {
        obj->m_dirtyPoints.erase(it);
        obj->m_dirtyPoints.push_back(i2);
    }
    it = std::find(obj->m_dirtyPoints.begin(), obj->m_dirtyPoints.end(), i2);
    if (it != obj->m_dirtyPoints.end())
    {
        obj->m_dirtyPoints.erase(it);
        obj->m_dirtyPoints.push_back(i1);
    }

    component::topology::TopologyDataHandler<Point, sofa::helper::vector<MappingData>>::swap(i1, i2);
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::projectDirtyPoints(const typename Out::VecCoord& out, const typename In::VecCoord& in)
{
    if (m_dirtyPoints.empty() || !m_fromContainer) return;

    // Get restPositions
    helper::ReadAccessor<Data<typename core::State<In>::VecCoord>> pIn = m_stateFrom->read(sofa::core::ConstVecCoordId::restPosition());
    helper::ReadAccessor<Data<typename core::State<Out>::VecCoord>> pOut = m_stateTo->read(sofa::core::ConstVecCoordId::restPosition());

    // Evaluate dirty projections : fill in d_vBaryTetraInfo and map
    helper::WriteAccessor<Data<VecBaryTetraInfo>> vBaryTetraInfo = d_vBaryTetraInfo;
    helper::WriteAccessor<Data<sofa::helper::vector<MappingData>>> vectorData = map;
    vectorData.resize(out.size());

    const sofa::helper::vector<core::topology::Tetrahedron>& tetra = this->fromTopology->getTetrahedra();

    assert(tetra.size() == vBaryTetraInfo.size());

    if (!tetra.empty())
    {
        for (auto dirtyPoint : m_dirtyPoints)
        {
            sofa::defaulttype::Vec3d pos = (m_useRestPosition && pOut.size() > 0) ? Out::getCPos(pOut[dirtyPoint]) : Out::getCPos(out[dirtyPoint]);
            sofa::defaulttype::Vector3 coefs;
            int index = -1;
            double distance = std::numeric_limits<double>::max();

            for (unsigned int t = 0; t < tetra.size(); t++)
            {
                sofa::defaulttype::Vec3d xTetra = (m_useRestPosition && pIn.size() > 0) ? In::getCPos(pIn[tetra[t][0]]) : In::getCPos(in[tetra[t][0]]);

                const BaryElementInfo& baryTetraInfo = vBaryTetraInfo[t];
                const sofa::defaulttype::Mat3x3d& restBase = baryTetraInfo.restBase;
                const sofa::defaulttype::Vector3& restCenter = baryTetraInfo.restCenter;

                sofa::defaulttype::Vec3d v = restBase * (pos - xTetra);
                double d = std::max(std::max(-v[0], -v[1]), std::max(-v[2], v[0] + v[1] + v[2] - 1));
                if (d > 0) d = (pos - restCenter).norm2();
                if (d < distance) { coefs = v; distance = d; index = t; }
            }

            vBaryTetraInfo[index].vPointsIncluded.emplace_back(dirtyPoint);

            assert(dirtyPoint < vectorData.size());
            MappingData& mapData = vectorData[dirtyPoint];
            mapData.in_index = index;
            mapData.baryCoords[0] = (Real)coefs[0];
            mapData.baryCoords[1] = (Real)coefs[1];
            mapData.baryCoords[2] = (Real)coefs[2];
        }
    }
    m_dirtyPoints.clear();
}

template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::clear ( int reserve )
{
    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    vectorData.clear();
    if ( reserve>0 ) vectorData.reserve ( reserve );
    map.endEdit();
}

template <class In, class Out>
int BarycentricMapperHexahedronSetTopology<In,Out>::addPointInCube ( const int cubeIndex, const SReal* baryCoords )
{
    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    vectorData.resize ( map.getValue().size() +1 );
    MappingData& data = *vectorData.rbegin();
    map.endEdit();
    data.in_index = cubeIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    data.baryCoords[2] = ( Real ) baryCoords[2];
    return map.getValue().size()-1;
}

template <class In, class Out>
int BarycentricMapperHexahedronSetTopology<In,Out>::setPointInCube ( const int pointIndex,
        const int cubeIndex,
        const SReal* baryCoords )
{
    if ( pointIndex >= ( int ) map.getValue().size() )
        return -1;

    helper::vector<MappingData>& vectorData = *(map.beginEdit());
    MappingData& data = vectorData[pointIndex];
    data.in_index = cubeIndex;
    data.baryCoords[0] = ( Real ) baryCoords[0];
    data.baryCoords[1] = ( Real ) baryCoords[1];
    data.baryCoords[2] = ( Real ) baryCoords[2];
    map.endEdit();

    if(cubeIndex == -1)
        _invalidIndex.insert(pointIndex);
    else
        _invalidIndex.erase(pointIndex);

    return pointIndex;
}

template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::init ( const typename Out::VecCoord& out,
        const typename In::VecCoord& /*in*/ )
{
    _fromContainer->getContext()->get ( _fromGeomAlgo );

    if ( _fromGeomAlgo == NULL )
    {
        std::cerr << "Error [BarycentricMapperHexahedronSetTopology::init] cannot find GeometryAlgorithms component." << std::endl;
    }

    if ( !map.getValue().empty() ) return;


    clear ( out.size() );

    typename In::VecCoord coord;
    helper::vector<int>   elements ( out.size() );
    helper::vector<sofa::defaulttype::Vector3> coefs ( out.size() );
    helper::vector<Real>  distances ( out.size() );

    //coord.assign(out.begin(), out.end());
    coord.resize ( out.size() );
    for ( unsigned int i=0; i<out.size(); ++i ) coord[i] = Out::getCPos(out[i]);

    _fromGeomAlgo->findNearestElementsInRestPos ( coord, elements, coefs, distances );

    for ( unsigned int i=0; i<elements.size(); ++i )
    {
        if ( elements[i] != -1 )
            addPointInCube ( elements[i], coefs[i].ptr() );
        else
            std::cerr << "Error [BarycentricMapperHexahedronSetTopology::init] cannot find a cell for barycentric mapping." << std::endl;
    }
}

template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::initTopologyChange()
{
    if (this->toTopology)
    {
        map.createTopologicalEngine(this->toTopology);
        map.registerTopologicalData();
    }
}

template <class TIn, class TOut>
void BarycentricMapping<TIn, TOut>::createMapperFromTopology ( BaseMeshTopology * topology )
{
    using sofa::core::behavior::BaseMechanicalState;

    mapper = NULL;

    topology::PointSetTopologyContainer* toTopoCont = topology::PointSetTopologyContainer::DynamicCast(this->toModel->getContext()->getActiveMeshTopology());

    core::topology::TopologyContainer* fromTopoCont = 0;
//	this->fromModel->getContext()->get(fromTopoCont);

    if (core::topology::TopologyContainer::DynamicCast(topology) != 0)
    {
        fromTopoCont = core::topology::TopologyContainer::DynamicCast(topology);
    }
    else if (topology == 0)
    {
        this->fromModel->getContext()->get(fromTopoCont);
    }
    
    if (fromTopoCont != NULL)
    {
        topology::HexahedronSetTopologyContainer* t1 = topology::HexahedronSetTopologyContainer::DynamicCast(fromTopoCont);
        if (t1 != NULL)
        {
            typedef BarycentricMapperHexahedronSetTopology<InDataTypes, OutDataTypes> HexahedronSetMapper;
            mapper = sofa::core::objectmodel::New<HexahedronSetMapper>(t1, toTopoCont);
        }
        else
        {
            topology::TetrahedronSetTopologyContainer* t2 = topology::TetrahedronSetTopologyContainer::DynamicCast(fromTopoCont);
            if (t2 != NULL)
            {
                typedef BarycentricMapperTetrahedronSetTopology<InDataTypes, OutDataTypes> TetrahedronSetMapper;
                mapper = sofa::core::objectmodel::New<TetrahedronSetMapper>(t2, toTopoCont, this->fromModel, this->toModel, useRestPosition.getValue());
            }
            else
            {
                topology::QuadSetTopologyContainer* t3 = topology::QuadSetTopologyContainer::DynamicCast(fromTopoCont);
                if (t3 != NULL)
                {
                    typedef BarycentricMapperQuadSetTopology<InDataTypes, OutDataTypes> QuadSetMapper;
                    mapper = sofa::core::objectmodel::New<QuadSetMapper>(t3, toTopoCont);
                }
                else
                {
                    topology::TriangleSetTopologyContainer* t4 = topology::TriangleSetTopologyContainer::DynamicCast(fromTopoCont);
                    if (t4 != NULL)
                    {
                        typedef BarycentricMapperTriangleSetTopology<InDataTypes, OutDataTypes> TriangleSetMapper;
                        mapper = sofa::core::objectmodel::New<TriangleSetMapper>(t4, toTopoCont, this->fromModel, this->toModel, useRestPosition.getValue());
                    }
                    else
                    {
                        topology::EdgeSetTopologyContainer* t5 = topology::EdgeSetTopologyContainer::DynamicCast(fromTopoCont);
                        if ( t5 != NULL )
                        {
                            typedef BarycentricMapperEdgeSetTopology<InDataTypes, OutDataTypes> EdgeSetMapper;
                            mapper = sofa::core::objectmodel::New<EdgeSetMapper>(t5, toTopoCont);
                        }
                    }
                }
            }
        }
    }
    else
    {
        using sofa::component::topology::RegularGridTopology;

        RegularGridTopology* rgt = RegularGridTopology::DynamicCast(topology);

        if (rgt != NULL && rgt->isVolume())
        {
            typedef BarycentricMapperRegularGridTopology< InDataTypes, OutDataTypes > RegularGridMapper;

            mapper = sofa::core::objectmodel::New<RegularGridMapper>(rgt, toTopoCont);
        }
        else
        {
            using sofa::component::topology::SparseGridTopology;

            SparseGridTopology* sgt = SparseGridTopology::DynamicCast(topology);
            if (sgt != NULL && sgt->isVolume())
            {
                typedef BarycentricMapperSparseGridTopology< InDataTypes, OutDataTypes > SparseGridMapper;
                mapper = sofa::core::objectmodel::New<SparseGridMapper>(sgt, toTopoCont);
            }
            else // generic MeshTopology
            {
                using sofa::core::topology::BaseMeshTopology;

                typedef BarycentricMapperMeshTopology< InDataTypes, OutDataTypes > MeshMapper;
                BaseMeshTopology* bmt = BaseMeshTopology::DynamicCast(topology);
                mapper = sofa::core::objectmodel::New<MeshMapper>(bmt, toTopoCont);
            }
        }
    }
    if (mapper)
    {
        mapper->setName("mapper");
        this->addSlave(mapper.get());
    }
}

template <class TIn, class TOut>
void BarycentricMapping<TIn, TOut>::init()
{
    topology_from = this->fromModel->getContext()->getActiveMeshTopology();
    topology_to = this->toModel->getContext()->getActiveMeshTopology();

    //IPB
    //core::objectmodel::BaseContext* context = this->fromModel->getContext();
    //->get(tetForceField);
    //serr << "!!!!!!!!!!!! getDT = " <<  this->fromModel->getContext()->getDt() << sendl;
    //IPE

    if ( mapper == NULL ) // try to create a mapper according to the topology of the In model
    {
        if ( topology_from!=NULL )
        {
            createMapperFromTopology ( topology_from );
        }
    }

    if ( mapper != NULL )
    {
        if (useRestPosition.getValue())
            mapper->init ( ((const core::State<Out> *)this->toModel)->read(core::ConstVecCoordId::restPosition())->getValue(), ((const core::State<In> *)this->fromModel)->read(core::ConstVecCoordId::restPosition())->getValue() );
        else
            mapper->init (((const core::State<Out> *)this->toModel)->read(core::ConstVecCoordId::position())->getValue(), ((const core::State<In> *)this->fromModel)->read(core::ConstVecCoordId::position())->getValue() );
        if (d_handleTopologyChange.getValue())
        {
            mapper->initTopologyChange();
        }
    }
    else
    {
        serr << "ERROR: Barycentric mapping does not understand topology."<<sendl;
    }

    Inherit::init();
}

template <class TIn, class TOut>
void BarycentricMapping<TIn, TOut>::reinit()
{
    if ( mapper != NULL )
    {
        mapper->clear();
        if (useRestPosition.getValue())
            mapper->init(((const core::State<Out> *)this->toModel)->read(core::ConstVecCoordId::restPosition())->getValue(), ((const core::State<In> *)this->fromModel)->read(core::ConstVecCoordId::restPosition())->getValue());
        else
            mapper->init(((const core::State<Out> *)this->toModel)->read(core::ConstVecCoordId::position())->getValue(), ((const core::State<In> *)this->fromModel)->read(core::ConstVecCoordId::position())->getValue());
        if (d_handleTopologyChange.getValue())
        {
            mapper->initTopologyChange();
        }
    }
}

template <class TIn, class TOut>
void BarycentricMapping<TIn, TOut>::apply(const core::MechanicalParams * /*mparams*/ /* PARAMS FIRST */, Data< typename Out::VecCoord >& out, const Data< typename In::VecCoord >& in)
{
    if (
#ifdef SOFA_DEV
        sleeping.getValue()==false &&
#endif
        mapper != NULL)
    {
        mapper->apply(*out.beginEdit(), in.getValue());
        out.endEdit();
    }
}

template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::apply ( typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    out.resize ( map1d.size() +map2d.size() +map3d.size() );
    const sofa::core::topology::BaseMeshTopology::SeqLines& lines = this->fromTopology->getLines();
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& triangles = this->fromTopology->getTriangles();
    const sofa::core::topology::BaseMeshTopology::SeqQuads& quads = this->fromTopology->getQuads();
    const sofa::core::topology::BaseMeshTopology::SeqTetrahedra& tetrahedra = this->fromTopology->getTetrahedra();
#ifdef SOFA_NEW_HEXA
    const sofa::core::topology::BaseMeshTopology::SeqHexahedra& cubes = this->fromTopology->getHexahedra();
#else
    const sofa::core::topology::BaseMeshTopology::SeqCubes& cubes = this->fromTopology->getCubes();
#endif
    // 1D elements
    {
        for ( unsigned int i=0; i<map1d.size(); i++ )
        {
            const Real fx = map1d[i].baryCoords[0];
            int index = map1d[i].in_index;
            {
                const sofa::core::topology::BaseMeshTopology::Line& line = lines[index];
                Out::setCPos(out[i] , In::getCPos(in[line[0]]) * ( 1-fx )
                        + In::getCPos(in[line[1]]) * fx );
            }
        }
    }
    // 2D elements
    {
        const int i0 = map1d.size();
        const int c0 = triangles.size();
        for ( unsigned int i=0; i<map2d.size(); i++ )
        {
            const Real fx = map2d[i].baryCoords[0];
            const Real fy = map2d[i].baryCoords[1];
            int index = map2d[i].in_index;
            if ( index<c0 )
            {
                const sofa::core::topology::BaseMeshTopology::Triangle& triangle = triangles[index];
                Out::setCPos(out[i+i0] , In::getCPos(in[triangle[0]]) * ( 1-fx-fy )
                        + In::getCPos(in[triangle[1]]) * fx
                        + In::getCPos(in[triangle[2]]) * fy );
            }
            else
            {
                if (quads.size())
                {
                    const sofa::core::topology::BaseMeshTopology::Quad& quad = quads[index-c0];
                    Out::setCPos(out[i+i0] , In::getCPos(in[quad[0]]) * ( ( 1-fx ) * ( 1-fy ) )
                            + In::getCPos(in[quad[1]]) * ( ( fx ) * ( 1-fy ) )
                            + In::getCPos(in[quad[3]]) * ( ( 1-fx ) * ( fy ) )
                            + In::getCPos(in[quad[2]]) * ( ( fx ) * ( fy ) ) );
                }
            }
        }
    }
    // 3D elements
    {
        const int i0 = map1d.size() + map2d.size();
        const int c0 = tetrahedra.size();
        for ( unsigned int i=0; i<map3d.size(); i++ )
        {
            const Real fx = map3d[i].baryCoords[0];
            const Real fy = map3d[i].baryCoords[1];
            const Real fz = map3d[i].baryCoords[2];
            int index = map3d[i].in_index;
            if ( index<c0 )
            {
                const sofa::core::topology::BaseMeshTopology::Tetra& tetra = tetrahedra[index];
                Out::setCPos(out[i+i0] , In::getCPos(in[tetra[0]]) * ( 1-fx-fy-fz )
                        + In::getCPos(in[tetra[1]]) * fx
                        + In::getCPos(in[tetra[2]]) * fy
                        + In::getCPos(in[tetra[3]]) * fz );
            }
            else
            {
#ifdef SOFA_NEW_HEXA
                const sofa::core::topology::BaseMeshTopology::Hexa& cube = cubes[index-c0];
#else
                const sofa::core::topology::BaseMeshTopology::Cube& cube = cubes[index-c0];
#endif
                Out::setCPos(out[i+i0] , In::getCPos(in[cube[0]]) * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) )
                        + In::getCPos(in[cube[1]]) * ( ( fx ) * ( 1-fy ) * ( 1-fz ) )
#ifdef SOFA_NEW_HEXA
                        + In::getCPos(in[cube[3]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                        + In::getCPos(in[cube[2]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#else
                        + In::getCPos(in[cube[2]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                        + In::getCPos(in[cube[3]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#endif
                        + In::getCPos(in[cube[4]]) * ( ( 1-fx ) * ( 1-fy ) * ( fz ) )
                        + In::getCPos(in[cube[5]]) * ( ( fx ) * ( 1-fy ) * ( fz ) )
#ifdef SOFA_NEW_HEXA
                        + In::getCPos(in[cube[7]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                        + In::getCPos(in[cube[6]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#else
                        + In::getCPos(in[cube[6]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                        + In::getCPos(in[cube[7]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
            }
        }
    }
}

template <class In, class Out>
void BarycentricMapperRegularGridTopology<In,Out>::apply ( typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    out.resize ( map.size() );
    for ( unsigned int i=0; i<map.size(); i++ )
    {
#ifdef SOFA_NEW_HEXA
        const topology::RegularGridTopology::Hexa cube = this->fromTopology->getHexaCopy ( this->map[i].in_index );
#else
        const topology::RegularGridTopology::Cube cube = this->fromTopology->getCubeCopy ( this->map[i].in_index );
#endif
        const Real fx = map[i].baryCoords[0];
        const Real fy = map[i].baryCoords[1];
        const Real fz = map[i].baryCoords[2];
        Out::setCPos(out[i] , In::getCPos(in[cube[0]]) * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[1]]) * ( ( fx ) * ( 1-fy ) * ( 1-fz ) )
#ifdef SOFA_NEW_HEXA
                + In::getCPos(in[cube[3]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[2]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#else
                + In::getCPos(in[cube[2]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[3]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#endif
                + In::getCPos(in[cube[4]]) * ( ( 1-fx ) * ( 1-fy ) * ( fz ) )
                + In::getCPos(in[cube[5]]) * ( ( fx ) * ( 1-fy ) * ( fz ) )
#ifdef SOFA_NEW_HEXA
                + In::getCPos(in[cube[7]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                + In::getCPos(in[cube[6]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#else
                + In::getCPos(in[cube[6]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                + In::getCPos(in[cube[7]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
    }
}

template <class In, class Out>
void BarycentricMapperSparseGridTopology<In,Out>::apply ( typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    out.resize ( map.size() );

    typedef sofa::helper::vector< CubeData > CubeDataVector;
    typedef typename CubeDataVector::const_iterator CubeDataVectorIt;

    CubeDataVectorIt it = map.begin();
    CubeDataVectorIt itEnd = map.end();

    unsigned int i = 0;

    while (it != itEnd)
    {
#ifdef SOFA_NEW_HEXA
        const topology::SparseGridTopology::Hexa cube = this->fromTopology->getHexahedron( it->in_index );
#else
        const topology::SparseGridTopology::Cube cube = this->fromTopology->getCube ( it->in_index );
#endif
        const Real fx = it->baryCoords[0];
        const Real fy = it->baryCoords[1];
        const Real fz = it->baryCoords[2];

        Out::setCPos(out[i] , In::getCPos(in[cube[0]]) * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[1]]) * ( ( fx ) * ( 1-fy ) * ( 1-fz ) )
#ifdef SOFA_NEW_HEXA
                + In::getCPos(in[cube[3]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[2]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#else
                + In::getCPos(in[cube[2]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[3]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#endif
                + In::getCPos(in[cube[4]]) * ( ( 1-fx ) * ( 1-fy ) * ( fz ) )
                + In::getCPos(in[cube[5]]) * ( ( fx ) * ( 1-fy ) * ( fz ) )
#ifdef SOFA_NEW_HEXA
                + In::getCPos(in[cube[7]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                + In::getCPos(in[cube[6]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#else
                + In::getCPos(in[cube[6]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                + In::getCPos(in[cube[7]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
        ++it;
        ++i;
    }
}

template <class In, class Out>
void BarycentricMapperEdgeSetTopology<In,Out>::apply ( typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Edge>& edges = this->fromTopology->getEdges();
    // 2D elements
    helper::vector<MappingData>& vectorData = *(map.beginEdit());

    for ( unsigned int i=0; i<map.getValue().size(); i++ )
    {
        const Real fx = vectorData[i].baryCoords[0];
        int index = vectorData[i].in_index;
        const topology::Edge& edge = edges[index];
        Out::setCPos(out[i] , In::getCPos(in[edge[0]]) * ( 1-fx )
                + In::getCPos(in[edge[1]]) * fx );
    }
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In,Out>::apply ( typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Triangle>& triangles = this->fromTopology->getTriangles();
    for ( unsigned int i=0; i<map.getValue().size(); i++ )
    {
        const Real fx = map.getValue()[i].baryCoords[0];
        const Real fy = map.getValue()[i].baryCoords[1];
        int index = map.getValue()[i].in_index;
        const topology::Triangle& triangle = triangles[index];
        Out::setCPos(out[i] , In::getCPos(in[triangle[0]]) * ( 1-fx-fy )
                + In::getCPos(in[triangle[1]]) * fx
                + In::getCPos(in[triangle[2]]) * fy );
    }
}

template <class In, class Out>
void BarycentricMapperQuadSetTopology<In,Out>::apply ( typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Quad>& quads = this->fromTopology->getQuads();
    for ( unsigned int i=0; i<map.getValue().size(); i++ )
    {
        const Real fx = map.getValue()[i].baryCoords[0];
        const Real fy = map.getValue()[i].baryCoords[1];
        int index = map.getValue()[i].in_index;
        const topology::Quad& quad = quads[index];
        Out::setCPos(out[i] , In::getCPos(in[quad[0]]) * ( ( 1-fx ) * ( 1-fy ) )
                + In::getCPos(in[quad[1]]) * ( ( fx ) * ( 1-fy ) )
                + In::getCPos(in[quad[3]]) * ( ( 1-fx ) * ( fy ) )
                + In::getCPos(in[quad[2]]) * ( ( fx ) * ( fy ) ) );
    }
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::apply ( typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    if (!m_dirtyPoints.empty())
    {
        projectDirtyPoints(out, in);
    }

    out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Tetrahedron>& tetrahedra = this->fromTopology->getTetrahedra();
    if (!tetrahedra.empty())
    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            const Real fy = map.getValue()[i].baryCoords[1];
            const Real fz = map.getValue()[i].baryCoords[2];
            int index = map.getValue()[i].in_index;
            const core::topology::BaseMeshTopology::Tetrahedron& tetra = tetrahedra[index];
            Out::setCPos(out[i] , In::getCPos(in[tetra[0]]) * ( 1-fx-fy-fz )
                    + In::getCPos(in[tetra[1]]) * fx
                    + In::getCPos(in[tetra[2]]) * fy
                    + In::getCPos(in[tetra[3]]) * fz );
        }
    }
    //serr<<"BarycentricMapperTetrahedronSetTopology<In,Out>::apply, in = "<<in<<sendl;
    //serr<<"BarycentricMapperTetrahedronSetTopology<In,Out>::apply, out = "<<out<<sendl;
}

template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::apply ( typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Hexahedron>& cubes = this->fromTopology->getHexahedra();
    for ( unsigned int i=0; i<map.getValue().size(); i++ )
    {
        const Real fx = map.getValue()[i].baryCoords[0];
        const Real fy = map.getValue()[i].baryCoords[1];
        const Real fz = map.getValue()[i].baryCoords[2];
        int index = map.getValue()[i].in_index;
        const topology::Hexahedron& cube = cubes[index];
        Out::setCPos(out[i] , In::getCPos(in[cube[0]]) * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[1]]) * ( ( fx ) * ( 1-fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[3]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[2]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
                + In::getCPos(in[cube[4]]) * ( ( 1-fx ) * ( 1-fy ) * ( fz ) )
                + In::getCPos(in[cube[5]]) * ( ( fx ) * ( 1-fy ) * ( fz ) )
                + In::getCPos(in[cube[7]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                + In::getCPos(in[cube[6]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
    }
}


//-- test mapping partiel
template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::applyOnePoint( const unsigned int& hexaPointId,typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    //out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Hexahedron>& cubes = this->fromTopology->getHexahedra();
    const Real fx = map.getValue()[hexaPointId].baryCoords[0];
    const Real fy = map.getValue()[hexaPointId].baryCoords[1];
    const Real fz = map.getValue()[hexaPointId].baryCoords[2];
    int index = map.getValue()[hexaPointId].in_index;
    const topology::Hexahedron& cube = cubes[index];
    Out::setCPos(out[hexaPointId] , In::getCPos(in[cube[0]]) * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) )
            + In::getCPos(in[cube[1]]) * ( ( fx ) * ( 1-fy ) * ( 1-fz ) )
            + In::getCPos(in[cube[3]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
            + In::getCPos(in[cube[2]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
            + In::getCPos(in[cube[4]]) * ( ( 1-fx ) * ( 1-fy ) * ( fz ) )
            + In::getCPos(in[cube[5]]) * ( ( fx ) * ( 1-fy ) * ( fz ) )
            + In::getCPos(in[cube[7]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
            + In::getCPos(in[cube[6]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
}
//--

template <class TIn, class TOut>
void BarycentricMapping<TIn, TOut>::applyJ (const core::MechanicalParams * /*mparams*/ /* PARAMS FIRST */, Data< typename Out::VecDeriv >& _out, const Data< typename In::VecDeriv >& in)
{
#ifdef SOFA_DEV
    if ( sleeping.getValue()==false)
    {
#endif
        typename Out::VecDeriv* out = _out.beginEdit();
        out->resize(this->toModel->read(core::ConstVecCoordId::position())->getValue().size());
        if (mapper != NULL)
        {
            mapper->applyJ(*out, in.getValue());
        }
        _out.endEdit();
#ifdef SOFA_DEV
    }
#endif
}

template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::applyJ ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
{

    out.resize ( map1d.size() +map2d.size() +map3d.size() );
    const sofa::core::topology::BaseMeshTopology::SeqLines& lines = this->fromTopology->getLines();
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& triangles = this->fromTopology->getTriangles();
    const sofa::core::topology::BaseMeshTopology::SeqQuads& quads = this->fromTopology->getQuads();
    const sofa::core::topology::BaseMeshTopology::SeqTetrahedra& tetrahedra = this->fromTopology->getTetrahedra();
#ifdef SOFA_NEW_HEXA
    const sofa::core::topology::BaseMeshTopology::SeqHexahedra& cubes = this->fromTopology->getHexahedra();
#else
    const sofa::core::topology::BaseMeshTopology::SeqCubes& cubes = this->fromTopology->getCubes();
#endif


    {
        // 1D elements
        {
            for ( unsigned int i=0; i<map1d.size(); i++ )
            {
                const Real fx = map1d[i].baryCoords[0];
                int index = map1d[i].in_index;
                {
                    const sofa::core::topology::BaseMeshTopology::Line& line = lines[index];
                    Out::setDPos(out[i] , In::getDPos(in[line[0]]) * ( 1-fx )
                            + In::getDPos(in[line[1]]) * fx );
                }
            }
        }
        // 2D elements
        {
            const int i0 = map1d.size();
            const int c0 = triangles.size();
            for ( unsigned int i=0; i<map2d.size(); i++ )
            {
                const Real fx = map2d[i].baryCoords[0];
                const Real fy = map2d[i].baryCoords[1];
                int index = map2d[i].in_index;
                if ( index<c0 )
                {
                    const sofa::core::topology::BaseMeshTopology::Triangle& triangle = triangles[index];
                    Out::setDPos(out[i+i0] , In::getDPos(in[triangle[0]]) * ( 1-fx-fy )
                            + In::getDPos(in[triangle[1]]) * fx
                            + In::getDPos(in[triangle[2]]) * fy );
                }
                else
                {
                    const sofa::core::topology::BaseMeshTopology::Quad& quad = quads[index-c0];
                    Out::setDPos(out[i+i0] , In::getDPos(in[quad[0]]) * ( ( 1-fx ) * ( 1-fy ) )
                            + In::getDPos(in[quad[1]]) * ( ( fx ) * ( 1-fy ) )
                            + In::getDPos(in[quad[3]]) * ( ( 1-fx ) * ( fy ) )
                            + In::getDPos(in[quad[2]]) * ( ( fx ) * ( fy ) ) );
                }
            }
        }
        // 3D elements
        {
            const int i0 = map1d.size() + map2d.size();
            const int c0 = tetrahedra.size();
            for ( unsigned int i=0; i<map3d.size(); i++ )
            {
                const Real fx = map3d[i].baryCoords[0];
                const Real fy = map3d[i].baryCoords[1];
                const Real fz = map3d[i].baryCoords[2];
                int index = map3d[i].in_index;
                if ( index<c0 )
                {
                    const sofa::core::topology::BaseMeshTopology::Tetra& tetra = tetrahedra[index];
                    Out::setDPos(out[i+i0] , In::getDPos(in[tetra[0]]) * ( 1-fx-fy-fz )
                            + In::getDPos(in[tetra[1]]) * fx
                            + In::getDPos(in[tetra[2]]) * fy
                            + In::getDPos(in[tetra[3]]) * fz );
                }
                else
                {
#ifdef SOFA_NEW_HEXA
                    const sofa::core::topology::BaseMeshTopology::Hexa& cube = cubes[index-c0];
#else
                    const sofa::core::topology::BaseMeshTopology::Cube& cube = cubes[index-c0];
#endif
                    Out::setDPos(out[i+i0] , In::getDPos(in[cube[0]]) * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) )
                            + In::getDPos(in[cube[1]]) * ( ( fx ) * ( 1-fy ) * ( 1-fz ) )
#ifdef SOFA_NEW_HEXA
                            + In::getDPos(in[cube[3]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                            + In::getDPos(in[cube[2]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#else
                            + In::getDPos(in[cube[2]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                            + In::getDPos(in[cube[3]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#endif
                            + In::getDPos(in[cube[4]]) * ( ( 1-fx ) * ( 1-fy ) * ( fz ) )
                            + In::getDPos(in[cube[5]]) * ( ( fx ) * ( 1-fy ) * ( fz ) )
#ifdef SOFA_NEW_HEXA
                            + In::getDPos(in[cube[7]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                            + In::getDPos(in[cube[6]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#else
                            + In::getDPos(in[cube[6]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                            + In::getDPos(in[cube[7]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
                }
            }
        }
    }

}

template <class In, class Out>
void BarycentricMapperRegularGridTopology<In,Out>::applyJ ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
{
    {
        for ( unsigned int i=0; i<map.size(); i++ )
        {
#ifdef SOFA_NEW_HEXA
            const topology::RegularGridTopology::Hexa cube = this->fromTopology->getHexaCopy ( this->map[i].in_index );
#else
            const topology::RegularGridTopology::Cube cube = this->fromTopology->getCubeCopy ( this->map[i].in_index );
#endif
            const Real fx = map[i].baryCoords[0];
            const Real fy = map[i].baryCoords[1];
            const Real fz = map[i].baryCoords[2];
            Out::setDPos(out[i] , In::getDPos(in[cube[0]]) * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[1]]) * ( ( fx ) * ( 1-fy ) * ( 1-fz ) )
#ifdef SOFA_NEW_HEXA
                    + In::getDPos(in[cube[3]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[2]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#else
                    + In::getDPos(in[cube[2]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[3]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#endif
                    + In::getDPos(in[cube[4]]) * ( ( 1-fx ) * ( 1-fy ) * ( fz ) )
                    + In::getDPos(in[cube[5]]) * ( ( fx ) * ( 1-fy ) * ( fz ) )
#ifdef SOFA_NEW_HEXA
                    + In::getDPos(in[cube[7]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                    + In::getDPos(in[cube[6]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#else
                    + In::getDPos(in[cube[6]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                    + In::getDPos(in[cube[7]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
        }
    }
}

template <class In, class Out>
void BarycentricMapperSparseGridTopology<In,Out>::applyJ ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
{
    {
        for ( unsigned int i=0; i<map.size(); i++ )
        {
#ifdef SOFA_NEW_HEXA
            const topology::SparseGridTopology::Hexa cube = this->fromTopology->getHexahedron ( this->map[i].in_index );
#else
            const topology::SparseGridTopology::Cube cube = this->fromTopology->getCube ( this->map[i].in_index );
#endif
            const Real fx = map[i].baryCoords[0];
            const Real fy = map[i].baryCoords[1];
            const Real fz = map[i].baryCoords[2];
            Out::setDPos(out[i] , In::getDPos(in[cube[0]]) * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[1]]) * ( ( fx ) * ( 1-fy ) * ( 1-fz ) )
#ifdef SOFA_NEW_HEXA
                    + In::getDPos(in[cube[3]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[2]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#else
                    + In::getDPos(in[cube[2]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[3]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
#endif
                    + In::getDPos(in[cube[4]]) * ( ( 1-fx ) * ( 1-fy ) * ( fz ) )
                    + In::getDPos(in[cube[5]]) * ( ( fx ) * ( 1-fy ) * ( fz ) )
#ifdef SOFA_NEW_HEXA
                    + In::getDPos(in[cube[7]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                    + In::getDPos(in[cube[6]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#else
                    + In::getDPos(in[cube[6]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                    + In::getDPos(in[cube[7]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
        }
    }
}

template <class In, class Out>
void BarycentricMapperEdgeSetTopology<In,Out>::applyJ ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
{

    out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Edge>& edges = this->fromTopology->getEdges();
    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            int index = map.getValue()[i].in_index;
            const topology::Edge& edge = edges[index];
            Out::setDPos(out[i] , In::getDPos(in[edge[0]]) * ( 1-fx )
                    + In::getDPos(in[edge[1]]) * fx );
        }
    }
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In,Out>::applyJ ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
{
    out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Triangle>& triangles = this->fromTopology->getTriangles();

    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            const Real fy = map.getValue()[i].baryCoords[1];
            int index = map.getValue()[i].in_index;
            const topology::Triangle& triangle = triangles[index];
            Out::setDPos(out[i] , In::getDPos(in[triangle[0]]) * ( 1-fx-fy )
                    + In::getDPos(in[triangle[1]]) * fx
                    + In::getDPos(in[triangle[2]]) * fy);
        }
    }
}

template <class In, class Out>
void BarycentricMapperQuadSetTopology<In,Out>::applyJ ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
{
    out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Quad>& quads = this->fromTopology->getQuads();

    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            const Real fy = map.getValue()[i].baryCoords[1];
            int index = map.getValue()[i].in_index;
            const topology::Quad& quad = quads[index];
            Out::setDPos(out[i] , In::getDPos(in[quad[0]]) * ( ( 1-fx ) * ( 1-fy ) )
                    + In::getDPos(in[quad[1]]) * ( ( fx ) * ( 1-fy ) )
                    + In::getDPos(in[quad[3]]) * ( ( 1-fx ) * ( fy ) )
                    + In::getDPos(in[quad[2]]) * ( ( fx ) * ( fy ) ));
        }
    }
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::applyJ ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
{
    out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Tetrahedron>& tetrahedra = this->fromTopology->getTetrahedra();

    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            const Real fy = map.getValue()[i].baryCoords[1];
            const Real fz = map.getValue()[i].baryCoords[2];
            int index = map.getValue()[i].in_index;
            const topology::Tetrahedron& tetra = tetrahedra[index];
            Out::setDPos(out[i] , In::getDPos(in[tetra[0]]) * ( 1-fx-fy-fz )
                    + In::getDPos(in[tetra[1]]) * fx
                    + In::getDPos(in[tetra[2]]) * fy
                    + In::getDPos(in[tetra[3]]) * fz );
        }
    }
}

template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::applyJ ( typename Out::VecDeriv& out, const typename In::VecDeriv& in )
{
   out.resize ( map.getValue().size() );
    const sofa::helper::vector<topology::Hexahedron>& cubes = this->fromTopology->getHexahedra();

    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            const Real fy = map.getValue()[i].baryCoords[1];
            const Real fz = map.getValue()[i].baryCoords[2];
            int index = map.getValue()[i].in_index;
            const topology::Hexahedron& cube = cubes[index];
            Out::setDPos(out[i] ,
                    In::getDPos(in[cube[0]]) * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[1]]) * ( ( fx ) * ( 1-fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[3]]) * ( ( 1-fx ) * ( fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[2]]) * ( ( fx ) * ( fy ) * ( 1-fz ) )
                    + In::getDPos(in[cube[4]]) * ( ( 1-fx ) * ( 1-fy ) * ( fz ) )
                    + In::getDPos(in[cube[5]]) * ( ( fx ) * ( 1-fy ) * ( fz ) )
                    + In::getDPos(in[cube[7]]) * ( ( 1-fx ) * ( fy ) * ( fz ) )
                    + In::getDPos(in[cube[6]]) * ( ( fx ) * ( fy ) * ( fz ) ) );
        }
    }
}

template <class TIn, class TOut>
void BarycentricMapping<TIn, TOut>::applyJT (const core::MechanicalParams * /*mparams*/ /* PARAMS FIRST */, Data< typename In::VecDeriv >& out, const Data< typename Out::VecDeriv >& in)
{
    if (
#ifdef SOFA_DEV
        sleeping.getValue()==false &&
#endif
        mapper != NULL)
    {
        mapper->applyJT(*out.beginEdit(), in.getValue());
        out.endEdit();
    }
}

template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::applyJT ( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
{
    const sofa::core::topology::BaseMeshTopology::SeqLines& lines = this->fromTopology->getLines();
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& triangles = this->fromTopology->getTriangles();
    const sofa::core::topology::BaseMeshTopology::SeqQuads& quads = this->fromTopology->getQuads();
    const sofa::core::topology::BaseMeshTopology::SeqTetrahedra& tetrahedra = this->fromTopology->getTetrahedra();
#ifdef SOFA_NEW_HEXA
    const sofa::core::topology::BaseMeshTopology::SeqHexahedra& cubes = this->fromTopology->getHexahedra();
#else
    const sofa::core::topology::BaseMeshTopology::SeqCubes& cubes = this->fromTopology->getCubes();
#endif

    {
        // 1D elements
        {
            for ( unsigned int i=0; i<map1d.size(); i++ )
            {
                const typename Out::DPos v = Out::getDPos(in[i]);
                const OutReal fx = ( OutReal ) map1d[i].baryCoords[0];
                int index = map1d[i].in_index;
                {
                    const sofa::core::topology::BaseMeshTopology::Line& line = lines[index];
                In::setDPos(out[line[0]], In::getDPos(out[line[0]]) +  v * ( 1-fx ) );
                In::setDPos(out[line[1]], In::getDPos(out[line[1]]) + v * fx );
                }
            }
        }
        // 2D elements
        {
            const int i0 = map1d.size();
            const int c0 = triangles.size();
            for ( unsigned int i=0; i<map2d.size(); i++ )
            {
                const typename Out::DPos v = Out::getDPos(in[i+i0]);
                const OutReal fx = ( OutReal ) map2d[i].baryCoords[0];
                const OutReal fy = ( OutReal ) map2d[i].baryCoords[1];
                int index = map2d[i].in_index;
                if ( index<c0 )
                {
                    const sofa::core::topology::BaseMeshTopology::Triangle& triangle = triangles[index];
                    In::setDPos(out[triangle[0]], In::getDPos(out[triangle[0]]) + v * ( 1-fx-fy ) );
                    In::setDPos(out[triangle[1]], In::getDPos(out[triangle[1]]) + v * fx );
                    In::setDPos(out[triangle[2]], In::getDPos(out[triangle[2]]) + v * fy );
                }
                else
                {
                    const sofa::core::topology::BaseMeshTopology::Quad& quad = quads[index-c0];
                    In::setDPos(out[quad[0]], In::getDPos(out[quad[0]]) + v * ( ( 1-fx ) * ( 1-fy ) ) );
                    In::setDPos(out[quad[1]], In::getDPos(out[quad[1]]) + v * ( ( fx ) * ( 1-fy ) ) );
                    In::setDPos(out[quad[3]], In::getDPos(out[quad[3]]) + v * ( ( 1-fx ) * ( fy ) ) );
                    In::setDPos(out[quad[2]], In::getDPos(out[quad[2]]) + v * ( ( fx ) * ( fy ) ) );
                }
            }
        }
        // 3D elements
        {
            const int i0 = map1d.size() + map2d.size();
            const int c0 = tetrahedra.size();
            for ( unsigned int i=0; i<map3d.size(); i++ )
            {
                const typename Out::DPos v = Out::getDPos(in[i+i0]);
                const OutReal fx = ( OutReal ) map3d[i].baryCoords[0];
                const OutReal fy = ( OutReal ) map3d[i].baryCoords[1];
                const OutReal fz = ( OutReal ) map3d[i].baryCoords[2];
                int index = map3d[i].in_index;
                if ( index<c0 )
                {
                    const sofa::core::topology::BaseMeshTopology::Tetra& tetra = tetrahedra[index];
                    In::setDPos(out[tetra[0]], In::getDPos(out[tetra[0]]) + v * ( 1-fx-fy-fz ) );
                    In::setDPos(out[tetra[1]], In::getDPos(out[tetra[1]]) + v * fx );
                    In::setDPos(out[tetra[2]], In::getDPos(out[tetra[2]]) + v * fy );
                    In::setDPos(out[tetra[3]], In::getDPos(out[tetra[3]]) + v * fz );
                }
                else
                {
#ifdef SOFA_NEW_HEXA
                    const sofa::core::topology::BaseMeshTopology::Hexa& cube = cubes[index-c0];
#else
                    const sofa::core::topology::BaseMeshTopology::Cube& cube = cubes[index-c0];
#endif
                    In::setDPos(out[cube[0]], In::getDPos(out[cube[0]]) + v * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) ) );
                    In::setDPos(out[cube[1]], In::getDPos(out[cube[1]]) + v * ( ( fx ) * ( 1-fy ) * ( 1-fz ) ) );
#ifdef SOFA_NEW_HEXA
                    In::setDPos(out[cube[3]], In::getDPos(out[cube[3]]) + v * ( ( 1-fx ) * ( fy ) * ( 1-fz ) ) );
                    In::setDPos(out[cube[2]], In::getDPos(out[cube[2]]) + v * ( ( fx ) * ( fy ) * ( 1-fz ) ) );
#else
                    In::setDPos(out[cube[2]], In::getDPos(out[cube[2]]) + v * ( ( 1-fx ) * ( fy ) * ( 1-fz ) ) );
                    In::setDPos(out[cube[3]], In::getDPos(out[cube[3]]) + v * ( ( fx ) * ( fy ) * ( 1-fz ) ) );
#endif
                    In::setDPos(out[cube[4]], In::getDPos(out[cube[4]]) + v * ( ( 1-fx ) * ( 1-fy ) * ( fz ) ) );
                    In::setDPos(out[cube[5]], In::getDPos(out[cube[5]]) + v * ( ( fx ) * ( 1-fy ) * ( fz ) ) );
#ifdef SOFA_NEW_HEXA
                    In::setDPos(out[cube[7]], In::getDPos(out[cube[7]]) + v * ( ( 1-fx ) * ( fy ) * ( fz ) ) );
                    In::setDPos(out[cube[6]], In::getDPos(out[cube[6]]) + v * ( ( fx ) * ( fy ) * ( fz ) ) );
#else
                    In::setDPos(out[cube[6]], In::getDPos(out[cube[6]]) + v * ( ( 1-fx ) * ( fy ) * ( fz ) ) );
                    In::setDPos(out[cube[7]], In::getDPos(out[cube[7]]) + v * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
                }
            }
        }
    }
}

template <class In, class Out>
void BarycentricMapperRegularGridTopology<In,Out>::applyJT ( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
{
    {
        for ( unsigned int i=0; i<map.size(); i++ )
        {
            const typename Out::DPos v = Out::getDPos(in[i]);
#ifdef SOFA_NEW_HEXA
            const topology::RegularGridTopology::Hexa cube = this->fromTopology->getHexaCopy ( this->map[i].in_index );
#else
            const topology::RegularGridTopology::Cube cube = this->fromTopology->getCubeCopy ( this->map[i].in_index );
#endif
            const OutReal fx = ( OutReal ) map[i].baryCoords[0];
            const OutReal fy = ( OutReal ) map[i].baryCoords[1];
            const OutReal fz = ( OutReal ) map[i].baryCoords[2];
            In::setDPos(out[cube[0]], In::getDPos(out[cube[0]]) + v * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) ));
            In::setDPos(out[cube[1]], In::getDPos(out[cube[1]]) + v * ( ( fx ) * ( 1-fy ) * ( 1-fz ) ) );
#ifdef SOFA_NEW_HEXA
            In::setDPos(out[cube[3]], In::getDPos(out[cube[3]]) + v * ( ( 1-fx ) * ( fy ) * ( 1-fz ) ) );
            In::setDPos(out[cube[2]], In::getDPos(out[cube[2]]) + v * ( ( fx ) * ( fy ) * ( 1-fz ) ) );
#else
            In::setDPos(out[cube[2]], In::getDPos(out[cube[2]]) + v * ( ( 1-fx ) * ( fy ) * ( 1-fz ) ) );
            In::setDPos(out[cube[3]], In::getDPos(out[cube[3]]) + v * ( ( fx ) * ( fy ) * ( 1-fz ) ) );
#endif
            In::setDPos(out[cube[4]], In::getDPos(out[cube[4]]) + v * ( ( 1-fx ) * ( 1-fy ) * ( fz ) ) );
            In::setDPos(out[cube[5]], In::getDPos(out[cube[5]]) + v * ( ( fx ) * ( 1-fy ) * ( fz ) ) );
#ifdef SOFA_NEW_HEXA
            In::setDPos(out[cube[7]], In::getDPos(out[cube[7]]) + v * ( ( 1-fx ) * ( fy ) * ( fz ) ) );
            In::setDPos(out[cube[6]], In::getDPos(out[cube[6]]) + v * ( ( fx ) * ( fy ) * ( fz ) ) );
#else
            In::setDPos(out[cube[6]], In::getDPos(out[cube[6]]) + v * ( ( 1-fx ) * ( fy ) * ( fz ) ) );
            In::setDPos(out[cube[7]], In::getDPos(out[cube[7]]) + v * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
        }
    }
}

template <class In, class Out>
void BarycentricMapperSparseGridTopology<In,Out>::applyJT ( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
{
    {
        for ( unsigned int i=0; i<map.size(); i++ )
        {
            const typename Out::DPos v = Out::getDPos(in[i]);
#ifdef SOFA_NEW_HEXA
            const topology::SparseGridTopology::Hexa cube = this->fromTopology->getHexahedron ( this->map[i].in_index );
#else
            const topology::SparseGridTopology::Cube cube = this->fromTopology->getCube ( this->map[i].in_index );
#endif
            const OutReal fx = ( OutReal ) map[i].baryCoords[0];
            const OutReal fy = ( OutReal ) map[i].baryCoords[1];
            const OutReal fz = ( OutReal ) map[i].baryCoords[2];
            In::setDPos(out[cube[0]], In::getDPos(out[cube[0]]) + v * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) ) );
            In::setDPos(out[cube[1]], In::getDPos(out[cube[1]]) + v * ( ( fx ) * ( 1-fy ) * ( 1-fz ) ) );
#ifdef SOFA_NEW_HEXA
            In::setDPos(out[cube[3]], In::getDPos(out[cube[3]]) + v * ( ( 1-fx ) * ( fy ) * ( 1-fz ) ) );
            In::setDPos(out[cube[2]], In::getDPos(out[cube[2]]) + v * ( ( fx ) * ( fy ) * ( 1-fz ) ) );
#else
            In::setDPos(out[cube[2]], In::getDPos(out[cube[2]]) + v * ( ( 1-fx ) * ( fy ) * ( 1-fz ) ) );
            In::setDPos(out[cube[3]], In::getDPos(out[cube[3]]) + v * ( ( fx ) * ( fy ) * ( 1-fz ) ) );
#endif
            In::setDPos(out[cube[4]], In::getDPos(out[cube[4]]) + v * ( ( 1-fx ) * ( 1-fy ) * ( fz ) ) );
            In::setDPos(out[cube[5]], In::getDPos(out[cube[5]]) + v * ( ( fx ) * ( 1-fy ) * ( fz ) ) );
#ifdef SOFA_NEW_HEXA
            In::setDPos(out[cube[7]], In::getDPos(out[cube[7]]) + v * ( ( 1-fx ) * ( fy ) * ( fz ) ) );
            In::setDPos(out[cube[6]], In::getDPos(out[cube[6]]) + v * ( ( fx ) * ( fy ) * ( fz ) ) );
#else
            In::setDPos(out[cube[6]], In::getDPos(out[cube[6]]) + v * ( ( 1-fx ) * ( fy ) * ( fz ) ) );
            In::setDPos(out[cube[7]], In::getDPos(out[cube[7]]) + v * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
        }
    }
}

template <class In, class Out>
void BarycentricMapperEdgeSetTopology<In,Out>::applyJT ( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
{
    const sofa::helper::vector<topology::Edge>& edges = this->fromTopology->getEdges();

    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const typename Out::DPos v = Out::getDPos(in[i]);
            const OutReal fx = ( OutReal ) map.getValue()[i].baryCoords[0];
            int index = map.getValue()[i].in_index;
            const topology::Edge& edge = edges[index];
            In::setDPos(out[edge[0]], In::getDPos(out[edge[0]]) + v * ( 1-fx ) );
            In::setDPos(out[edge[1]], In::getDPos(out[edge[1]]) + v * fx );
        }
    }
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In,Out>::applyJT ( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
{
    const sofa::helper::vector<topology::Triangle>& triangles = this->fromTopology->getTriangles();

    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const typename Out::DPos v = Out::getDPos(in[i]);
            const OutReal fx = ( OutReal ) map.getValue()[i].baryCoords[0];
            const OutReal fy = ( OutReal ) map.getValue()[i].baryCoords[1];
            int index = map.getValue()[i].in_index;
            const topology::Triangle& triangle = triangles[index];
            In::setDPos(out[triangle[0]], In::getDPos(out[triangle[0]]) + v * ( 1-fx-fy ) );
            In::setDPos(out[triangle[1]], In::getDPos(out[triangle[1]]) + v * fx);
            In::setDPos(out[triangle[2]], In::getDPos(out[triangle[2]]) + v * fy);
        }
    }
}

template <class In, class Out>
void BarycentricMapperQuadSetTopology<In,Out>::applyJT ( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
{
    const sofa::helper::vector<topology::Quad>& quads = this->fromTopology->getQuads();

    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const typename Out::DPos v = Out::getDPos(in[i]);
            const OutReal fx = ( OutReal ) map.getValue()[i].baryCoords[0];
            const OutReal fy = ( OutReal ) map.getValue()[i].baryCoords[1];
            int index = map.getValue()[i].in_index;
            const topology::Quad& quad = quads[index];
            In::setDPos(out[quad[0]], In::getDPos(out[quad[0]]) + v * ( ( 1-fx ) * ( 1-fy ) ) );
            In::setDPos(out[quad[1]], In::getDPos(out[quad[1]]) + v * ( ( fx ) * ( 1-fy ) ) );
            In::setDPos(out[quad[3]], In::getDPos(out[quad[3]]) + v * ( ( 1-fx ) * ( fy ) ) );
            In::setDPos(out[quad[2]], In::getDPos(out[quad[2]]) + v * ( ( fx ) * ( fy ) ) );
        }
    }
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::applyJT ( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
{
    const sofa::helper::vector<topology::Tetrahedron>& tetrahedra = this->fromTopology->getTetrahedra();

    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const typename Out::DPos v = Out::getDPos(in[i]);
            const OutReal fx = ( OutReal ) map.getValue()[i].baryCoords[0];
            const OutReal fy = ( OutReal ) map.getValue()[i].baryCoords[1];
            const OutReal fz = ( OutReal ) map.getValue()[i].baryCoords[2];
            int index = map.getValue()[i].in_index;
            const topology::Tetrahedron& tetra = tetrahedra[index];
            In::setDPos(out[tetra[0]], In::getDPos(out[tetra[0]]) + v * ( 1-fx-fy-fz ) );
            In::setDPos(out[tetra[1]], In::getDPos(out[tetra[1]]) + v * fx );
            In::setDPos(out[tetra[2]], In::getDPos(out[tetra[2]]) + v * fy );
            In::setDPos(out[tetra[3]], In::getDPos(out[tetra[3]]) + v * fz );
        }
    }
}

template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::applyJT ( typename In::VecDeriv& out, const typename Out::VecDeriv& in )
{
    const sofa::helper::vector<topology::Hexahedron>& cubes = this->fromTopology->getHexahedra();

    //////////////  DEBUG  /////////////
    // unsigned int mapSize = map.size();
    // std::cout << "Map size: " << mapSize << std::endl;
    // for(unsigned int i=0;i<map.size();i++)
    // {
    //   std::cout << "index " << map[i].in_index << ", baryCoord ( " << (OutReal)map[i].baryCoords[0] << ", " << (OutReal)map[i].baryCoords[1] << ", " << (OutReal)map[i].baryCoords[2] << ")." << std::endl;
    // }
    ////////////////////////////////////

    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const typename Out::DPos v = Out::getDPos(in[i]);
            const OutReal fx = ( OutReal ) map.getValue()[i].baryCoords[0];
            const OutReal fy = ( OutReal ) map.getValue()[i].baryCoords[1];
            const OutReal fz = ( OutReal ) map.getValue()[i].baryCoords[2];
            int index = map.getValue()[i].in_index;
            const topology::Hexahedron& cube = cubes[index];
            In::setDPos(out[cube[0]], In::getDPos(out[cube[0]]) + v * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) ) );
            In::setDPos(out[cube[1]], In::getDPos(out[cube[1]]) + v * ( ( fx ) * ( 1-fy ) * ( 1-fz ) ) ) ;
            In::setDPos(out[cube[3]], In::getDPos(out[cube[3]]) + v * ( ( 1-fx ) * ( fy ) * ( 1-fz ) ) );
            In::setDPos(out[cube[2]], In::getDPos(out[cube[2]]) + v * ( ( fx ) * ( fy ) * ( 1-fz ) ) );
            In::setDPos(out[cube[4]], In::getDPos(out[cube[4]]) + v * ( ( 1-fx ) * ( 1-fy ) * ( fz ) ) );
            In::setDPos(out[cube[5]], In::getDPos(out[cube[5]]) + v * ( ( fx ) * ( 1-fy ) * ( fz ) ) );
            In::setDPos(out[cube[7]], In::getDPos(out[cube[7]]) + v * ( ( 1-fx ) * ( fy ) * ( fz ) ) );
            In::setDPos(out[cube[6]], In::getDPos(out[cube[6]]) + v * ( ( fx ) * ( fy ) * ( fz ) ) );
        }
    }
}


template <class TIn, class TOut>
const sofa::defaulttype::BaseMatrix* BarycentricMapping<TIn, TOut>::getJ()
{
    if (
#ifdef SOFA_DEV
        sleeping.getValue()==false &&
#endif
        mapper!=NULL )
    {
        const unsigned int outStateSize = this->toModel->read(core::ConstVecCoordId::position())->getValue().size();
        const unsigned int  inStateSize = this->fromModel->read(core::ConstVecCoordId::position())->getValue().size();
        const sofa::defaulttype::BaseMatrix* matJ = mapper->getJ(outStateSize, inStateSize);

        return matJ;
    }
    else
        return NULL;
}

template <class In, class Out>
const sofa::defaulttype::BaseMatrix* BarycentricMapperMeshTopology<In,Out>::getJ(int outSize, int inSize)
{

    if (matrixJ && !updateJ && matrixJ->rowBSize() == outSize && matrixJ->colBSize() == inSize)
        return matrixJ;
    if (outSize > 0 && map1d.size()+map2d.size()+map3d.size() == 0)
        return NULL; // error: maps not yet created ?
//	std::cout << "BarycentricMapperMeshTopology: creating " << outSize << "x" << inSize << " " << NOut << "x" << NIn << " J matrix" << std::endl;
    if (!matrixJ) matrixJ = new MatrixType;
    if (matrixJ->rowBSize() != outSize || matrixJ->colBSize() != inSize)
        matrixJ->resize(outSize*NOut, inSize*NIn);
    else
        matrixJ->clear();

    const sofa::core::topology::BaseMeshTopology::SeqLines& lines = this->fromTopology->getLines();
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& triangles = this->fromTopology->getTriangles();
    const sofa::core::topology::BaseMeshTopology::SeqQuads& quads = this->fromTopology->getQuads();
    const sofa::core::topology::BaseMeshTopology::SeqTetrahedra& tetrahedra = this->fromTopology->getTetrahedra();
#ifdef SOFA_NEW_HEXA
    const sofa::core::topology::BaseMeshTopology::SeqHexahedra& cubes = this->fromTopology->getHexahedra();
#else
    const sofa::core::topology::BaseMeshTopology::SeqCubes& cubes = this->fromTopology->getCubes();
#endif

    //         std::cerr << "BarycentricMapperMeshTopology<In,Out>::getJ() \n";

    // 1D elements
    {
        for ( unsigned int i=0; i<map1d.size(); i++ )
        {
            const int out = i;
            const Real fx = ( Real ) map1d[i].baryCoords[0];
            int index = map1d[i].in_index;
            {
                const sofa::core::topology::BaseMeshTopology::Line& line = lines[index];
                this->addMatrixContrib(matrixJ, out, line[0],  ( 1-fx ));
                this->addMatrixContrib(matrixJ, out, line[1],  fx);
            }
        }
    }
    // 2D elements
    {
        const int i0 = map1d.size();
        const int c0 = triangles.size();
        for ( unsigned int i=0; i<map2d.size(); i++ )
        {
            const int out = i+i0;
            const Real fx = ( Real ) map2d[i].baryCoords[0];
            const Real fy = ( Real ) map2d[i].baryCoords[1];
            int index = map2d[i].in_index;
            if ( index<c0 )
            {
                const sofa::core::topology::BaseMeshTopology::Triangle& triangle = triangles[index];
                this->addMatrixContrib(matrixJ, out, triangle[0],  ( 1-fx-fy ));
                this->addMatrixContrib(matrixJ, out, triangle[1],  fx);
                this->addMatrixContrib(matrixJ, out, triangle[2],  fy);
            }
            else
            {
                const sofa::core::topology::BaseMeshTopology::Quad& quad = quads[index-c0];
                this->addMatrixContrib(matrixJ, out, quad[0],  ( ( 1-fx ) * ( 1-fy ) ));
                this->addMatrixContrib(matrixJ, out, quad[1],  ( ( fx ) * ( 1-fy ) ));
                this->addMatrixContrib(matrixJ, out, quad[3],  ( ( 1-fx ) * ( fy ) ));
                this->addMatrixContrib(matrixJ, out, quad[2],  ( ( fx ) * ( fy ) ));
            }
        }
    }
    // 3D elements
    {
        const int i0 = map1d.size() + map2d.size();
        const int c0 = tetrahedra.size();
        for ( unsigned int i=0; i<map3d.size(); i++ )
        {
            const int out = i+i0;
            const Real fx = ( Real ) map3d[i].baryCoords[0];
            const Real fy = ( Real ) map3d[i].baryCoords[1];
            const Real fz = ( Real ) map3d[i].baryCoords[2];
            int index = map3d[i].in_index;
            if ( index<c0 )
            {
                const sofa::core::topology::BaseMeshTopology::Tetra& tetra = tetrahedra[index];
                this->addMatrixContrib(matrixJ, out, tetra[0],  ( 1-fx-fy-fz ));
                this->addMatrixContrib(matrixJ, out, tetra[1],  fx);
                this->addMatrixContrib(matrixJ, out, tetra[2],  fy);
                this->addMatrixContrib(matrixJ, out, tetra[3],  fz);
            }
            else
            {
#ifdef SOFA_NEW_HEXA
                const sofa::core::topology::BaseMeshTopology::Hexa& cube = cubes[index-c0];
#else
                const sofa::core::topology::BaseMeshTopology::Cube& cube = cubes[index-c0];
#endif
                this->addMatrixContrib(matrixJ, out, cube[0],  ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) ));
                this->addMatrixContrib(matrixJ, out, cube[1],  ( ( fx ) * ( 1-fy ) * ( 1-fz ) ));
#ifdef SOFA_NEW_HEXA
                this->addMatrixContrib(matrixJ, out, cube[3],  ( ( 1-fx ) * ( fy ) * ( 1-fz ) ));
                this->addMatrixContrib(matrixJ, out, cube[2],  ( ( fx ) * ( fy ) * ( 1-fz ) ));
#else
                this->addMatrixContrib(matrixJ, out, cube[2],  ( ( 1-fx ) * ( fy ) * ( 1-fz ) ));
                this->addMatrixContrib(matrixJ, out, cube[3],  ( ( fx ) * ( fy ) * ( 1-fz ) ));
#endif
                this->addMatrixContrib(matrixJ, out, cube[4],  ( ( 1-fx ) * ( 1-fy ) * ( fz ) ));
                this->addMatrixContrib(matrixJ, out, cube[5],  ( ( fx ) * ( 1-fy ) * ( fz ) ));
#ifdef SOFA_NEW_HEXA
                this->addMatrixContrib(matrixJ, out, cube[7],  ( ( 1-fx ) * ( fy ) * ( fz ) ));
                this->addMatrixContrib(matrixJ, out, cube[6],  ( ( fx ) * ( fy ) * ( fz ) ));
#else
                this->addMatrixContrib(matrixJ, out, cube[6],  ( ( 1-fx ) * ( fy ) * ( fz ) ));
                this->addMatrixContrib(matrixJ, out, cube[7],  ( ( fx ) * ( fy ) * ( fz ) ));
#endif
            }
        }
    }
    matrixJ->compress();
//	std::cout << "J = " << *matrixJ << std::endl;
    updateJ = false;
    return matrixJ;
}


template <class In, class Out>
const sofa::defaulttype::BaseMatrix* BarycentricMapperRegularGridTopology<In,Out>::getJ(int outSize, int inSize)
{

    if (matrixJ && !updateJ)
        return matrixJ;

    if (!matrixJ) matrixJ = new MatrixType;
    if (matrixJ->rowBSize() != outSize || matrixJ->colBSize() != inSize)
        matrixJ->resize(outSize*NOut, inSize*NIn);
    else
        matrixJ->clear();
    //         std::cerr << "BarycentricMapperRegularGridTopology<In,Out>::getJ() \n";

    for ( unsigned int i=0; i<map.size(); i++ )
    {
        const int out = i;
#ifdef SOFA_NEW_HEXA
        const topology::RegularGridTopology::Hexa cube = this->fromTopology->getHexaCopy ( this->map[i].in_index );
#else
        const topology::RegularGridTopology::Cube cube = this->fromTopology->getCubeCopy ( this->map[i].in_index );
#endif
        const Real fx = ( Real ) map[i].baryCoords[0];
        const Real fy = ( Real ) map[i].baryCoords[1];
        const Real fz = ( Real ) map[i].baryCoords[2];
        this->addMatrixContrib(matrixJ, out, cube[0], ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[1], ( ( fx ) * ( 1-fy ) * ( 1-fz ) ));
#ifdef SOFA_NEW_HEXA
        this->addMatrixContrib(matrixJ, out, cube[3], ( ( 1-fx ) * ( fy ) * ( 1-fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[2], ( ( fx ) * ( fy ) * ( 1-fz ) ));
#else
        this->addMatrixContrib(matrixJ, out, cube[2], ( ( 1-fx ) * ( fy ) * ( 1-fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[3], ( ( fx ) * ( fy ) * ( 1-fz ) ));
#endif
        this->addMatrixContrib(matrixJ, out, cube[4], ( ( 1-fx ) * ( 1-fy ) * ( fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[5], ( ( fx ) * ( 1-fy ) * ( fz ) ));
#ifdef SOFA_NEW_HEXA
        this->addMatrixContrib(matrixJ, out, cube[7], ( ( 1-fx ) * ( fy ) * ( fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[6], ( ( fx ) * ( fy ) * ( fz ) ));
#else
        this->addMatrixContrib(matrixJ, out, cube[6], ( ( 1-fx ) * ( fy ) * ( fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[7], ( ( fx ) * ( fy ) * ( fz ) ));
#endif
    }
    updateJ = false;
    return matrixJ;
}

template <class In, class Out>
const sofa::defaulttype::BaseMatrix* BarycentricMapperSparseGridTopology<In,Out>::getJ(int outSize, int inSize)
{

    if (matrixJ && !updateJ)
        return matrixJ;

    if (!matrixJ) matrixJ = new MatrixType;
    if (matrixJ->rowBSize() != outSize || matrixJ->colBSize() != inSize)
        matrixJ->resize(outSize*NOut, inSize*NIn);
    else
        matrixJ->clear();
    //         std::cerr << "BarycentricMapperSparseGridTopology<In,Out>::getJ() \n";

    for ( unsigned int i=0; i<map.size(); i++ )
    {
        const int out = i;
#ifdef SOFA_NEW_HEXA
        const topology::SparseGridTopology::Hexa cube = this->fromTopology->getHexahedron ( this->map[i].in_index );
#else
        const topology::SparseGridTopology::Cube cube = this->fromTopology->getCube ( this->map[i].in_index );
#endif
        const Real fx = ( Real ) map[i].baryCoords[0];
        const Real fy = ( Real ) map[i].baryCoords[1];
        const Real fz = ( Real ) map[i].baryCoords[2];
        this->addMatrixContrib(matrixJ, out, cube[0], ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[1], ( ( fx ) * ( 1-fy ) * ( 1-fz ) ));
#ifdef SOFA_NEW_HEXA
        this->addMatrixContrib(matrixJ, out, cube[3], ( ( 1-fx ) * ( fy ) * ( 1-fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[2], ( ( fx ) * ( fy ) * ( 1-fz ) ));
#else
        this->addMatrixContrib(matrixJ, out, cube[2], ( ( 1-fx ) * ( fy ) * ( 1-fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[3], ( ( fx ) * ( fy ) * ( 1-fz ) ));
#endif
        this->addMatrixContrib(matrixJ, out, cube[4], ( ( 1-fx ) * ( 1-fy ) * ( fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[5], ( ( fx ) * ( 1-fy ) * ( fz ) ));
#ifdef SOFA_NEW_HEXA
        this->addMatrixContrib(matrixJ, out, cube[7], ( ( 1-fx ) * ( fy ) * ( fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[6], ( ( fx ) * ( fy ) * ( fz ) ));
#else
        this->addMatrixContrib(matrixJ, out, cube[6], ( ( 1-fx ) * ( fy ) * ( fz ) ));
        this->addMatrixContrib(matrixJ, out, cube[7], ( ( fx ) * ( fy ) * ( fz ) ));
#endif
    }
    matrixJ->compress();
//	std::cout << "J = " << *matrixJ << std::endl;
    updateJ = false;
    return matrixJ;
}

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

template <class In, class Out>
const sofa::defaulttype::BaseMatrix* BarycentricMapperEdgeSetTopology<In,Out>::getJ(int outSize, int inSize)
{
    if (matrixJ && !updateJ)
        return matrixJ;

    if (!matrixJ) matrixJ = new MatrixType;
    if (matrixJ->rowBSize() != outSize || matrixJ->colBSize() != inSize)
        matrixJ->resize(outSize*NOut, inSize*NIn);
    else
        matrixJ->clear();
    //         std::cerr << "BarycentricMapperEdgeSetTopology<In,Out>::getJ() \n";

    const sofa::helper::vector<topology::Edge>& edges = this->fromTopology->getEdges();

    {
        for ( unsigned int outId=0; outId<map.getValue().size(); outId++ )
        {
            const Real fx = map.getValue()[outId].baryCoords[0];
            int index = map.getValue()[outId].in_index;
            const topology::Edge& edge = edges[index];

            this->addMatrixContrib(matrixJ, outId, edge[0], ( 1-fx));
            this->addMatrixContrib(matrixJ, outId, edge[1], (   fx));
        }
    }
    matrixJ->compress();
//	std::cout << "BarycentricMapperEdgeSetTopology  J = " << *matrixJ << std::endl;
    updateJ = false;
    return matrixJ;
}

template <class In, class Out>
const sofa::defaulttype::BaseMatrix* BarycentricMapperTriangleSetTopology<In,Out>::getJ(int outSize, int inSize)
{
    if (matrixJ && !updateJ)
        return matrixJ;

    if (!matrixJ) matrixJ = new MatrixType;
    if (matrixJ->rowBSize() != outSize || matrixJ->colBSize() != inSize)
        matrixJ->resize(outSize*NOut, inSize*NIn);
    else
        matrixJ->clear();
    //         std::cerr << "BarycentricMapperTriangleSetTopology<In,Out>::getJ() \n";

    const sofa::helper::vector<topology::Triangle>& triangles = this->fromTopology->getTriangles();

    {
        for ( unsigned int outId=0; outId<map.getValue().size(); outId++ )
        {
            const Real fx = map.getValue()[outId].baryCoords[0];
            const Real fy = map.getValue()[outId].baryCoords[1];
            int index = map.getValue()[outId].in_index;
            const topology::Triangle& triangle = triangles[index];

            this->addMatrixContrib(matrixJ, outId, triangle[0], ( 1-fx-fy ));
            this->addMatrixContrib(matrixJ, outId, triangle[1],      ( fx ));
            this->addMatrixContrib(matrixJ, outId, triangle[2],      ( fy ));
        }
    }

    matrixJ->compress();
//	std::cout << "BarycentricMapperTriangleSetTopology  J = " << *matrixJ << std::endl;
    updateJ = false;
    return matrixJ;
}

template <class In, class Out>
const sofa::defaulttype::BaseMatrix* BarycentricMapperQuadSetTopology<In,Out>::getJ(int outSize, int inSize)
{
    if (matrixJ && !updateJ)
        return matrixJ;

    if (!matrixJ) matrixJ = new MatrixType;
    if (matrixJ->rowBSize() != outSize || matrixJ->colBSize() != inSize)
        matrixJ->resize(outSize*NOut, inSize*NIn);
    else
        matrixJ->clear();
    //         std::cerr << "BarycentricMapperQuadSetTopology<In,Out>::getJ() \n";

    const sofa::helper::vector<topology::Quad>& quads = this->fromTopology->getQuads();


    {
        for ( unsigned int outId=0; outId<map.getValue().size(); outId++ )
        {
            const Real fx = map.getValue()[outId].baryCoords[0];
            const Real fy = map.getValue()[outId].baryCoords[1];
            int index = map.getValue()[outId].in_index;
            const topology::Quad& quad = quads[index];

            this->addMatrixContrib(matrixJ, outId, quad[0], ( ( 1-fx ) * ( 1-fy ) ));
            this->addMatrixContrib(matrixJ, outId, quad[1], (   ( fx ) * ( 1-fy ) ));
            this->addMatrixContrib(matrixJ, outId, quad[2], (   ( fx ) *   ( fy ) ));
            this->addMatrixContrib(matrixJ, outId, quad[3], ( ( 1-fx ) *   ( fy ) ));
        }
    }
    matrixJ->compress();
//	std::cout << "BarycentricMapperQuadSetTopology  J = " << std::endl<< *matrixJ << std::endl;
    updateJ = false;
    return matrixJ;
}

template <class In, class Out>
const sofa::defaulttype::BaseMatrix* BarycentricMapperTetrahedronSetTopology<In,Out>::getJ(int outSize, int inSize)
{

    if (matrixJ && !updateJ)
        return matrixJ;

    if (!matrixJ) matrixJ = new MatrixType;
    if (matrixJ->rowBSize() != outSize || matrixJ->colBSize() != inSize)
        matrixJ->resize(outSize*NOut, inSize*NIn);
    else
        matrixJ->clear();

    const sofa::helper::vector<topology::Tetrahedron>& tetrahedra = this->fromTopology->getTetrahedra();

    {
        for ( unsigned int outId=0; outId<map.getValue().size(); outId++ )
        {
            const Real fx = map.getValue()[outId].baryCoords[0];
            const Real fy = map.getValue()[outId].baryCoords[1];
            const Real fz = map.getValue()[outId].baryCoords[2];
            int index = map.getValue()[outId].in_index;
            const topology::Tetrahedron& tetra = tetrahedra[index];

            this->addMatrixContrib(matrixJ, outId, tetra[0], ( 1-fx-fy-fz ));
            this->addMatrixContrib(matrixJ, outId, tetra[1],         ( fx ));
            this->addMatrixContrib(matrixJ, outId, tetra[2],         ( fy ));
            this->addMatrixContrib(matrixJ, outId, tetra[3],         ( fz ));

        }
    }
    matrixJ->compress();
    updateJ = false;
    return matrixJ;
}

template <class In, class Out>
const sofa::defaulttype::BaseMatrix* BarycentricMapperHexahedronSetTopology<In,Out>::getJ(int outSize, int inSize)
{
    if (matrixJ && !updateJ)
        return matrixJ;

    if (!matrixJ) matrixJ = new MatrixType;
    if (matrixJ->rowBSize() != outSize || matrixJ->colBSize() != inSize)
        matrixJ->resize(outSize*NOut, inSize*NIn);
    else
        matrixJ->clear();

    const sofa::helper::vector<topology::Hexahedron>& cubes = this->fromTopology->getHexahedra();

    {
        for ( unsigned int outId=0; outId<map.getValue().size(); outId++ )
        {
            const Real fx = map.getValue()[outId].baryCoords[0];
            const Real fy = map.getValue()[outId].baryCoords[1];
            const Real fz = map.getValue()[outId].baryCoords[2];
            int index = map.getValue()[outId].in_index;
            const topology::Hexahedron& cube = cubes[index];

            this->addMatrixContrib(matrixJ, outId, cube[0], ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) ));
            this->addMatrixContrib(matrixJ, outId, cube[1], (   ( fx ) * ( 1-fy ) * ( 1-fz ) ));
            this->addMatrixContrib(matrixJ, outId, cube[2], (   ( fx ) *   ( fy ) * ( 1-fz ) ));
            this->addMatrixContrib(matrixJ, outId, cube[3], ( ( 1-fx ) *   ( fy ) * ( 1-fz ) ));
            this->addMatrixContrib(matrixJ, outId, cube[4], ( ( 1-fx ) * ( 1-fy ) *   ( fz ) ));
            this->addMatrixContrib(matrixJ, outId, cube[5], (   ( fx ) * ( 1-fy ) *   ( fz ) ));
            this->addMatrixContrib(matrixJ, outId, cube[6], (   ( fx ) *   ( fy ) *   ( fz ) ));
            this->addMatrixContrib(matrixJ, outId, cube[7], ( ( 1-fx ) *   ( fy ) *   ( fz ) ));
        }
    }
    matrixJ->compress();
    updateJ = false;
    return matrixJ;
}


///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////






template <class TIn, class TOut>
void BarycentricMapping<TIn, TOut>::draw(const core::visual::VisualParams* vparams)
{
    if ( !vparams->displayFlags().getShowMappings() ) return;

    const OutVecCoord& out = this->toModel->read(core::ConstVecCoordId::position())->getValue();
    sofa::helper::vector< sofa::defaulttype::Vector3 > points;
    for ( unsigned int i=0; i<out.size(); i++ )
    {
        points.push_back ( OutDataTypes::getCPos(out[i]) );
    }
//	glEnd();
    const InVecCoord& in = this->fromModel->read(core::ConstVecCoordId::position())->getValue();
    if ( mapper!=NULL ) mapper->draw(vparams,out, in );

    vparams->drawTool()->drawPoints ( points, 7, sofa::defaulttype::Vec<4,float> ( 1,1,0,1 ) );
}

template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::draw  (const core::visual::VisualParams* vparams,const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    const sofa::core::topology::BaseMeshTopology::SeqLines& lines = this->fromTopology->getLines();
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& triangles = this->fromTopology->getTriangles();
    const sofa::core::topology::BaseMeshTopology::SeqQuads& quads = this->fromTopology->getQuads();
    const sofa::core::topology::BaseMeshTopology::SeqTetrahedra& tetrahedra = this->fromTopology->getTetrahedra();
#ifdef SOFA_NEW_HEXA
    const sofa::core::topology::BaseMeshTopology::SeqHexahedra& cubes = this->fromTopology->getHexahedra();
#else
    const sofa::core::topology::BaseMeshTopology::SeqCubes& cubes = this->fromTopology->getCubes();
#endif
    sofa::helper::vector< sofa::defaulttype::Vector3 > points;
    // 1D elements
    {
        const int i0 = 0;
        for ( unsigned int i=0; i<map1d.size(); i++ )
        {
            const Real fx = map1d[i].baryCoords[0];
            int index = map1d[i].in_index;
            {
                const sofa::core::topology::BaseMeshTopology::Line& line = lines[index];
                Real f[2];
                f[0] = ( 1-fx );
                f[1] = fx;
                for ( int j=0; j<2; j++ )
                {
                    if ( f[j]<=-0.0001 || f[j]>=0.0001 )
                    {
                        //                         glColor3f((float)f[j],1,(float)f[j]);
                        points.push_back ( Out::getCPos(out[i+i0]) );
                        points.push_back ( In::getCPos(in[line[j]]) );
                    }
                }
            }
        }
    }
    // 2D elements
    {
        const int i0 = map1d.size();
        const int c0 = triangles.size();
        for ( unsigned int i=0; i<map2d.size(); i++ )
        {
            const Real fx = map2d[i].baryCoords[0];
            const Real fy = map2d[i].baryCoords[1];
            int index = map2d[i].in_index;
            if ( index<c0 )
            {
                const sofa::core::topology::BaseMeshTopology::Triangle& triangle = triangles[index];
                Real f[3];
                f[0] = ( 1-fx-fy );
                f[1] = fx;
                f[2] = fy;
                for ( int j=0; j<3; j++ )
                {
                    if ( f[j]<=-0.0001 || f[j]>=0.0001 )
                    {
                        //                         glColor3f((float)f[j],1,(float)f[j]);
                        points.push_back ( Out::getCPos(out[i+i0]) );
                        points.push_back ( In::getCPos(in[triangle[j]]) );
                    }
                }
            }
            else
            {
                const sofa::core::topology::BaseMeshTopology::Quad& quad = quads[index-c0];
                Real f[4];
                f[0] = ( ( 1-fx ) * ( 1-fy ) );
                f[1] = ( ( fx ) * ( 1-fy ) );
                f[3] = ( ( 1-fx ) * ( fy ) );
                f[2] = ( ( fx ) * ( fy ) );
                for ( int j=0; j<4; j++ )
                {
                    if ( f[j]<=-0.0001 || f[j]>=0.0001 )
                    {
                        //                         glColor3f((float)f[j],1,(float)f[j]);
                        points.push_back ( Out::getCPos(out[i+i0]) );
                        points.push_back ( In::getCPos(in[quad[j]]) );
                    }
                }
            }
        }
    }
    // 3D elements
    {
        const int i0 = map1d.size() +map2d.size();
        const int c0 = tetrahedra.size();
        for ( unsigned int i=0; i<map3d.size(); i++ )
        {
            const Real fx = map3d[i].baryCoords[0];
            const Real fy = map3d[i].baryCoords[1];
            const Real fz = map3d[i].baryCoords[2];
            int index = map3d[i].in_index;
            if ( index<c0 )
            {
                const sofa::core::topology::BaseMeshTopology::Tetra& tetra = tetrahedra[index];
                Real f[4];
                f[0] = ( 1-fx-fy-fz );
                f[1] = fx;
                f[2] = fy;
                f[3] = fz;
                for ( int j=0; j<4; j++ )
                {
                    if ( f[j]<=-0.0001 || f[j]>=0.0001 )
                    {
                        //                         glColor3f((float)f[j],1,(float)f[j]);
                        points.push_back ( Out::getCPos(out[i+i0]) );
                        points.push_back ( In::getCPos(in[tetra[j]]) );
                    }
                }
            }
            else
            {
#ifdef SOFA_NEW_HEXA
                const sofa::core::topology::BaseMeshTopology::Hexa& cube = cubes[index-c0];
#else
                const sofa::core::topology::BaseMeshTopology::Cube& cube = cubes[index-c0];
#endif
                Real f[8];
                f[0] = ( 1-fx ) * ( 1-fy ) * ( 1-fz );
                f[1] = ( fx ) * ( 1-fy ) * ( 1-fz );
#ifdef SOFA_NEW_HEXA
                f[3] = ( 1-fx ) * ( fy ) * ( 1-fz );
                f[2] = ( fx ) * ( fy ) * ( 1-fz );
#else
                f[2] = ( 1-fx ) * ( fy ) * ( 1-fz );
                f[3] = ( fx ) * ( fy ) * ( 1-fz );
#endif
                f[4] = ( 1-fx ) * ( 1-fy ) * ( fz );
                f[5] = ( fx ) * ( 1-fy ) * ( fz );
#ifdef SOFA_NEW_HEXA
                f[7] = ( 1-fx ) * ( fy ) * ( fz );
                f[6] = ( fx ) * ( fy ) * ( fz );
#else
                f[6] = ( 1-fx ) * ( fy ) * ( fz );
                f[7] = ( fx ) * ( fy ) * ( fz );
#endif
                for ( int j=0; j<8; j++ )
                {
                    if ( f[j]<=-0.0001 || f[j]>=0.0001 )
                    {
                        //                         glColor3f((float)f[j],1,1);
                        points.push_back ( Out::getCPos(out[i+i0]) );
                        points.push_back ( In::getCPos(in[cube[j]]) );
                    }
                }
            }
        }
    }
    vparams->drawTool()->drawLines ( points, 1, sofa::defaulttype::Vec<4,float> ( 0,1,0,1 ) );
}

template <class In, class Out>
void BarycentricMapperRegularGridTopology<In,Out>::draw  (const core::visual::VisualParams* vparams,const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    sofa::helper::vector< sofa::defaulttype::Vector3 > points;

    for ( unsigned int i=0; i<map.size(); i++ )
    {
#ifdef SOFA_NEW_HEXA
        const topology::RegularGridTopology::Hexa cube = this->fromTopology->getHexaCopy ( this->map[i].in_index );
#else
        const topology::RegularGridTopology::Cube cube = this->fromTopology->getCubeCopy ( this->map[i].in_index );
#endif
        const Real fx = map[i].baryCoords[0];
        const Real fy = map[i].baryCoords[1];
        const Real fz = map[i].baryCoords[2];
        Real f[8];
        f[0] = ( 1-fx ) * ( 1-fy ) * ( 1-fz );
        f[1] = ( fx ) * ( 1-fy ) * ( 1-fz );
#ifdef SOFA_NEW_HEXA
        f[3] = ( 1-fx ) * ( fy ) * ( 1-fz );
        f[2] = ( fx ) * ( fy ) * ( 1-fz );
#else
        f[2] = ( 1-fx ) * ( fy ) * ( 1-fz );
        f[3] = ( fx ) * ( fy ) * ( 1-fz );
#endif
        f[4] = ( 1-fx ) * ( 1-fy ) * ( fz );
        f[5] = ( fx ) * ( 1-fy ) * ( fz );
#ifdef SOFA_NEW_HEXA
        f[7] = ( 1-fx ) * ( fy ) * ( fz );
        f[6] = ( fx ) * ( fy ) * ( fz );
#else
        f[6] = ( 1-fx ) * ( fy ) * ( fz );
        f[7] = ( fx ) * ( fy ) * ( fz );
#endif
        for ( int j=0; j<8; j++ )
        {
            if ( f[j]<=-0.0001 || f[j]>=0.0001 )
            {
                //glColor3f((float)f[j],(float)f[j],1);
                points.push_back ( Out::getCPos(out[i]) );
                points.push_back ( In::getCPos(in[cube[j]]) );
            }
        }
    }
    vparams->drawTool()->drawLines ( points, 1, sofa::defaulttype::Vec<4,float> ( 0,0,1,1 ) );

}

template <class In, class Out>
void BarycentricMapperSparseGridTopology<In,Out>::draw  (const core::visual::VisualParams* vparams,const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    sofa::helper::vector< sofa::defaulttype::Vector3 > points;
    for ( unsigned int i=0; i<map.size(); i++ )
    {
#ifdef SOFA_NEW_HEXA
        const topology::SparseGridTopology::Hexa cube = this->fromTopology->getHexahedron ( this->map[i].in_index );
#else
        const topology::SparseGridTopology::Cube cube = this->fromTopology->getCube ( this->map[i].in_index );
#endif
        const Real fx = map[i].baryCoords[0];
        const Real fy = map[i].baryCoords[1];
        const Real fz = map[i].baryCoords[2];
        Real f[8];
        f[0] = ( 1-fx ) * ( 1-fy ) * ( 1-fz );
        f[1] = ( fx ) * ( 1-fy ) * ( 1-fz );
#ifdef SOFA_NEW_HEXA
        f[3] = ( 1-fx ) * ( fy ) * ( 1-fz );
        f[2] = ( fx ) * ( fy ) * ( 1-fz );
#else
        f[2] = ( 1-fx ) * ( fy ) * ( 1-fz );
        f[3] = ( fx ) * ( fy ) * ( 1-fz );
#endif
        f[4] = ( 1-fx ) * ( 1-fy ) * ( fz );
        f[5] = ( fx ) * ( 1-fy ) * ( fz );
#ifdef SOFA_NEW_HEXA
        f[7] = ( 1-fx ) * ( fy ) * ( fz );
        f[6] = ( fx ) * ( fy ) * ( fz );
#else
        f[6] = ( 1-fx ) * ( fy ) * ( fz );
        f[7] = ( fx ) * ( fy ) * ( fz );
#endif
        for ( int j=0; j<8; j++ )
        {
            if ( f[j]<=-0.0001 || f[j]>=0.0001 )
            {
                //glColor3f((float)f[j],(float)f[j],1);
                points.push_back ( Out::getCPos(out[i]) );
                points.push_back ( In::getCPos(in[cube[j]]) );
            }
        }
    }
    vparams->drawTool()->drawLines ( points, 1, sofa::defaulttype::Vec<4,float> ( 0,0,1,1 ) );

}

template <class In, class Out>
void BarycentricMapperEdgeSetTopology<In,Out>::draw  (const core::visual::VisualParams* vparams,const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    const sofa::helper::vector<topology::Edge>& edges = this->fromTopology->getEdges();

    sofa::helper::vector< sofa::defaulttype::Vector3 > points;
    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            int index = map.getValue()[i].in_index;
            const topology::Edge& edge = edges[index];
            {
                const Real f = Real ( 1.0 )-fx;
                if ( f<=-0.0001 || f>=0.0001 )
                {
                    //                     glColor3f((float)f,1,(float)f);
                    points.push_back ( Out::getCPos(out[i]) );
                    points.push_back ( In::getCPos(in[edge[0]]) );
                }
            }
            {
                const Real f = fx;
                if ( f<=-0.0001 || f>=0.0001 )
                {
                    //                     glColor3f((float)f,1,(float)f);
                    points.push_back ( Out::getCPos(out[i]) );
                    points.push_back ( In::getCPos(in[edge[1]]) );
                }
            }
        }
    }
    vparams->drawTool()->drawLines ( points, 1, sofa::defaulttype::Vec<4,float> ( 0,1,0,1 ) );
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In,Out>::draw  (const core::visual::VisualParams* vparams,const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    const sofa::helper::vector<topology::Triangle>& triangles = this->fromTopology->getTriangles();

    sofa::helper::vector< sofa::defaulttype::Vector3 > points;
    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            const Real fy = map.getValue()[i].baryCoords[1];
            int index = map.getValue()[i].in_index;
            const topology::Triangle& triangle = triangles[index];
            Real f[3];
            f[0] = ( 1-fx-fy );
            f[1] = fx;
            f[2] = fy;
            for ( int j=0; j<3; j++ )
            {
                if ( f[j]<=-0.0001 || f[j]>=0.0001 )
                {
                    //                     glColor3f((float)f[j],1,(float)f[j]);
                    points.push_back ( Out::getCPos(out[i]) );
                    points.push_back ( In::getCPos(in[triangle[j]]) );
                }
            }
        }
    }
    vparams->drawTool()->drawLines ( points, 1, sofa::defaulttype::Vec<4,float> ( 0,1,0,1 ) );
}

template <class In, class Out>
void BarycentricMapperQuadSetTopology<In,Out>::draw  (const core::visual::VisualParams* vparams,const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    const sofa::helper::vector<topology::Quad>& quads = this->fromTopology->getQuads();
    sofa::helper::vector< sofa::defaulttype::Vector3 > points;
    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            const Real fy = map.getValue()[i].baryCoords[1];
            int index = map.getValue()[i].in_index;
            const topology::Quad& quad = quads[index];
            Real f[4];
            f[0] = ( ( 1-fx ) * ( 1-fy ) );
            f[1] = ( ( fx ) * ( 1-fy ) );
            f[3] = ( ( 1-fx ) * ( fy ) );
            f[2] = ( ( fx ) * ( fy ) );
            for ( int j=0; j<4; j++ )
            {
                if ( f[j]<=-0.0001 || f[j]>=0.0001 )
                {
                    //                     glColor3f((float)f[j],1,(float)f[j]);
                    points.push_back ( Out::getCPos(out[i]) );
                    points.push_back ( In::getCPos(in[quad[j]]) );
                }
            }
        }
    }
    vparams->drawTool()->drawLines ( points, 1, sofa::defaulttype::Vec<4,float> ( 0,1,0,1 ) );
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::draw  (const core::visual::VisualParams* vparams,const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    const sofa::helper::vector<topology::Tetrahedron>& tetrahedra = this->fromTopology->getTetrahedra();

    sofa::helper::vector< sofa::defaulttype::Vector3 > points;
    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            const Real fy = map.getValue()[i].baryCoords[1];
            const Real fz = map.getValue()[i].baryCoords[2];
            int index = map.getValue()[i].in_index;
            const topology::Tetrahedron& tetra = tetrahedra[index];
            Real f[4];
            f[0] = ( 1-fx-fy-fz );
            f[1] = fx;
            f[2] = fy;
            f[3] = fz;
            for ( int j=0; j<4; j++ )
            {
                if ( f[j]<=-0.0001 || f[j]>=0.0001 )
                {
                    //                     glColor3f((float)f[j],1,(float)f[j]);
                    points.push_back ( Out::getCPos(out[i]) );
                    points.push_back ( In::getCPos(in[tetra[j]]) );
                }
            }
        }
    }
    vparams->drawTool()->drawLines ( points, 1, sofa::defaulttype::Vec<4,float> ( 0,1,0,1 ) );
}

template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::draw  (const core::visual::VisualParams* vparams,const typename Out::VecCoord& out, const typename In::VecCoord& in )
{
    const sofa::helper::vector<topology::Hexahedron>& cubes = this->fromTopology->getHexahedra();

    sofa::helper::vector< sofa::defaulttype::Vector3 > points;
    {
        for ( unsigned int i=0; i<map.getValue().size(); i++ )
        {
            const Real fx = map.getValue()[i].baryCoords[0];
            const Real fy = map.getValue()[i].baryCoords[1];
            const Real fz = map.getValue()[i].baryCoords[2];
            int index = map.getValue()[i].in_index;
            const topology::Hexahedron& cube = cubes[index];
            Real f[8];
            f[0] = ( 1-fx ) * ( 1-fy ) * ( 1-fz );
            f[1] = ( fx ) * ( 1-fy ) * ( 1-fz );
            f[3] = ( 1-fx ) * ( fy ) * ( 1-fz );
            f[2] = ( fx ) * ( fy ) * ( 1-fz );
            f[4] = ( 1-fx ) * ( 1-fy ) * ( fz );
            f[5] = ( fx ) * ( 1-fy ) * ( fz );
            f[7] = ( 1-fx ) * ( fy ) * ( fz );
            f[6] = ( fx ) * ( fy ) * ( fz );
            for ( int j=0; j<8; j++ )
            {
                if ( f[j]<=-0.0001 || f[j]>=0.0001 )
                {
                    //                     glColor3f((float)f[j],1,1);
                    points.push_back ( Out::getCPos(out[i]) );
                    points.push_back ( In::getCPos(in[cube[j]]) );
                }
            }
        }
    }
    vparams->drawTool()->drawLines ( points, 1, sofa::defaulttype::Vec<4,float> ( 0,1,0,1 ) );
}

/************************************* PropagateConstraint ***********************************/


template <class TIn, class TOut>
void BarycentricMapping<TIn, TOut>::applyJT(const core::ConstraintParams * /*cparams*/ /* PARAMS FIRST */, Data< typename In::MatrixDeriv >& out, const Data< typename Out::MatrixDeriv >& in)
{
    if (
#ifdef SOFA_DEV
        sleeping.getValue()==false &&
#endif
        mapper!=NULL )
    {
        mapper->applyJT(*out.beginEdit(), in.getValue());
        out.endEdit();
    }
}


/// @todo Optimization
template <class In, class Out>
void BarycentricMapperMeshTopology<In,Out>::applyJT ( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in )
{
    const sofa::core::topology::BaseMeshTopology::SeqLines& lines = this->fromTopology->getLines();
    const sofa::core::topology::BaseMeshTopology::SeqTriangles& triangles = this->fromTopology->getTriangles();
    const sofa::core::topology::BaseMeshTopology::SeqQuads& quads = this->fromTopology->getQuads();
    const sofa::core::topology::BaseMeshTopology::SeqTetrahedra& tetrahedra = this->fromTopology->getTetrahedra();
#ifdef SOFA_NEW_HEXA
    const sofa::core::topology::BaseMeshTopology::SeqHexahedra& cubes = this->fromTopology->getHexahedra();
#else
    const sofa::core::topology::BaseMeshTopology::SeqCubes& cubes = this->fromTopology->getCubes();
#endif
    //const int iLine = lines.size();
    const int iTri = triangles.size();
    //const int iQuad = quads.size();
    const int iTetra= tetrahedra.size();
    //const int iCube = cubes.size();

    const int i1d = map1d.size();
    const int i2d = map2d.size();
    const int i3d = map3d.size();

    int indexIn;

    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();

        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            for ( ; colIt != colItEnd; ++colIt)
            {
                indexIn = colIt.index();
                InDeriv data;
                In::setDPos(data, Out::getDPos(colIt.val()));

                // 1D elements
                if ( indexIn < i1d )
                {
                    const OutReal fx = ( OutReal ) map1d[indexIn].baryCoords[0];
                    int index = map1d[indexIn].in_index;
                    {
                        const sofa::core::topology::BaseMeshTopology::Line& line = lines[index];
                        o.addCol( line[0], data * ( 1-fx ) );
                        o.addCol( line[1], data * fx );
                    }
                }
                // 2D elements : triangle or quad
                else if ( indexIn < i2d )
                {
                    const OutReal fx = ( OutReal ) map2d[indexIn].baryCoords[0];
                    const OutReal fy = ( OutReal ) map2d[indexIn].baryCoords[1];
                    int index = map2d[indexIn].in_index;
                    if ( index < iTri ) // triangle
                    {
                        const sofa::core::topology::BaseMeshTopology::Triangle& triangle = triangles[index];
                        o.addCol( triangle[0], data * ( 1-fx-fy ) );
                        o.addCol( triangle[1], data * fx );
                        o.addCol( triangle[2], data * fy );
                    }
                    else // 2D element : Quad
                    {
                        const sofa::core::topology::BaseMeshTopology::Quad& quad = quads[index - iTri];
                        o.addCol( quad[0], data * ( ( 1-fx ) * ( 1-fy ) ) );
                        o.addCol( quad[1], data * ( ( fx ) * ( 1-fy ) ) );
                        o.addCol( quad[3], data * ( ( 1-fx ) * ( fy ) ) );
                        o.addCol( quad[2], data * ( ( fx ) * ( fy ) ) );
                    }
                }
                // 3D elements
                else if ( indexIn < i3d )
                {
                    const OutReal fx = ( OutReal ) map3d[indexIn].baryCoords[0];
                    const OutReal fy = ( OutReal ) map3d[indexIn].baryCoords[1];
                    const OutReal fz = ( OutReal ) map3d[indexIn].baryCoords[2];
                    int index = map3d[indexIn].in_index;
                    if ( index < iTetra ) // tetra
                    {
                        const sofa::core::topology::BaseMeshTopology::Tetra& tetra = tetrahedra[index];
                        o.addCol ( tetra[0], data * ( 1-fx-fy-fz ) );
                        o.addCol ( tetra[1], data * fx );
                        o.addCol ( tetra[2], data * fy );
                        o.addCol ( tetra[3], data * fz );
                    }
                    else // cube
                    {
#ifdef SOFA_NEW_HEXA
                        const sofa::core::topology::BaseMeshTopology::Hexa& cube = cubes[index-iTetra];
#else
                        const sofa::core::topology::BaseMeshTopology::Cube& cube = cubes[index-iTetra];
#endif
                        o.addCol ( cube[0],data * ( ( 1-fx ) * ( 1-fy ) * ( 1-fz ) ) ) ;
                        o.addCol ( cube[1],data * ( ( fx ) * ( 1-fy ) * ( 1-fz ) ) ) ;
#ifdef SOFA_NEW_HEXA
                        o.addCol ( cube[3],data * ( ( 1-fx ) * ( fy ) * ( 1-fz ) ) ) ;
                        o.addCol ( cube[2],data * ( ( fx ) * ( fy ) * ( 1-fz ) ) ) ;
#else
                        o.addCol ( cube[2],data * ( ( 1-fx ) * ( fy ) * ( 1-fz ) ) ) ;
                        o.addCol ( cube[3],data * ( ( fx ) * ( fy ) * ( 1-fz ) ) ) ;
#endif
                        o.addCol ( cube[4],data * ( ( 1-fx ) * ( 1-fy ) * ( fz ) ) ) ;
                        o.addCol ( cube[5],data * ( ( fx ) * ( 1-fy ) * ( fz ) ) ) ;
#ifdef SOFA_NEW_HEXA
                        o.addCol ( cube[7],data * ( ( 1-fx ) * ( fy ) * ( fz ) ) ) ;
                        o.addCol ( cube[6],data * ( ( fx ) * ( fy ) * ( fz ) ) ) ;
#else
                        o.addCol ( cube[6],data * ( ( 1-fx ) * ( fy ) * ( fz ) ) ) ;
                        o.addCol ( cube[7],data * ( ( fx ) * ( fy ) * ( fz ) ) );
#endif
                    }
                }
            }
        }
    }
}

template <class In, class Out>
void BarycentricMapperRegularGridTopology<In,Out>::applyJT ( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in )
{
    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();

        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            for ( ; colIt != colItEnd; ++colIt)
            {
                unsigned int indexIn = colIt.index();
                InDeriv data;
                In::setDPos(data, Out::getDPos(colIt.val()));

#ifdef SOFA_NEW_HEXA
                const topology::RegularGridTopology::Hexa cube = this->fromTopology->getHexaCopy ( this->map[indexIn].in_index );
#else
                const topology::RegularGridTopology::Cube cube = this->fromTopology->getCubeCopy ( this->map[indexIn].in_index );
#endif
                const OutReal fx = (OutReal) map[indexIn].baryCoords[0];
                const OutReal fy = (OutReal) map[indexIn].baryCoords[1];
                const OutReal fz = (OutReal) map[indexIn].baryCoords[2];
                const OutReal oneMinusFx = 1-fx;
                const OutReal oneMinusFy = 1-fy;
                const OutReal oneMinusFz = 1-fz;

                o.addCol(cube[0], data * ((oneMinusFx) * (oneMinusFy) * (oneMinusFz)));
                o.addCol(cube[1], data * ((fx) * (oneMinusFy) * (oneMinusFz)));
#ifdef SOFA_NEW_HEXA
                o.addCol(cube[3], data * ((oneMinusFx) * (fy) * (oneMinusFz)));
                o.addCol(cube[2], data * ((fx) * (fy) * (oneMinusFz)));
#else
                o.addCol(cube[2], data * ((oneMinusFx) * (fy) * (oneMinusFz)));
                o.addCol(cube[3], data * ((fx) * (fy) * (oneMinusFz)));
#endif
                o.addCol(cube[4], data * ((oneMinusFx) * (oneMinusFy) * (fz)));
                o.addCol(cube[5], data * ((fx) * (oneMinusFy) * (fz)));
#ifdef SOFA_NEW_HEXA
                o.addCol(cube[7], data * ((oneMinusFx) * (fy) * (fz)));
                o.addCol(cube[6], data * ((fx) * (fy) * (fz)));
#else
                o.addCol(cube[6], data * ((oneMinusFx) * (fy) * (fz)));
                o.addCol(cube[7], data * ((fx) * (fy) * (fz)));
#endif
            }
        }
    }
}

template <class In, class Out>
void BarycentricMapperSparseGridTopology<In,Out>::applyJT ( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in )
{
    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();

        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            for ( ; colIt != colItEnd; ++colIt)
            {
                unsigned indexIn = colIt.index();
                InDeriv data;
                In::setDPos(data, Out::getDPos(colIt.val()));

#ifdef SOFA_NEW_HEXA
                const topology::SparseGridTopology::Hexa cube = this->fromTopology->getHexahedron ( this->map[indexIn].in_index );
#else
                const topology::SparseGridTopology::Cube cube = this->fromTopology->getCube ( this->map[indexIn].in_index );
#endif
                const OutReal fx = ( OutReal ) map[indexIn].baryCoords[0];
                const OutReal fy = ( OutReal ) map[indexIn].baryCoords[1];
                const OutReal fz = ( OutReal ) map[indexIn].baryCoords[2];
                const OutReal oneMinusFx = 1-fx;
                const OutReal oneMinusFy = 1-fy;
                const OutReal oneMinusFz = 1-fz;

                OutReal f = ( oneMinusFx * oneMinusFy * oneMinusFz );
                o.addCol ( cube[0],  ( data * f ) );

                f = ( ( fx ) * oneMinusFy * oneMinusFz );
                o.addCol ( cube[1],  ( data * f ) );

#ifdef SOFA_NEW_HEXA
                f = ( oneMinusFx * ( fy ) * oneMinusFz );
                o.addCol ( cube[3],  ( data * f ) );

                f = ( ( fx ) * ( fy ) * oneMinusFz );
                o.addCol ( cube[2],  ( data * f ) );

#else
                f = ( oneMinusFx * ( fy ) * oneMinusFz );
                o.addCol ( cube[2],  ( data * f ) );

                f = ( ( fx ) * ( fy ) * oneMinusFz );
                o.addCol ( cube[3],  ( data * f ) );

#endif
                f = ( oneMinusFx * oneMinusFy * ( fz ) );
                o.addCol ( cube[4],  ( data * f ) );

                f = ( ( fx ) * oneMinusFy * ( fz ) );
                o.addCol ( cube[5],  ( data * f ) );

#ifdef SOFA_NEW_HEXA
                f = ( oneMinusFx * ( fy ) * ( fz ) );
                o.addCol ( cube[7],  ( data * f ) );

                f = ( ( fx ) * ( fy ) * ( fz ) );
                o.addCol ( cube[6],  ( data * f ) );
#else
                f = ( oneMinusFx * ( fy ) * ( fz ) );
                o.addCol ( cube[6],  ( data * f ) );

                f = ( ( fx ) * ( fy ) * ( fz ) );
                o.addCol ( cube[7],  ( data * f ) );
#endif
            }
        }
    }
}

template <class In, class Out>
void BarycentricMapperEdgeSetTopology<In,Out>::applyJT ( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in )
{
    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();
    const sofa::helper::vector<topology::Edge>& edges = this->fromTopology->getEdges();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();

        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());
            for ( ; colIt != colItEnd; ++colIt)
            {
                unsigned indexIn = colIt.index();
                InDeriv data;
                In::setDPos(data, Out::getDPos(colIt.val()));

                const topology::Edge edge = edges[this->map.getValue()[indexIn].in_index];
                const OutReal fx = ( OutReal ) map.getValue()[indexIn].baryCoords[0];

                o.addCol ( edge[0], data * ( 1-fx ) );
                o.addCol ( edge[1], data * ( fx ) );
            }
        }
    }
}

template <class In, class Out>
void BarycentricMapperTriangleSetTopology<In,Out>::applyJT ( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in )
{
    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();
    const sofa::helper::vector<topology::Triangle>& triangles = this->fromTopology->getTriangles();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();

        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            for ( ; colIt != colItEnd; ++colIt)
            {
                unsigned indexIn = colIt.index();
                InDeriv data;
                In::setDPos(data, Out::getDPos(colIt.val()));

                const topology::Triangle triangle = triangles[this->map.getValue()[indexIn].in_index];
                const OutReal fx = ( OutReal ) map.getValue()[indexIn].baryCoords[0];
                const OutReal fy = ( OutReal ) map.getValue()[indexIn].baryCoords[1];

                o.addCol (triangle[0],data * ( 1-fx-fy ) );
                o.addCol (triangle[1],data * ( fx ) );
                o.addCol (triangle[2],data * ( fy ) );
            }
        }
    }
}

template <class In, class Out>
void BarycentricMapperQuadSetTopology<In,Out>::applyJT ( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in )
{
    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();
    const sofa::helper::vector<topology::Quad>& quads = this->fromTopology->getQuads();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();

        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            for ( ; colIt != colItEnd; ++colIt)
            {
                unsigned indexIn = colIt.index();
                InDeriv data;
                In::setDPos(data, Out::getDPos(colIt.val()));

                const OutReal fx = ( OutReal ) map.getValue()[indexIn].baryCoords[0];
                const OutReal fy = ( OutReal ) map.getValue()[indexIn].baryCoords[1];
                const sofa::core::topology::BaseMeshTopology::Quad& quad = quads[map.getValue()[indexIn].in_index];

                o.addCol (quad[0], data * ( ( 1-fx ) * ( 1-fy ) ) );
                o.addCol (quad[1], data * ( ( fx ) * ( 1-fy ) ) );
                o.addCol (quad[3], data * ( ( 1-fx ) * ( fy ) ) );
                o.addCol (quad[2], data * ( ( fx ) * ( fy ) ) );
            }
        }
    }
}

template <class In, class Out>
void BarycentricMapperTetrahedronSetTopology<In,Out>::applyJT ( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in )
{
    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();
    const sofa::helper::vector<topology::Tetrahedron>& tetrahedra = this->fromTopology->getTetrahedra();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();

        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            for ( ; colIt != colItEnd; ++colIt)
            {
                unsigned indexIn = colIt.index();
                InDeriv data;
                In::setDPos(data, Out::getDPos(colIt.val()));

                const OutReal fx = ( OutReal ) map.getValue()[indexIn].baryCoords[0];
                const OutReal fy = ( OutReal ) map.getValue()[indexIn].baryCoords[1];
                const OutReal fz = ( OutReal ) map.getValue()[indexIn].baryCoords[2];
                int index = map.getValue()[indexIn].in_index;
                const topology::Tetrahedron& tetra = tetrahedra[index];

                o.addCol (tetra[0], data * ( 1-fx-fy-fz ) );
                o.addCol (tetra[1], data * fx );
                o.addCol (tetra[2], data * fy );
                o.addCol (tetra[3], data * fz );
            }
        }
    }
}

template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::applyJT ( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in )
{
    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();
    const sofa::helper::vector< topology::Hexahedron >& cubes = this->fromTopology->getHexahedra();

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();

        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            for ( ; colIt != colItEnd; ++colIt)
            {
                unsigned indexIn = colIt.index();
                InDeriv data;
                In::setDPos(data, Out::getDPos(colIt.val()));

                const OutReal fx = ( OutReal ) map.getValue()[indexIn].baryCoords[0];
                const OutReal fy = ( OutReal ) map.getValue()[indexIn].baryCoords[1];
                const OutReal fz = ( OutReal ) map.getValue()[indexIn].baryCoords[2];
                const OutReal oneMinusFx = 1-fx;
                const OutReal oneMinusFy = 1-fy;
                const OutReal oneMinusFz = 1-fz;

                int index = map.getValue()[indexIn].in_index;
                const topology::Hexahedron& cube = cubes[index];

                o.addCol (cube[0], data * ( oneMinusFx * oneMinusFy * oneMinusFz ) );
                o.addCol (cube[1], data * ( ( fx ) * oneMinusFy * oneMinusFz ) );
                o.addCol (cube[3], data * ( oneMinusFx * ( fy ) * oneMinusFz ) );
                o.addCol (cube[2], data * ( ( fx ) * ( fy ) * oneMinusFz ) );
                o.addCol (cube[4], data * ( oneMinusFx * oneMinusFy * ( fz ) ) );
                o.addCol (cube[5], data * ( ( fx ) * oneMinusFy * ( fz ) ) );
                o.addCol (cube[7], data * ( oneMinusFx * ( fy ) * ( fz ) ) );
                o.addCol (cube[6], data * ( ( fx ) * ( fy ) * ( fz ) ) );
            }
        }
    }
}

/************************************* Topological Changes ***********************************/



template <class In, class Out>
void BarycentricMapperHexahedronSetTopology<In,Out>::handleTopologyChange(core::topology::Topology* t)
{
    using sofa::core::behavior::MechanicalState;

    if (t != this->fromTopology) return;

    if ( this->fromTopology->beginChange() == this->fromTopology->endChange() )
        return;

    std::list<const core::topology::TopologyChange *>::const_iterator itBegin = this->fromTopology->beginChange();
    std::list<const core::topology::TopologyChange *>::const_iterator itEnd = this->fromTopology->endChange();

    for ( std::list<const core::topology::TopologyChange *>::const_iterator changeIt = itBegin;
            changeIt != itEnd; ++changeIt )
    {
        const core::topology::TopologyChangeType changeType = ( *changeIt )->getChangeType();
        switch ( changeType )
        {
            //TODO: implementation of BarycentricMapperHexahedronSetTopology<In,Out>::handleTopologyChange()
        case core::topology::ENDING_EVENT:       ///< To notify the end for the current sequence of topological change events
        {
            if(!_invalidIndex.empty())
            {
                helper::vector<MappingData>& mapData = *(map.beginEdit());

                for ( std::set<int>::const_iterator iter = _invalidIndex.begin();
                        iter != _invalidIndex.end(); ++iter )
                {
                    const int j = *iter;
                    if ( mapData[j].in_index == -1 ) // compute new mapping
                    {
                        //  std::cout << "BarycentricMapperHexahedronSetTopology : new mapping" << std::endl;
                        sofa::defaulttype::Vector3 coefs;
                        typename In::Coord pos;
                        pos[0] = mapData[j].baryCoords[0];
                        pos[1] = mapData[j].baryCoords[1];
                        pos[2] = mapData[j].baryCoords[2];

                        // find nearest cell and barycentric coords
                        Real distance = 1e10;

                        int index = -1;
                        // When smoothing a mesh, the element has to be found using the rest position of the point. Then, its position is set using this element.
                        if( this->toTopology)
                        {
                            typedef MechanicalState<Out> MechanicalStateT;
                            MechanicalStateT* mState;
                            this->toTopology->getContext()->get( mState);
                            if( !mState)
                            {
                                std::cerr << "Can not find mechanical state." << std::endl;
                            }
                            else
                            {
                                const typename MechanicalStateT::VecCoord& xto0 = (mState->read(core::ConstVecCoordId::restPosition())->getValue());
                                typename In::Coord xto0coord;
                                In::setCPos(xto0coord,Out::getCPos(xto0[j]));
                                index = _fromGeomAlgo->findNearestElementInRestPos ( xto0coord, coefs, distance );
                                //_fromGeomAlgo->findNearestElementInRestPos ( pos, coefs, distance );
                                coefs = _fromGeomAlgo->computeHexahedronRestBarycentricCoeficients(index, pos);
                            }
                        }
                        else
                        {
                            index = _fromGeomAlgo->findNearestElementInRestPos ( pos, coefs, distance );
                        }

                        if ( index != -1 )
                        {
                            mapData[j].baryCoords[0] = ( Real ) coefs[0];
                            mapData[j].baryCoords[1] = ( Real ) coefs[1];
                            mapData[j].baryCoords[2] = ( Real ) coefs[2];
                            mapData[j].in_index = index;
                        }
                    }
                }

                map.endEdit();
                _invalidIndex.clear();
            }
        }
        break;
        case core::topology::POINTSINDICESSWAP:  ///< For PointsIndicesSwap.
            break;
        case core::topology::POINTSADDED:        ///< For PointsAdded.
            break;
        case core::topology::POINTSREMOVED:      ///< For PointsRemoved.
            break;
        case core::topology::POINTSRENUMBERING:  ///< For PointsRenumbering.
            break;
        case core::topology::TRIANGLESADDED:  ///< For Triangles Added.
            break;
        case core::topology::TRIANGLESREMOVED:  ///< For Triangles Removed.
            break;
        case core::topology::HEXAHEDRAADDED:     ///< For HexahedraAdded.
        {
        }
        break;
        case core::topology::HEXAHEDRAREMOVED:   ///< For HexahedraRemoved.
        {
            // std::cout << "BarycentricMapperHexahedronSetTopology() HEXAHEDRAREMOVED" << std::endl;
            const unsigned int nbHexahedra = this->fromTopology->getNbHexahedra();

            const sofa::helper::vector<unsigned int> &hexahedra =
                    ( static_cast< const core::topology::HexahedraRemoved *> ( *changeIt ) )->getArray();
            //        sofa::helper::vector<unsigned int> hexahedra(tab);

            for ( unsigned int i=0; i<hexahedra.size(); ++i )
            {
                // remove all references to the removed cubes from the mapping data
                unsigned int cubeId = hexahedra[i];
                for ( unsigned int j=0; j<map.getValue().size(); ++j )
                {
                    if ( map.getValue()[j].in_index == ( int ) cubeId ) // invalidate mapping
                    {
                        sofa::defaulttype::Vector3 coefs;
                        coefs[0] = map.getValue()[j].baryCoords[0];
                        coefs[1] = map.getValue()[j].baryCoords[1];
                        coefs[2] = map.getValue()[j].baryCoords[2];

                        typename In::Coord restPos = _fromGeomAlgo->getRestPointPositionInHexahedron ( cubeId, coefs );

                        helper::vector<MappingData>& vectorData = *(map.beginEdit());
                        vectorData[j].in_index = -1;
                        vectorData[j].baryCoords[0] = restPos[0];
                        vectorData[j].baryCoords[1] = restPos[1];
                        vectorData[j].baryCoords[2] = restPos[2];
                        map.endEdit();

                        _invalidIndex.insert(j);
                    }
                }
            }

            // renumber
            unsigned int lastCubeId = nbHexahedra-1;
            for ( unsigned int i=0; i<hexahedra.size(); ++i, --lastCubeId )
            {
                unsigned int cubeId = hexahedra[i];
                for ( unsigned int j=0; j<map.getValue().size(); ++j )
                {
                    if ( map.getValue()[j].in_index == ( int ) lastCubeId )
                    {
                        helper::vector<MappingData>& vectorData = *(map.beginEdit());
                        vectorData[j].in_index = cubeId;
                        map.endEdit();
                    }
                }
            }
        }
        break;
        case core::topology::HEXAHEDRARENUMBERING: ///< For HexahedraRenumbering.
            break;
        default:
            break;
        }
    }
}

// handle topology changes depending on the topology
template <class TIn, class TOut>
void BarycentricMapping<TIn, TOut>::handleTopologyChange ( core::topology::Topology* t )
{
    if (d_handleTopologyChange.getValue() && mapper)
    {
        mapper->handleTopologyChange(t);
    }
}

#ifdef SOFA_HAVE_EIGEN2

template<class TIn, class TOut>
const vector< defaulttype::BaseMatrix*>* BarycentricMapping<TIn, TOut>::getJs()
{
    //std::cerr << this->getName() << ": getJs " << std::endl;
    typedef typename Mapper::MatrixType mat_type;
    const sofa::defaulttype::BaseMatrix* matJ = getJ();

    const mat_type* mat = mat_type::DynamicCast(matJ);
    assert( mat );

    eigen.copyFrom( *mat );   // woot

    js.resize( 1 );
    js[0] = &eigen;
    return &js;
}


#endif

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
