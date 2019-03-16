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
#ifndef SOFA_COMPONENT_COLLISION_TRIANGLEMODEL_INL
#define SOFA_COMPONENT_COLLISION_TRIANGLEMODEL_INL

#include <SofaMeshCollision/TriangleModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaMeshCollision/TriangleLocalMinDistanceFilter.h>
#include <SofaBaseCollision/CubeModel.h>
#include <SofaMeshCollision/Triangle.h>
#include <SofaBaseTopology/TopologyData.inl>
#include <sofa/simulation/common/Node.h>
#include <SofaBaseTopology/RegularGridTopology.h>
#include <sofa/core/CollisionElement.h>
#include <vector>
#include <sofa/helper/gl/template.h>
#include <iostream>

#include <sofa/core/topology/TopologyChange.h>

#include <sofa/simulation/common/Simulation.h>

namespace sofa
{

namespace component
{

namespace collision
{

template<class DataTypes>
TTriangleModel<DataTypes>::TTriangleModel()
    : computeNormals(initData(&computeNormals, true, "computeNormals", "set to false to disable computation of triangles normal"))
    , d_boundaryAngleThreshold(initData(&d_boundaryAngleThreshold,Real(180),"boundaryAngleThreshold","Angle threshold (in degrees) above which an edge or a point is qualified as boundary.\
                                                                                                          0    -> All edges/points are flagged as boundary. \
                                                                                                          180  -> Only edges with a single adjacent triangle are marked as boundary \
                                                                                                                  and only points attached to these boundary edges are marked as boundary."))
    , d_minTriangleArea(initData(&d_minTriangleArea, Real(1.0e-6), "minTriangleArea", "Triangle area threshold below which elements are considered as badly shaped and collisions are disabled"))
    , d_drawBoundaryPoints(initData(&d_drawBoundaryPoints, false, "drawBoundaryPoints", "Draw triangle points that are classified as boundary."))
    , d_drawBoundaryEdges(initData(&d_drawBoundaryEdges, false, "drawBoundaryEdges", "Draw triangle edges that are classified as boundary."))
{
    triangles = &mytriangles;
    enum_type = TRIANGLE_TYPE;

    d_boundaryAngleThreshold.setGroup("TriangleFlags_");
}

template<class DataTypes>
void TTriangleModel<DataTypes>::resize(int size)
{
    this->core::CollisionModel::resize(size);
    //helper::vector<TriangleInfo>& e = *(elems.beginEdit());
    //e.resize(size);
    //elems.endEdit();
    normals.resize(size);
    triangleFlags.resize(size);
}

template<class DataTypes>
void TTriangleModel<DataTypes>::init()
{
    _topology = this->getContext()->getActiveMeshTopology();
    mstate = core::behavior::MechanicalState<DataTypes>::DynamicCast(this->getContext()->getMechanicalState());
    this->getContext()->get(mpoints);

    Inherit1::init();

    if (mstate==NULL)
    {
        serr << "TriangleModel requires a Vec3 Mechanical Model" << sendl;
        return;
    }

    if (!_topology)
    {
        serr << "TriangleModel requires a BaseMeshTopology" << sendl;
        return;
    }

    simulation::Node* node = simulation::Node::DynamicCast(this->getContext());
    if (node != 0)
    {
        m_lmdFilter = node->getNodeObject< TriangleLocalMinDistanceFilter >();
    }

    //sout << "INFO_print : Col - init TRIANGLE " << sendl;
    sout << "TriangleModel: initially "<<_topology->getNbTriangles()<<" triangles." << sendl;
    triangles = &_topology->getTriangles();
    resize(_topology->getNbTriangles());

    updateFromTopology();
    updateNormals();
}

template<class DataTypes>
void TTriangleModel<DataTypes>::reinit()
{
    updateFlags();
}

template< class DataTypes> 
inline typename DataTypes::Deriv computeTriangleNormal(const typename DataTypes::Coord& p0,
    const typename DataTypes::Coord& p1,
    const typename DataTypes::Coord& p2)
{
    typename DataTypes::Deriv n;
    DataTypes::setDPos(n,cross(DataTypes::getCPos(p1-p0), DataTypes::getCPos(p2-p0)).normalized());
    return n;
}


template<class DataTypes>
void TTriangleModel<DataTypes>::updateNormals()
{
    for (int i=0; i<size; i++)
    {
        Element t(this,i);
        t.n() = computeTriangleNormal<DataTypes>(t.p1(), t.p2(), t.p3());
    }
}

template<class DataTypes>
void TTriangleModel<DataTypes>::updateFromTopology()
{
    //    needsUpdate = false;
    const unsigned ntris = _topology->getNbTriangles();
    const unsigned nquads = _topology->getNbQuads();
    const unsigned newsize = ntris+2*nquads;

    int revision = _topology->getRevision();
    if (newsize==(unsigned)size && revision == meshRevision)
        return;
    meshRevision = revision;

    needsUpdate = true;

    resize(newsize);

    unsigned npoints = mstate->read(core::ConstVecCoordId::position())->getValue().size();
    if (npoints != (unsigned)_topology->getNbPoints())
    {
        serr << "Mismatch between number of points in topology ("<<_topology->getNbPoints()
             << ") and size of mstate positions vector ("<< npoints << ")" << sendl;
        npoints = (unsigned)_topology->getNbPoints();
        mstate->resize(npoints);
    }

    if (newsize == ntris)
    {
        // no need to copy the triangle indices
        triangles = & _topology->getTriangles();
    }
    else
    {
        triangles = &mytriangles;
        mytriangles.resize(newsize);
        int index = 0;
        for (unsigned i=0; i<ntris; i++)
        {
            topology::BaseMeshTopology::Triangle idx = _topology->getTriangle(i);
            if (idx[0] >= npoints || idx[1] >= npoints || idx[2] >= npoints)
            {
                serr << "ERROR: Out of range index in triangle "<<i<<": "<<idx[0]<<" "<<idx[1]<<" "<<idx[2]<<" ( total points="<<npoints<<")"<<sendl;
                if (idx[0] >= npoints) idx[0] = npoints-1;
                if (idx[1] >= npoints) idx[1] = npoints-1;
                if (idx[2] >= npoints) idx[2] = npoints-1;
            }
            mytriangles[index] = idx;
            ++index;
        }
        for (unsigned i=0; i<nquads; i++)
        {
            topology::BaseMeshTopology::Quad idx = _topology->getQuad(i);
            if (idx[0] >= npoints || idx[1] >= npoints || idx[2] >= npoints || idx[3] >= npoints)
            {
                serr << "ERROR: Out of range index in quad "<<i<<": "<<idx[0]<<" "<<idx[1]<<" "<<idx[2]<<" "<<idx[3]<<" ( total points="<<npoints<<")"<<sendl;
                if (idx[0] >= npoints) idx[0] = npoints-1;
                if (idx[1] >= npoints) idx[1] = npoints-1;
                if (idx[2] >= npoints) idx[2] = npoints-1;
                if (idx[3] >= npoints) idx[3] = npoints-1;
            }
            mytriangles[index][0] = idx[1];
            mytriangles[index][1] = idx[2];
            mytriangles[index][2] = idx[0];
            ++index;
            mytriangles[index][0] = idx[3];
            mytriangles[index][1] = idx[0];
            mytriangles[index][2] = idx[2];
            ++index;
        }
    }
    updateNormals();
    updateFlags();
}

template<class DataTypes>
void TTriangleModel<DataTypes>::updateFlags()
{
    auto x0 = this->mstate->readRestPositions();
    auto x  = this->mstate->readPositions();

    // if the topology of the triangle mesh used for the collision is modified 
    // and the rest position is not updated, we cannot update the edge border
    // flags value based on the angle made by the normal between adjacent triangles. 
    const bool computeAngleAtEdge = x0.size() == x.size();

    Real angleThreshold = d_boundaryAngleThreshold.getValue();
    angleThreshold *= M_PI / Real(180);
    const Real cosAngleThreshold = std::cos(angleThreshold);
    sofa::helper::vector<sofa::core::topology::BaseMeshTopology::EdgeID> boundaryEdges;

    // support for quads split as two triangles
    const unsigned int ntris = _topology->getNbTriangles();
    //const unsigned int nquads = _topology->getNbQuads();

    for (sofa::core::topology::BaseMeshTopology::TriangleID tid=0; tid< ntris; ++tid)
    {
        sofa::core::topology::BaseMeshTopology::Triangle t = (*triangles)[tid];
        int f = 0;
        for (unsigned int j=0; j<3; ++j)
        {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex& tav = _topology->getTrianglesAroundVertex(t[j]);
            if (tav[0] == tid)
                f |= (FLAG_P1 << j);
            if (tav.size() == 1)
                f |= (FLAG_BP1 << j);
        }

        const sofa::core::topology::BaseMeshTopology::EdgesInTriangle& e = _topology->getEdgesInTriangle(tid);

        for (unsigned int j=0; j<3; ++j)
        {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundEdge& tae = _topology->getTrianglesAroundEdge(e[j]);
            if (tae[0] == tid)
            {
                f |= (FLAG_E23 << j);

                if (tae.size() == 1)
                {
                    f |= (FLAG_BE23 << j);
                    boundaryEdges.push_back(e[j]);
                }
                else if (computeAngleAtEdge)
                {
                    const sofa::core::topology::Topology::Triangle tri_0 = (*triangles)[tae[0]];
                    const Deriv n_0 = computeTriangleNormal<DataTypes>(x0[tri_0[0]],
                                                                       x0[tri_0[1]],
                                                                       x0[tri_0[2]]);
                    for (std::size_t k=1; k<tae.size(); ++k)
                    {
                        const sofa::core::topology::Topology::Triangle tri_k = (*triangles)[tae[k]];
                        const Deriv n_k = computeTriangleNormal<DataTypes>(x0[tri_k[0]],
                                                                           x0[tri_k[1]],
                                                                           x0[tri_k[2]]);
                        const Real cos = dot(DataTypes::getDPos(n_0), DataTypes::getDPos(n_k));
                        const bool isAngleAboveThreshold = cos <= cosAngleThreshold;
                        if (isAngleAboveThreshold)
                        {
                            f |= (FLAG_BE23 << j);
                            boundaryEdges.push_back(e[j]);
                        }
                    }
                }
            }
        }
        triangleFlags[tid] = f;
    }

    // 2nd pass to set up boundary points according to boundary edges
    for (sofa::core::topology::BaseMeshTopology::TriangleID tid = 0; tid < triangles->size(); ++tid)
    {
        sofa::core::topology::BaseMeshTopology::Triangle t = (*triangles)[tid];
        int f = triangleFlags[tid];
        for (unsigned int i = 0; i < 3; ++i)
        {
            if (!(f&FLAG_P1 << i)) continue; // this point is not attached to the triangle
            if (f&FLAG_BP1 << i) continue; // already classified as a boundary point

            const sofa::core::topology::BaseMeshTopology::EdgesAroundVertex& eav = _topology->getEdgesAroundVertex(t[i]);

            // a point is a boundary if at least one adjacent edge is a boundary
            for (auto eid : eav)
            {
                if (std::find(boundaryEdges.begin(), boundaryEdges.end(), eid) != boundaryEdges.end())
                {
                    f |= (FLAG_BP1 << i);
                    break;
                }
            }
        }
        triangleFlags[tid] = f;
    }

    const auto& quads = _topology->getQuads();

    // each quad [0,1,2,3] is split in two triangles: [1,2,0] and [3,0,2]
    for (sofa::core::topology::BaseMeshTopology::TriangleID tid = ntris; tid < triangleFlags.size(); ++tid)
    {
        sofa::core::topology::BaseMeshTopology::QuadID qid = (tid-ntris)/2;
        int tIndexInQuad = (tid-ntris)&1;
        sofa::core::topology::BaseMeshTopology::Triangle t = (*triangles)[tid];
        int f = 0;
        // only look at the first 2 vertices of the triangles, covering all 4 quad vertices in the pair of triangles
        for (unsigned int j=0; j<2; ++j)
        {
            if (_topology->getTrianglesAroundVertex(t[j]).empty() && _topology->getQuadsAroundVertex(t[j])[0] == qid)
                f |= (FLAG_P1 << j);
        }

        const sofa::core::topology::BaseMeshTopology::EdgesInQuad eq = _topology->getEdgesInQuad(qid);
        sofa::core::topology::BaseMeshTopology::EdgesInTriangle e;
        e[0] = sofa::core::topology::BaseMeshTopology::InvalidID;
        if (tIndexInQuad == 0)
        {
            e[1] = eq[3]; // 01
            e[2] = eq[0]; // 12
            f |= (FLAG_E23 << 0); // we arbitrarly associate the diagonal edge to the first triangle in the quad
        }
        else
        {
            e[1] = eq[1]; // 23
            e[2] = eq[2]; // 30
        }

        // skip the first edge of each triangle, which is the quad diagonal
        for (unsigned int j=1; j<3; ++j)
        {
            const sofa::core::topology::BaseMeshTopology::QuadsAroundEdge& qae = _topology->getQuadsAroundEdge(e[j]);
            if (qae[0] == qid)
            {
                f |= (FLAG_E23 << j);
                if (qae.size() == 1)
                {
                    f |= (FLAG_BE23 << j);
                }
                else if (computeAngleAtEdge)
                {
                    const sofa::core::topology::Topology::Quad q_0 = quads[qae[0]];
                    const Deriv n_0 = computeTriangleNormal<DataTypes>(x0[q_0[0]], x0[q_0[1]], x0[q_0[2]]);
                    for (std::size_t k=1; k<qae.size(); ++k)
                    {
                        const sofa::core::topology::Topology::Quad q_k = quads[qae[k]];
                        const Deriv n_k = computeTriangleNormal<DataTypes>(x0[q_k[0]], x0[q_k[1]], x0[q_k[2]]);
                        const Real cos = dot(DataTypes::getDPos(n_0), DataTypes::getDPos(n_k));
                        const bool isAngleAboveThreshold = cos < cosAngleThreshold;
                        if (isAngleAboveThreshold)
                        {
                            f |= (FLAG_BE23 << j);
                        }
                    }
                }
            }
        }
        triangleFlags[tid] = f;
    }
}

template<class DataTypes>
void TTriangleModel<DataTypes>::handleTopologyChange()
{
    //bool debug_mode = false;
    if (triangles != &mytriangles)
    {
        // We use the same triangle array as the topology -> only resize and recompute flags

        std::list<const sofa::core::topology::TopologyChange *>::const_iterator itBegin=_topology->beginChange();
        std::list<const sofa::core::topology::TopologyChange *>::const_iterator itEnd=_topology->endChange();
        //elems.handleTopologyEvents(itBegin,itEnd);

        while( itBegin != itEnd )
        {
            core::topology::TopologyChangeType changeType = (*itBegin)->getChangeType();

            switch( changeType )
            {
                case core::topology::ENDING_EVENT:
                {
                    updateFromTopology();
                }
                // fallthrough
                case sofa::core::topology::TRIANGLESREMOVED:
                case sofa::core::topology::TRIANGLESADDED:
                case sofa::core::topology::QUADSREMOVED:
                case sofa::core::topology::QUADSADDED:
                {
                    sout << "TriangleModel: now "<<_topology->getNbTriangles()<<" triangles." << sendl;
                    needsUpdate=true;
                }

            default: break;
            }
            ++itBegin;
        }
    }
}

template<class DataTypes>
void TTriangleModel<DataTypes>::draw(const core::visual::VisualParams* vparams ,int index)
{
    Element t(this,index);
//        glBegin(GL_TRIANGLES);
//        helper::gl::glNormalT(t.n());
//        helper::gl::glVertexT(t.p1());
//        helper::gl::glVertexT(t.p2());
//        helper::gl::glVertexT(t.p3());
//        glEnd();

    vparams->drawTool()->setPolygonMode(0,vparams->displayFlags().getShowWireFrame());
    vparams->drawTool()->setLightingEnabled(true);
    vparams->drawTool()->drawTriangle( DataTypes::getCPos(t.p1()), DataTypes::getCPos(t.p2()), DataTypes::getCPos(t.p3()), DataTypes::getDPos(t.n()) );
    vparams->drawTool()->setLightingEnabled(false);

}


template<class DataTypes>
void TTriangleModel<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (vparams->displayFlags().getShowCollisionModels())
    {
        //if( size != _topology->getNbTriangles())
        //  updateFromTopology();

        const bool drawBPoints = d_drawBoundaryPoints.getValue();
        const bool drawBEdges = d_drawBoundaryEdges.getValue();

        if (bBothSide.getValue() || vparams->displayFlags().getShowWireFrame())
        {
            vparams->drawTool()->setPolygonMode(0, vparams->displayFlags().getShowWireFrame());
        }
        else
        {
            vparams->drawTool()->setPolygonMode(2,true);
            vparams->drawTool()->setPolygonMode(1,false);
        }

        sofa::helper::vector< defaulttype::Vector3 > points, pointsBP, pointsBE;
        sofa::helper::vector< defaulttype::Vec<3,int> > indices;
        sofa::helper::vector< defaulttype::Vector3 > normals;
        int index=0;
        for (int i=0; i<size; i++)
        {
            Element t(this,i);
            normals.push_back(DataTypes::getDPos(t.n()));
            points.push_back(DataTypes::getCPos(t.p1()));
            points.push_back(DataTypes::getCPos(t.p2()));
            points.push_back(DataTypes::getCPos(t.p3()));
            indices.push_back(defaulttype::Vec<3,int>(index,index+1,index+2));
            index+=3;

            if (drawBPoints)
            {
                const int f = t.flags();
                sofa::helper::fixed_array< sofa::defaulttype::Vector3, 3> tpv;
                tpv[0] = DataTypes::getCPos(t.p1());
                tpv[1] = DataTypes::getCPos(t.p2());
                tpv[2] = DataTypes::getCPos(t.p3());
                for (unsigned int j = 0; j < 3; ++j)
                {
                    if (f&FLAG_BP1 << j)
                    {
                        pointsBP.push_back(tpv[j]);
                    }
                }
            }

            if (drawBEdges)
            {
                const int f = t.flags();
                sofa::core::topology::BaseMeshTopology* topo = t.getCollisionModel()->getMeshTopology();
                const sofa::core::topology::BaseMeshTopology::EdgesInTriangle& eit = topo->getEdgesInTriangle(t.getIndex());
                sofa::helper::fixed_array< sofa::defaulttype::Vector3, 3> tpv;
                tpv[0] = DataTypes::getCPos(t.p1());
                tpv[1] = DataTypes::getCPos(t.p2());
                tpv[2] = DataTypes::getCPos(t.p3());
                sofa::helper::fixed_array<unsigned int,3> tiv;
                tiv[0] = t.p1Index();
                tiv[1] = t.p2Index();
                tiv[2] = t.p3Index();
                for (unsigned int j = 0; j < eit.size(); ++j)
                {
                    if (f&FLAG_BE23 << j)
                    {
                        sofa::core::topology::BaseMeshTopology::Edge E = topo->getEdge(eit[j]);
                        const unsigned int piv[2] = { (E[0] == tiv[0] ) ? 0U : (E[0] == tiv[1] ) ? 1U : 2U,
                                                      (E[1] == tiv[0] ) ? 0U : (E[1] == tiv[1] ) ? 1U : 2U };
                        const unsigned int i1 = piv[0];
                        const unsigned int i2 = piv[1];
                        pointsBE.push_back(tpv[i1]);
                        pointsBE.push_back(tpv[i2]);
                    }
                }
            }
        }

        vparams->drawTool()->setLightingEnabled(true);
        vparams->drawTool()->drawTriangles(points, indices, normals, defaulttype::Vec<4,float>(getColor4f()));
        vparams->drawTool()->setLightingEnabled(false);
        vparams->drawTool()->setPolygonMode(0,false);

        if (drawBPoints) vparams->drawTool()->drawPoints(pointsBP, 10.0, defaulttype::Vec<4,float>(1,0,0,1));
        if (drawBEdges) vparams->drawTool()->drawLines(pointsBE, 1.0, defaulttype::Vec<4,float>(1,0,0,1));

        if (vparams->displayFlags().getShowNormals())
        {
            sofa::helper::vector< defaulttype::Vector3 > pointsEdges;
            for (int i=0; i<size; i++)
            {
                Element t(this,i);
                pointsEdges.push_back(DataTypes::getCPos((t.p1()+t.p2()+t.p3()))/3.0);
                pointsEdges.push_back(pointsEdges.back()+DataTypes::getDPos(t.n()));
            }

            vparams->drawTool()->drawLines(pointsEdges, 1, defaulttype::Vec<4,float>(1,1,1,1));
        }
    }

    if (getPrevious()!=NULL && vparams->displayFlags().getShowBoundingCollisionModels())
        getPrevious()->draw(vparams);
}


template<class DataTypes>
bool TTriangleModel<DataTypes>::canCollideWithElement(int index, CollisionModel* model2, int index2)
{
    if (!this->bSelfCollision.getValue()) return true; // we need to perform this verification process only for the selfcollision case.
    if (this->getContext() != model2->getContext()) return true;

    //if (model2 == mpoints && index2==4)
    //{
    //	std::cout<<"Triangle model : at index ["<<index<<"] can collide with point 4 ?"<<std::endl;
    //}


    //return true;

    Element t(this,index);
    if (model2 == mpoints)
    {
        // if point belong to the triangle, return false
        if ( index2==t.p1Index() || index2==t.p2Index() || index2==t.p3Index())
            return false;

        //const helper::vector <unsigned int>& EdgesAroundVertex11 =topology->getEdgesAroundVertex(p11);
        //const helper::vector <unsigned int>& EdgesAroundVertex12 =topology->getEdgesAroundVertex(p12);

        //// if the point belong to the the neighborhood of the triangle, return false
        //for (unsigned int i1=0; i1<EdgesAroundVertex11.size(); i1++)
        //{
        //	unsigned int e11 = EdgesAroundVertex11[i1];
        //	p11 = elems[e11].i1;
        //	p12 = elems[e11].i2;
        //	if (index2==p11 || index2==p12)
        //		return false;
        //}
        //for (unsigned int i1=0; i1<EdgesAroundVertex11.size(); i1++)
        //{
        //	unsigned int e12 = EdgesAroundVertex12[i1];
        //	p11 = elems[e12].i1;
        //	p12 = elems[e12].i2;
        //	if (index2==p11 || index2==p12)
        //		return false;
    }

    //// TODO : case with auto-collis with segment and auto-collis with itself

    return true;

}

template< class Real >
Real computeTriangleAreaSquared(const sofa::defaulttype::Vec<3, Real>& p0, const sofa::defaulttype::Vec<3, Real>& p1, const sofa::defaulttype::Vec<3, Real>& p2)
{
    return (Real(0.25)*((p1-p0).cross(p2-p0)).norm2());
}

template<class DataTypes>
void TTriangleModel<DataTypes>::computeBoundingTree(int maxDepth)
{
    CubeModel* cubeModel = createPrevious<CubeModel>();
    updateFromTopology();

    if (needsUpdate && !cubeModel->empty()) cubeModel->resize(0);

    if (!isMoving() && !cubeModel->empty() && !needsUpdate) return; // No need to recompute BBox if immobile

    needsUpdate=false;
    defaulttype::Vector3 minElem, maxElem;
    const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();

    const bool calcNormals = computeNormals.getValue();
    const Real minTriangleArea = d_minTriangleArea.getValue();
    const Real minTriangleArea2 = minTriangleArea*minTriangleArea;
    const bool updateBadShape = minTriangleArea != Real(0.0);

//    if (maxDepth == 0)
//    {
//        // no hierarchy
//        if (empty())
//            cubeModel->resize(0);
//        else
//        {
//            cubeModel->resize(1);
//            minElem = x[0];
//            maxElem = x[0];
//            for (unsigned i=1; i<x.size(); i++)
//            {
//                const defaulttype::Vector3& pt1 = x[i];
//                if (pt1[0] > maxElem[0]) maxElem[0] = pt1[0];
//                else if (pt1[0] < minElem[0]) minElem[0] = pt1[0];
//                if (pt1[1] > maxElem[1]) maxElem[1] = pt1[1];
//                else if (pt1[1] < minElem[1]) minElem[1] = pt1[1];
//                if (pt1[2] > maxElem[2]) maxElem[2] = pt1[2];
//                else if (pt1[2] < minElem[2]) minElem[2] = pt1[2];
//            }
//            const SReal distance = (SReal)this->proximity.getValue();
//            for (int c = 0; c < 3; c++)
//            {
//                minElem[c] -= distance;
//                maxElem[c] += distance;
//            }
//            cubeModel->setLeafCube(0, std::make_pair(this->begin(),this->end()), minElem, maxElem); // define the bounding box of the current triangle
//            if (calcNormals)
//                for (int i=0; i<size; i++)
//                {
//                    Element t(this,i);
//                    const defaulttype::Vector3& pt1 = x[t.p1Index()];
//                    const defaulttype::Vector3& pt2 = x[t.p2Index()];
//                    const defaulttype::Vector3& pt3 = x[t.p3Index()];

//                    /*for (int c = 0; c < 3; c++)
//                    {
//                    if (i==0)
//                    {
//                    minElem[c] = pt1[c];
//                    maxElem[c] = pt1[c];
//                    }
//                    else
//                    {
//                    if (pt1[c] > maxElem[c]) maxElem[c] = pt1[c];
//                    else if (pt1[c] < minElem[c]) minElem[c] = pt1[c];
//                    }
//                    if (pt2[c] > maxElem[c]) maxElem[c] = pt2[c];
//                    else if (pt2[c] < minElem[c]) minElem[c] = pt2[c];
//                    if (pt3[c] > maxElem[c]) maxElem[c] = pt3[c];
//                    else if (pt3[c] < minElem[c]) minElem[c] = pt3[c];
//                    }*/

//                    // Also recompute normal vector
//                    t.n() = cross(pt2-pt1,pt3-pt1);
//                    t.n().normalize();
//                }
//        }
//    }
//    else
//    {

        cubeModel->resize(size);  // size = number of triangles
        if (!empty())
        {
            const SReal distance = (SReal)this->proximity.getValue();
            for (int i=0; i<size; i++)
            {
                Element t(this,i);
                const defaulttype::Vector3& pt1 = DataTypes::getCPos(x[t.p1Index()]);
                const defaulttype::Vector3& pt2 = DataTypes::getCPos(x[t.p2Index()]);
                const defaulttype::Vector3& pt3 = DataTypes::getCPos(x[t.p3Index()]);

                if (updateBadShape)
                {
                    const Real triangleArea2 = computeTriangleAreaSquared(pt1,pt2,pt3); // area of the triangle squared.
                    if (triangleArea2 < minTriangleArea2)
                    {
                        triangleFlags[i] |= FLAG_BADSHAPE;
                        if (m_countBadShape++ == 0)
                        { // show a warning, but only the first time a triangle is flagged in this model
                            serr << "Triangle with index " << i << " (area = " << sqrt(triangleArea2)
                                 << ") is badly shaped: collision detection within this triangle will be disabled." << sendl;
                        }
                    }
                    else
                    {
                        triangleFlags[i] &= ~FLAG_BADSHAPE;
                    }
                }
                
                for (int c = 0; c < 3; c++)
                {
                    minElem[c] = pt1[c];
                    maxElem[c] = pt1[c];
                    if (pt2[c] > maxElem[c]) maxElem[c] = pt2[c];
                    else if (pt2[c] < minElem[c]) minElem[c] = pt2[c];
                    if (pt3[c] > maxElem[c]) maxElem[c] = pt3[c];
                    else if (pt3[c] < minElem[c]) minElem[c] = pt3[c];
                    minElem[c] -= distance;
                    maxElem[c] += distance;
                }
                if (calcNormals)
                {
                    // Also recompute normal vector
                    t.n() = computeTriangleNormal<DataTypes>(x[t.p1Index()], x[t.p2Index()], x[t.p3Index()]);
                }
                cubeModel->setParentOf(i, minElem, maxElem); // define the bounding box of the current triangle
            }
            cubeModel->computeBoundingTree(maxDepth);
        }
    //}

    if (m_lmdFilter != 0)
    {
        m_lmdFilter->invalidate();
    }
}

template<class DataTypes>
void TTriangleModel<DataTypes>::computeContinuousBoundingTree(double dt, int maxDepth)
{
    CubeModel* cubeModel = createPrevious<CubeModel>();
    updateFromTopology();
    if (needsUpdate) cubeModel->resize(0);
    if (!isMoving() && !cubeModel->empty() && !needsUpdate) return; // No need to recompute BBox if immobile

    needsUpdate=false;
    defaulttype::Vector3 minElem, maxElem;

    cubeModel->resize(size);
    if (!empty())
    {
        const SReal distance = (SReal)this->proximity.getValue();
        for (int i=0; i<size; i++)
        {
            Element t(this,i);
            const defaulttype::Vector3& pt1 = DataTypes::getCPos(t.p1());
            const defaulttype::Vector3& pt2 = DataTypes::getCPos(t.p2());
            const defaulttype::Vector3& pt3 = DataTypes::getCPos(t.p3());
            const defaulttype::Vector3 pt1v = pt1 + DataTypes::getDPos(t.v1())*dt;
            const defaulttype::Vector3 pt2v = pt2 + DataTypes::getDPos(t.v2())*dt;
            const defaulttype::Vector3 pt3v = pt3 + DataTypes::getDPos(t.v3())*dt;

            for (int c = 0; c < 3; c++)
            {
                minElem[c] = pt1[c];
                maxElem[c] = pt1[c];
                if (pt2[c] > maxElem[c]) maxElem[c] = pt2[c];
                else if (pt2[c] < minElem[c]) minElem[c] = pt2[c];
                if (pt3[c] > maxElem[c]) maxElem[c] = pt3[c];
                else if (pt3[c] < minElem[c]) minElem[c] = pt3[c];

                if (pt1v[c] > maxElem[c]) maxElem[c] = pt1v[c];
                else if (pt1v[c] < minElem[c]) minElem[c] = pt1v[c];
                if (pt2v[c] > maxElem[c]) maxElem[c] = pt2v[c];
                else if (pt2v[c] < minElem[c]) minElem[c] = pt2v[c];
                if (pt3v[c] > maxElem[c]) maxElem[c] = pt3v[c];
                else if (pt3v[c] < minElem[c]) minElem[c] = pt3v[c];

                minElem[c] -= distance;
                maxElem[c] += distance;
            }

            // Also recompute normal vector
            DataTypes::setDPos(t.n(), cross(pt2-pt1,pt3-pt1).normalized());

            cubeModel->setParentOf(i, minElem, maxElem);
        }
        cubeModel->computeBoundingTree(maxDepth);
    }
}

template<class DataTypes>
TriangleLocalMinDistanceFilter *TTriangleModel<DataTypes>::getFilter() const
{
    return m_lmdFilter;
}


template<class DataTypes>
void TTriangleModel<DataTypes>::setFilter(TriangleLocalMinDistanceFilter *lmdFilter)
{
    m_lmdFilter = lmdFilter;
}

template<class DataTypes>
int TTriangleModel<DataTypes>::getTriangleFlags(int i) const
{
    return triangleFlags[i];
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
