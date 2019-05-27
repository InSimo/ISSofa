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
#include <SofaMeshCollision/Triangle.h>
#include <SofaBaseTopology/TopologyData.inl>
#include <sofa/simulation/common/Node.h>
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
    , d_minTriangleArea(initData(&d_minTriangleArea, Real(1.0e-6), "minTriangleArea", "Triangle area threshold below which elements are considered as badly shaped and collisions are disabled"))
    , d_drawBoundaryPoints(initData(&d_drawBoundaryPoints, false, "drawBoundaryPoints", "Draw triangle points that are classified as boundary."))
    , d_drawBoundaryEdges(initData(&d_drawBoundaryEdges, false, "drawBoundaryEdges", "Draw triangle edges that are classified as boundary."))
{
    triangles = &mytriangles;
}

template<class DataTypes>
void TTriangleModel<DataTypes>::resize(int size)
{
    Inherit1::resize(size);
    normals.resize(size);
}

template<class DataTypes>
void TTriangleModel<DataTypes>::init()
{
    Inherit1::init();

    this->getContext()->get(mpoints);

    simulation::Node* node = simulation::Node::DynamicCast(this->getContext());
    if (node != 0)
    {
        m_lmdFilter = node->getNodeObject< TriangleLocalMinDistanceFilter >();
    }

    //sout << "INFO_print : Col - init TRIANGLE " << sendl;
    sout << "TriangleModel: initially "<<this->m_topology->getNbTriangles()<<" triangles." << sendl;
    triangles = &this->m_topology->getTriangles();
    resize(this->m_topology->getNbTriangles());

    updateFromTopology();
    updateNormals();
}

template<class DataTypes>
void TTriangleModel<DataTypes>::reinit()
{
    this->updateTopologicalTriangleFlags();
    this->updateMechanicalTriangleFlags();
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
    for (int i = 0; i < this->size; i++)
    {
        Element t(this,i);
        t.n() = computeTriangleNormal<DataTypes>(t.p1(), t.p2(), t.p3());
    }
}

template<class DataTypes>
void TTriangleModel<DataTypes>::updateFromTopology()
{
    //    needsUpdate = false;
    const unsigned ntris = this->m_topology->getNbTriangles();
    const unsigned nquads = this->m_topology->getNbQuads();
    const unsigned newsize = ntris+2*nquads;

    int revision = this->m_topology->getRevision();
    if (newsize==(unsigned)this->size && revision == meshRevision)
        return;
    meshRevision = revision;

    needsUpdate = true;

    resize(newsize);

    unsigned npoints = this->m_mstate->read(core::ConstVecCoordId::position())->getValue().size();
    if (npoints != (unsigned)this->m_topology->getNbPoints())
    {
        serr << "Mismatch between number of points in topology (" << this->m_topology->getNbPoints()
             << ") and size of mstate positions vector (" << npoints << ")" << sendl;
        npoints = (unsigned)this->m_topology->getNbPoints();
        this->m_mstate->resize(npoints);
    }

    if (newsize == ntris)
    {
        // no need to copy the triangle indices
        triangles = & this->m_topology->getTriangles();
    }
    else
    {
        triangles = &mytriangles;
        mytriangles.resize(newsize);
        int index = 0;
        for (unsigned i=0; i<ntris; i++)
        {
            topology::BaseMeshTopology::Triangle idx = this->m_topology->getTriangle(i);
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
            topology::BaseMeshTopology::Quad idx = this->m_topology->getQuad(i);
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
    this->updateTopologicalTriangleFlags();
}

template<class DataTypes>
void TTriangleModel<DataTypes>::handleTopologyChange(sofa::core::topology::Topology* t)
{
    if (t != this->m_topology) return;

    //bool debug_mode = false;
    if (triangles != &mytriangles)
    {
        // We use the same triangle array as the topology -> only resize and recompute flags

        std::list<const sofa::core::topology::TopologyChange *>::const_iterator itBegin=this->m_topology->beginChange();
        std::list<const sofa::core::topology::TopologyChange *>::const_iterator itEnd=this->m_topology->endChange();

        while( itBegin != itEnd )
        {
            core::topology::TopologyChangeType changeType = (*itBegin)->getChangeType();

            switch( changeType )
            {
                case core::topology::ENDING_EVENT:
                {
                    this->m_hasTopologicalChange = true;
                    updateFromTopology();
                }
                // fallthrough
                case sofa::core::topology::TRIANGLESREMOVED:
                case sofa::core::topology::TRIANGLESADDED:
                case sofa::core::topology::QUADSREMOVED:
                case sofa::core::topology::QUADSADDED:
                {
                    sout << "TriangleModel: now " << this->m_topology->getNbTriangles() << " triangles." << sendl;
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
        const bool drawBPoints = d_drawBoundaryPoints.getValue();
        const bool drawBEdges = d_drawBoundaryEdges.getValue();

        if (this->bBothSide.getValue() || vparams->displayFlags().getShowWireFrame())
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
        int index = 0;
        for (int i = 0; i < this->size; i++)
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
                    if (f & Inherit1::FLAG_BP1 << j)
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
                    if (f & Inherit1::FLAG_BE23 << j)
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
        vparams->drawTool()->drawTriangles(points, indices, normals, defaulttype::Vec<4,float>(this->getColor4f()));
        vparams->drawTool()->setLightingEnabled(false);
        vparams->drawTool()->setPolygonMode(0,false);

        if (drawBPoints) vparams->drawTool()->drawPoints(pointsBP, 10.0, defaulttype::Vec<4,float>(1,0,0,1));
        if (drawBEdges) vparams->drawTool()->drawLines(pointsBE, 1.0, defaulttype::Vec<4,float>(1,0,0,1));

        if (vparams->displayFlags().getShowNormals())
        {
            sofa::helper::vector< defaulttype::Vector3 > pointsEdges;
            for (int i = 0; i < this->size; i++)
            {
                Element t(this,i);
                pointsEdges.push_back(DataTypes::getCPos((t.p1()+t.p2()+t.p3()))/3.0);
                pointsEdges.push_back(pointsEdges.back()+DataTypes::getDPos(t.n()));
            }

            vparams->drawTool()->drawLines(pointsEdges, 1, defaulttype::Vec<4,float>(1,1,1,1));
        }
    }

    if (this->getPrevious()!=NULL && vparams->displayFlags().getShowBoundingCollisionModels())
    {
        this->getPrevious()->draw(vparams);
    }
}


template<class DataTypes>
bool TTriangleModel<DataTypes>::canCollideWithElement(int index, core::CollisionModel* model2, int index2)
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
defaulttype::BoundingBox TTriangleModel<DataTypes>::computeElementBBox(int index, SReal distance)
{
    const VecCoord& x = this->m_mstate->read(core::ConstVecCoordId::position())->getValue();

    Element t(this, index);
    const defaulttype::Vector3& pt1 = DataTypes::getCPos(x[t.p1Index()]);
    const defaulttype::Vector3& pt2 = DataTypes::getCPos(x[t.p2Index()]);
    const defaulttype::Vector3& pt3 = DataTypes::getCPos(x[t.p3Index()]);

    defaulttype::BoundingBox bbox;
    bbox.include(pt1);
    bbox.include(pt2);
    bbox.include(pt3);
    bbox.inflate(distance);

    return bbox;
}

template<class DataTypes>
defaulttype::BoundingBox TTriangleModel<DataTypes>::computeElementBBox(int index, SReal distance, double dt)
{
    const VecCoord& x = this->m_mstate->read(core::ConstVecCoordId::position())->getValue();

    Element t(this, index);
    const defaulttype::Vector3& pt1 = DataTypes::getCPos(x[t.p1Index()]);
    const defaulttype::Vector3& pt2 = DataTypes::getCPos(x[t.p2Index()]);
    const defaulttype::Vector3& pt3 = DataTypes::getCPos(x[t.p3Index()]);
    const defaulttype::Vector3 pt1v = pt1 + DataTypes::getDPos(t.v1())*dt;
    const defaulttype::Vector3 pt2v = pt2 + DataTypes::getDPos(t.v2())*dt;
    const defaulttype::Vector3 pt3v = pt3 + DataTypes::getDPos(t.v3())*dt;

    defaulttype::BoundingBox bbox;
    bbox.include(pt1);
    bbox.include(pt2);
    bbox.include(pt3);
    bbox.include(pt1v);
    bbox.include(pt2v);
    bbox.include(pt3v);
    bbox.inflate(distance);

    return bbox;
}

template<class DataTypes>
void TTriangleModel<DataTypes>::computeBoundingTree(int maxDepth)
{
    if (this->isMoving() || this->m_hasTopologicalChange)
    {
        const VecCoord& x = this->m_mstate->read(core::ConstVecCoordId::position())->getValue();

        const bool calcNormals = computeNormals.getValue();
        const Real minTriangleArea = d_minTriangleArea.getValue();
        const Real minTriangleArea2 = minTriangleArea*minTriangleArea;
        const bool updateBadShape = minTriangleArea != Real(0.0);

        for (int i = 0; i < this->size; i++)
        {
            Element t(this,i);
            const defaulttype::Vector3& pt1 = DataTypes::getCPos(x[t.p1Index()]);
            const defaulttype::Vector3& pt2 = DataTypes::getCPos(x[t.p2Index()]);
            const defaulttype::Vector3& pt3 = DataTypes::getCPos(x[t.p3Index()]);

            if (updateBadShape)
            {
                const Real triangleArea2 = computeTriangleAreaSquared(pt1,pt2,pt3);
                if (triangleArea2 < minTriangleArea2)
                {
                    this->m_triangleFlags[i] |= FLAG_BADSHAPE;
                    if (m_countBadShape++ == 0)
                    { // show a warning, but only the first time a triangle is flagged in this model
                        serr << "Triangle with index " << i << " (area = " << sqrt(triangleArea2)
                             << ") is badly shaped: collision detection within this triangle will be disabled." << sendl;
                    }
                }
                else
                {
                    this->m_triangleFlags[i] &= ~FLAG_BADSHAPE;
                }
            }
            if (calcNormals)
            {
                // Also recompute normal vector
                t.n() = computeTriangleNormal<DataTypes>(x[t.p1Index()], x[t.p2Index()], x[t.p3Index()]);
            }
        }
    }

    Inherit1::computeBoundingTree(maxDepth);

    if (m_lmdFilter != 0)
    {
        m_lmdFilter->invalidate();
    }
}

template<class DataTypes>
void TTriangleModel<DataTypes>::computeContinuousBoundingTree(double dt, int maxDepth)
{
    if (this->isMoving() || this->m_hasTopologicalChange)
    {
        const VecCoord& x = this->m_mstate->read(core::ConstVecCoordId::position())->getValue();

        const bool calcNormals = computeNormals.getValue();

        for (int i = 0; i < this->size; i++)
        {
            Element t(this,i);
            if (calcNormals)
            {
                // Also recompute normal vector
                t.n() = computeTriangleNormal<DataTypes>(x[t.p1Index()], x[t.p2Index()], x[t.p3Index()]);
            }
        }
    }

    Inherit1::computeContinuousBoundingTree(dt, maxDepth);
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

} // namespace collision

} // namespace component

} // namespace sofa

#endif
