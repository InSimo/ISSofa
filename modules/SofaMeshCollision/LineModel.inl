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

#ifndef SOFA_COMPONENT_COLLISION_LINEMODEL_INL
#define SOFA_COMPONENT_COLLISION_LINEMODEL_INL

#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/GenericLineModel.inl>
#include <sofa/core/visual/VisualParams.h>
#include <SofaMeshCollision/LineLocalMinDistanceFilter.h>
#include <SofaBaseCollision/CubeModel.h>
#include <SofaMeshCollision/Line.h>
#include <sofa/core/CollisionElement.h>
#include <vector>
#include <sofa/helper/gl/template.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/core/topology/TopologyChange.h>
#include <sofa/simulation/common/Simulation.h>

namespace sofa
{

namespace component
{

namespace collision
{

using core::topology::BaseMeshTopology;

template<class DataTypes>
TLineModel<DataTypes>::TLineModel()
    : m_lmdFilter(nullptr)
    , LineActiverPath(initData(&LineActiverPath,"LineActiverPath", "path of a component LineActiver that activates or deactivates collision line during execution") )
    , m_displayFreePosition(initData(&m_displayFreePosition, false, "displayFreePosition", "Display Collision Model Points free position(in green)") )
    , d_classificationSampling(initData(&d_classificationSampling, "classificationSampling", "Line sub sampling factor per edge to create additional contacts for classification"))
    , myActiver(nullptr)
{
}

template<class DataTypes>
void TLineModel<DataTypes>::init()
{
    Inherit1::init();

    this->getContext()->get(mpoints);

    simulation::Node* node = simulation::Node::DynamicCast(this->getContext());
    if (node != 0)
    {
        m_lmdFilter = node->getNodeObject< LineLocalMinDistanceFilter >();
    }

    resize(this->m_topology->getNbEdges());

    const std::string path = LineActiverPath.getValue();

    if (path.size()==0)
    {

        myActiver = LineActiver::getDefaultActiver();
        sout<<"path = "<<path<<" no Line Activer found for LineModel "<<this->getName()<<sendl;
    }
    else
    {
        core::objectmodel::BaseObject *activer=nullptr;
        this->getContext()->get(activer ,path  );

        if (activer != nullptr)
            sout<<" Activer named"<<activer->getName()<<" found"<<sendl;
        else
            serr<<"wrong path for Line Activer"<<sendl;

        myActiver = dynamic_cast<LineActiver*>(activer);
        if (myActiver==nullptr)
        {
            myActiver = LineActiver::getDefaultActiver();
            serr<<"wrong path for Line Activer for LineModel "<< this->getName() <<sendl;
        }
        else
        {
            sout<<"Line Activer named"<<activer->getName()<<" found !! for LineModel "<< this->getName() <<sendl;
        }
    }

    const std::size_t nbSamplingValues = d_classificationSampling.getValue().size();
    if (nbSamplingValues > 1u && nbSamplingValues != this->elems.size())
        serr << "Size mismatch in classification sampling values, the first will be used for all edges" << sendl;
}

template<class DataTypes>
void TLineModel<DataTypes>::resize(int size)
{
    Inherit1::resize(size);
    elems.resize(size);
}

template<class DataTypes>
unsigned int TLineModel<DataTypes>::getClassificationSampling(const unsigned int index) const
{
    sofa::helper::ReadAccessor< sofa::Data<sofa::helper::vector<unsigned> > > classificationSampling = d_classificationSampling;
    unsigned int sampling(1u);

    if (!classificationSampling.empty())
    {
        sampling = (elems.size() == classificationSampling.size()) ? classificationSampling[index] : classificationSampling[0];
    }

    return sampling;
}

template<class DataTypes>
void TLineModel<DataTypes>::draw(const core::visual::VisualParams* ,int index)
{
#ifndef SOFA_NO_OPENGL
    TLine<DataTypes> l(this,index);
    if (!l.activated())
        return;
    glBegin(GL_LINES);
    helper::gl::glVertexT(l.p1());
    helper::gl::glVertexT(l.p2());
    glEnd();
#endif /* SOFA_NO_OPENGL */
}

template<class DataTypes>
void TLineModel<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (vparams->displayFlags().getShowCollisionModels())
    {
        if (vparams->displayFlags().getShowWireFrame())
            vparams->drawTool()->setPolygonMode(0,true);

        sofa::helper::vector< defaulttype::Vector3 > points;
        for (int i = 0; i < this->size; i++)
        {
            TLine<DataTypes> l(this,i);
            if(l.activated())
            {
                points.push_back(l.p1());
                points.push_back(l.p2());
            }
        }

        vparams->drawTool()->drawLines(points, 1, defaulttype::Vec<4,float>(this->getColor4f()));

        if (m_displayFreePosition.getValue())
        {
            sofa::helper::vector< defaulttype::Vector3 > pointsFree;
            for (int i = 0; i < this->size; i++)
            {
                TLine<DataTypes> l(this,i);
                if(l.activated())
                {
                    pointsFree.push_back(l.p1Free());
                    pointsFree.push_back(l.p2Free());
                }
            }

            vparams->drawTool()->drawLines(pointsFree, 1, defaulttype::Vec<4,float>(0.0f,1.0f,0.2f,1.0f));
        }

        if (vparams->displayFlags().getShowWireFrame())
            vparams->drawTool()->setPolygonMode(0,false);
    }
    if (this->getPrevious()!=nullptr && vparams->displayFlags().getShowBoundingCollisionModels())
        this->getPrevious()->draw(vparams);
}

template<class DataTypes>
bool TLineModel<DataTypes>::canCollideWithElement(int index, core::CollisionModel* model2, int index2)
{
    if (!this->bSelfCollision.getValue()) return true;
    if (this->getContext() != model2->getContext()) return true;
    sofa::core::topology::BaseMeshTopology* topology = this->getMeshTopology();
    /*
    	TODO : separate 2 case: the model is only composed of lines or is composed of triangles
    	bool NoTriangles = true;
        if( this->getContext()->get<TriangleModel>  != nullptr)
    		NoTriangles = false;
    */
    const core::topology::Topology::Edge& e1 = this->m_topology->getEdge(index);
    int p11 = e1[0];
    int p12 = e1[1];

    if (!topology)
    {
        serr<<"no topology found"<<sendl;
        return true;
    }
    const helper::vector <unsigned int>& eav11 =topology->getEdgesAroundVertex(p11);
    const helper::vector <unsigned int>& eav12 =topology->getEdgesAroundVertex(p12);

    if (model2 == this)
    {
        // if point in common, return false:
        const core::topology::Topology::Edge& e2 = this->m_topology->getEdge(index2);
        int p21 = e2[0];
        int p22 = e2[1];

        if (p11==p21 || p11==p22 || p12==p21 || p12==p22)
            return false;


        // in the neighborhood, if we find a segment in common, we cancel the collision
        const helper::vector <unsigned int>& eav21 =topology->getEdgesAroundVertex(p21);
        const helper::vector <unsigned int>& eav22 =topology->getEdgesAroundVertex(p22);

        for (unsigned int i1=0; i1<eav11.size(); i1++)
        {
            unsigned int e11 = eav11[i1];
            unsigned int i2;
            for (i2=0; i2<eav21.size(); i2++)
            {
                if (e11==eav21[i2])
                    return false;
            }
            for (i2=0; i2<eav22.size(); i2++)
            {
                if (e11==eav22[i2])
                    return false;
            }
        }

        for (unsigned int i1=0; i1<eav12.size(); i1++)
        {
            unsigned int e11 = eav12[i1];
            unsigned int i2;
            for (i2=0; i2<eav21.size(); i2++)
            {
                if (e11==eav21[i2])
                    return false;
            }
            for (i2=0; i2<eav22.size(); i2++)
            {
                if (e11==eav22[i2])
                    return false;
            }

        }
        return true;
    }
    else if (model2 == mpoints)
    {
        //std::cerr<<" point Model"<<std::endl;

        // if point belong to the segment, return false
        if (index2==p11 || index2==p12)
            return false;

        // if the point belong to the a segment in the neighborhood, return false
        for (unsigned int i1=0; i1<eav11.size(); i1++)
        {
            const core::topology::Topology::Edge& e11 = this->m_topology->getEdge(eav11[i1]);
            int p11 = e11[0];
            int p12 = e11[1];
            if (index2==p11 || index2==p12)
                return false;
        }
        for (unsigned int i1=0; i1<eav12.size(); i1++)
        {
            const core::topology::Topology::Edge& e12 = this->m_topology->getEdge(eav12[i1]);
            int p11 = e12[0];
            int p12 = e12[1];
            if (index2==p11 || index2==p12)
                return false;
        }
        return true;

        //sout << "line-point self test "<<index<<" - "<<index2<<sendl;
        //std::cout << "line-point self test "<<index<<" - "<<index2<<"   - elems[index].p[0]-1"<<elems[index].p[0]-1<<"   elems[index]..p[1]+1 "<<elems[index].p[1]+1<<std::endl;


        // case1: only lines (aligned lines !!)
        //return index2 < p11-1 || index2 > p12+1;

        // only removes collision with the two vertices of the segment
        // TODO: neighborhood search !
        //return !(index2==p11 || index2==p12);
    }
    else
        return model2->canCollideWithElement(index2, this, index);
}

template<class DataTypes>
defaulttype::BoundingBox TLineModel<DataTypes>::computeElementBBox(int index, SReal distance)
{
    Element l(this, index);
    const defaulttype::Vector3& pt1 = l.p1();
    const defaulttype::Vector3& pt2 = l.p2();

    defaulttype::BoundingBox bbox;
    bbox.include(pt1);
    bbox.include(pt2);
    bbox.inflate(distance);

    return bbox;
}

template<class DataTypes>
defaulttype::BoundingBox TLineModel<DataTypes>::computeElementBBox(int index, SReal distance, double dt)
{
    Element l(this, index);
    const defaulttype::Vector3& pt1 = l.p1();
    const defaulttype::Vector3& pt2 = l.p2();
    const defaulttype::Vector3 pt1v = pt1 + l.v1()*dt;
    const defaulttype::Vector3 pt2v = pt2 + l.v2()*dt;

    defaulttype::BoundingBox bbox;
    bbox.include(pt1);
    bbox.include(pt2);
    bbox.include(pt1v);
    bbox.include(pt2v);
    bbox.inflate(distance);

    return bbox;
}

template<class DataTypes>
void TLineModel<DataTypes>::computeBoundingTree(int maxDepth)
{
    Inherit1::computeBoundingTree(maxDepth);

    if (m_lmdFilter != 0)
    {
        m_lmdFilter->invalidate();
    }
}

template<class DataTypes>
void TLineModel<DataTypes>::updateTopologicalLineFlags()
{
    Inherit1::updateTopologicalLineFlags();

    const int nbEdges = this->m_topology->getNbEdges();
    elems.resize(nbEdges);
    for (int i = 0u; i < nbEdges; ++i)
    {
        LineData& elem = elems[i];
        const core::topology::Topology::Edge& e = this->m_topology->getEdge(i);
        elem.p[0] = e[0];
        elem.p[1] = e[1];
    }
}

template<class DataTypes>
LineLocalMinDistanceFilter *TLineModel<DataTypes>::getFilter() const
{
    return m_lmdFilter;
}

template<class DataTypes>
void TLineModel<DataTypes>::setFilter(LineLocalMinDistanceFilter *lmdFilter)
{
    m_lmdFilter = lmdFilter;
}


} // namespace collision

} // namespace component

} // namespace sofa

#endif
