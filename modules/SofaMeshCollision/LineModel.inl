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
TLineModel<DataTypes>::~TLineModel()
{
}

template<class DataTypes>
void TLineModel<DataTypes>::init()
{
    Inherit1::init();

    simulation::Node* node = simulation::Node::DynamicCast(this->getContext());
    if (node != 0)
    {
        m_lmdFilter = node->getNodeObject< LineLocalMinDistanceFilter >();
    }

    this->resize(this->m_topology->getNbEdges());

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
    if (nbSamplingValues > 1u && nbSamplingValues != static_cast<std::size_t>(this->size))
    {
        serr << "Size mismatch in classification sampling values, the first will be used for all edges" << sendl;
    }
}

template<class DataTypes>
unsigned int TLineModel<DataTypes>::getClassificationSampling(const unsigned int index) const
{
    sofa::helper::ReadAccessor< sofa::Data<sofa::helper::vector<unsigned> > > classificationSampling = d_classificationSampling;
    unsigned int sampling(1u);

    if (!classificationSampling.empty())
    {
        sampling = (static_cast<std::size_t>(this->size) == classificationSampling.size()) ? classificationSampling[index] : classificationSampling[0];
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
        const defaulttype::Vec<4, float> color(this->getColor4f());
        const SReal proximity = this->getProximity();
        const bool drawCapsule = !(vparams->displayFlags().getShowWireFrame()) && proximity > 0;

        if (drawCapsule)
        {
            vparams->drawTool()->setLightingEnabled(true);
        }

        sofa::helper::vector< defaulttype::Vector3 > points;
        for (int i = 0; i < this->size; i++)
        {
            TLine<DataTypes> l(this,i);
            if(l.activated())
            {
                if (drawCapsule)
                {
                    vparams->drawTool()->drawCapsule(l.p1(), l.p2(), proximity, color);
                }
                else
                {
                    points.push_back(l.p1());
                    points.push_back(l.p2());
                }
            }
        }
        vparams->drawTool()->drawLines(points, 1, color);

        if (m_displayFreePosition.getValue())
        {
            sofa::helper::vector< defaulttype::Vector3 > pointsFree;
            const defaulttype::Vec<4, float> colorFree(0.0f, 1.0f, 0.2f, 1.0f);
            for (int i = 0; i < this->size; i++)
            {
                TLine<DataTypes> l(this,i);
                if(l.activated())
                {
                    if (drawCapsule)
                    {
                        vparams->drawTool()->drawCapsule(l.p1Free(), l.p2Free(), proximity, colorFree);
                    }
                    else
                    {
                        pointsFree.push_back(l.p1Free());
                        pointsFree.push_back(l.p2Free());
                    }
                }
            }
            vparams->drawTool()->drawLines(pointsFree, 1, colorFree);
        }

        if (drawCapsule)
        {
            vparams->drawTool()->setLightingEnabled(false);
        }
    }
    if (this->getPrevious()!=nullptr && vparams->displayFlags().getShowBoundingCollisionModels())
        this->getPrevious()->draw(vparams);
}

template<class DataTypes>
bool TLineModel<DataTypes>::canCollideWithElement(int index, core::CollisionModel* model2, int index2)
{
    // NOTE: same as BaseLineModel, but also looking for common points in the neighborhood

    if (!this->getSelfCollision() || this->getContext() != model2->getContext())
    {
        // Not a self-collision
        return true;
    }

    const core::topology::Topology::Edge& e1 = this->m_topology->getEdge(index);
    int p11 = e1[0];
    int p12 = e1[1];

    const helper::vector <unsigned int>& eav11 = this->m_topology->getEdgesAroundVertex(p11);
    const helper::vector <unsigned int>& eav12 = this->m_topology->getEdgesAroundVertex(p12);

    if (model2->getEnumType() == sofa::core::CollisionModel::LINE_TYPE)
    {
        // do not collide if the edges have a point in common
        const core::topology::Topology::Edge& e2 = this->m_topology->getEdge(index2);
        int p21 = e2[0];
        int p22 = e2[1];

        if (p11 == p21 || p11 == p22 || p12 == p21 || p12 == p22)
            return false;

        // do not collide if a common point is found in the neighborhood
        const helper::vector<unsigned int>& eav21 = this->m_topology->getEdgesAroundVertex(p21);
        const helper::vector<unsigned int>& eav22 = this->m_topology->getEdgesAroundVertex(p22);

        for (unsigned int e11 : eav11)
        {
            for (unsigned int e21 : eav21)
            {
                if (e11 == e21)
                    return false;
            }
            for (unsigned int e22 : eav22)
            {
                if (e11 == e22)
                    return false;
            }
        }

        for (unsigned int e12 : eav12)
        {
            for (unsigned int e21 : eav21)
            {
                if (e12 == e21)
                    return false;
            }
            for (unsigned int e22 : eav22)
            {
                if (e12 == e22)
                    return false;
            }
        }

        return true;
    }
    else if (model2->getEnumType() == sofa::core::CollisionModel::POINT_TYPE)
    {
        // do not collide if the point belongs to the edge
        if (index2 == p11 || index2 == p12)
            return false;

        // do not collide if the point belongs to the neighborhood of the edge
        for (unsigned int e11 : eav11)
        {
            const core::topology::Topology::Edge& edge11 = this->m_topology->getEdge(e11);
            int p111 = edge11[0];
            int p112 = edge11[1];
            if (index2 == p111 || index2 == p112)
                return false;
        }
        for (unsigned int e12 : eav12)
        {
            const core::topology::Topology::Edge& edge12 = this->m_topology->getEdge(e12);
            int p121 = edge12[0];
            int p122 = edge12[1];
            if (index2 == p121 || index2 == p122)
                return false;
        }

        return true;
    }
    else
    {
        return model2->canCollideWithElement(index2, this, index);
    }
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
