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

#ifndef SOFA_COMPONENT_COLLISION_POINTMODEL_INL
#define SOFA_COMPONENT_COLLISION_POINTMODEL_INL

#include "PointModel.h"
#include <SofaMeshCollision/GenericPointModel.inl>
#include <SofaMeshCollision/PointLocalMinDistanceFilter.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/template.h>
#include <sofa/simulation/common/Node.h>

namespace sofa
{

namespace component
{

namespace collision
{

template<class DataTypes>
TPointModel<DataTypes>::TPointModel()
    : d_computeNormals(initData(&d_computeNormals, false, "computeNormals", "activate computation of normal vectors (required for some collision detection algorithms)"))
    , d_displayFreePosition(initData(&d_displayFreePosition, false, "displayFreePosition", "Display Collision Model Points free position(in green)"))
    , d_pointActiverPath(initData(&d_pointActiverPath, "PointActiverPath", "path of a component PointActiver that activate or deactivate collision point during execution"))
{
}

template<class DataTypes>
void TPointModel<DataTypes>::init()
{
    Inherit1::init();

    simulation::Node* node = simulation::Node::DynamicCast(this->getContext());
    if (node != nullptr)
    {
        m_lmdFilter = node->getNodeObject< PointLocalMinDistanceFilter >();
    }

    const int npoints = this->m_mstate->read(core::ConstVecCoordId::position())->getValue().size();
    this->resize(npoints);

    if (d_computeNormals.getValue())
    {
        updateNormals();
    }

    const std::string path = d_pointActiverPath.getValue();
    if (path.empty())
    {
        m_myActiver = PointActiver::getDefaultActiver();
        sout << "Path = " << path << " no Point Activer found for PointModel " << this->getName() << sendl;
    }
    else
    {
        core::objectmodel::BaseObject *activer = nullptr;
        this->getContext()->get(activer , path);
        if (activer != nullptr)
        {
            sout << "Activer named" << activer->getName() << " found" << sendl;
        }
        else
        {
            serr << "Wrong path for PointActiver" << sendl;
        }

        m_myActiver = dynamic_cast<PointActiver*>(activer);
        if (m_myActiver != nullptr)
        {
            sout << "PointActiver named" << activer->getName() << " found for PointModel " << this->getName() << sendl;
        }
        else
        {
            m_myActiver = PointActiver::getDefaultActiver();
            serr << "No dynamic cast possible for Point Activer for PointModel " << this->getName() << sendl;
        }
    }
}

template<class DataTypes>
void TPointModel<DataTypes>::draw(const core::visual::VisualParams*, int index)
{
#ifndef SOFA_NO_OPENGL
    TPoint<DataTypes> p(this,index);
    if (!p.activated())
        return;
    glBegin(GL_POINTS);
    helper::gl::glVertexT(p.p());
    glEnd();
    if ((unsigned)index < m_normals.size())
    {
        glBegin(GL_LINES);
        helper::gl::glVertexT(DataTypes::getCPos(p.p()));
        helper::gl::glVertexT(DataTypes::getCPos(p.p())+m_normals[index]*0.1f);
        glEnd();
    }
#endif /* SOFA_NO_OPENGL */
}

template<class DataTypes>
void TPointModel<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (vparams->displayFlags().getShowCollisionModels())
    {
        if (vparams->displayFlags().getShowWireFrame())
        {
            vparams->drawTool()->setPolygonMode(0, true);
        }

        // Check topological modifications
        const int size = this->getSize();
        const int npoints = this->m_mstate->read(core::ConstVecCoordId::position())->getValue().size();
        if (npoints != size)
        {
            this->resize(npoints);
        }

        sofa::helper::vector< defaulttype::Vector3 > pointsP;
        sofa::helper::vector< defaulttype::Vector3 > pointsL;
        for (int i = 0; i < size; i++)
        {
            TPoint<DataTypes> p(this, i);
            if (p.activated())
            {
                pointsP.push_back(DataTypes::getCPos(p.p()));
                if (i < static_cast<int>(m_normals.size()))
                {
                    pointsL.push_back(DataTypes::getCPos(p.p()));
                    pointsL.push_back(DataTypes::getCPos(p.p()) + m_normals[i]*0.1f);
                }
            }
        }

        vparams->drawTool()->drawPoints(pointsP, 3, defaulttype::Vec<4,float>(this->getColor4f()));
        vparams->drawTool()->drawLines(pointsL, 1, defaulttype::Vec<4,float>(this->getColor4f()));

        if (d_displayFreePosition.getValue())
        {
            sofa::helper::vector< defaulttype::Vector3 > pointsPFree;

            for (int i = 0; i < size; i++)
            {
                TPoint<DataTypes> p(this,i);
                if (p.activated())
                {
                    pointsPFree.push_back(DataTypes::getCPos(p.pFree()));
                }
            }

            vparams->drawTool()->drawPoints(pointsPFree, 3, defaulttype::Vec<4,float>(0.0f,1.0f,0.2f,1.0f));
        }

        if (vparams->displayFlags().getShowWireFrame())
        {
            vparams->drawTool()->setPolygonMode(0, false);
        }
    }

    if (this->getPrevious() != nullptr && vparams->displayFlags().getShowBoundingCollisionModels())
    {
        this->getPrevious()->draw(vparams);
    }
}

template<class DataTypes>
bool TPointModel<DataTypes>::canCollideWithElement(int index, core::CollisionModel* model2, int index2)
{

    if (!this->bSelfCollision.getValue()) return true; // we need to perform this verification process only for the selfcollision case.
    if (this->getContext() != model2->getContext()) return true;

    if (model2 == this)
    {
        if (index <= index2) // to avoid to have two times the same auto-collision we only consider the case when index > index2
        {
            return false;
        }
        
        // in the neighborhood, if we find a point in common, we cancel the collision
        const helper::vector <unsigned int>& verticesAroundVertex1 = this->m_topology->getVerticesAroundVertex(index);
        const helper::vector <unsigned int>& verticesAroundVertex2 = this->m_topology->getVerticesAroundVertex(index2);

        for (unsigned int v1 : verticesAroundVertex1)
        {
            for (unsigned int v2 : verticesAroundVertex2)
            {
                if (v1 == v2 || static_cast<int>(v1) == index2 || static_cast<int>(v2) == index)
                {
                    return false;
                }
            }
        }
        return true;
    }
    else
    {
        return model2->canCollideWithElement(index2, this, index);
    }
}

template<class DataTypes>
void TPointModel<DataTypes>::updateNormals()
{
    const VecCoord& x = this->m_mstate->read(core::ConstVecCoordId::position())->getValue();
    m_normals.resize(x.size());
    for (DPos& n : m_normals)
    {
        n.clear();
    }

    if (this->m_topology->getNbTetrahedra() + this->m_topology->getNbHexahedra() > 0)
    {
        if (this->m_topology->getNbTetrahedra() > 0)
        {
            const core::topology::BaseMeshTopology::SeqTetrahedra& elems = this->m_topology->getTetrahedra();
            for (const core::topology::BaseMeshTopology::Tetra& e : elems)
            {
                const CPos& p1 = DataTypes::getCPos(x[e[0]]);
                const CPos& p2 = DataTypes::getCPos(x[e[1]]);
                const CPos& p3 = DataTypes::getCPos(x[e[2]]);
                const CPos& p4 = DataTypes::getCPos(x[e[3]]);
                DPos& n1 = m_normals[e[0]];
                DPos& n2 = m_normals[e[1]];
                DPos& n3 = m_normals[e[2]];
                DPos& n4 = m_normals[e[3]];
                CPos n;
                n = cross(p3-p1,p2-p1); n.normalize();
                n1 += n;
                n2 += n;
                n3 += n;
                n = cross(p4-p1,p3-p1); n.normalize();
                n1 += n;
                n3 += n;
                n4 += n;
                n = cross(p2-p1,p4-p1); n.normalize();
                n1 += n;
                n4 += n;
                n2 += n;
                n = cross(p3-p2,p4-p2); n.normalize();
                n2 += n;
                n4 += n;
                n3 += n;
            }
        }
        /// @todo Hexahedra
    }
    else if (this->m_topology->getNbTriangles() + this->m_topology->getNbQuads() > 0)
    {
        if (this->m_topology->getNbTriangles()>0)
        {
            const core::topology::BaseMeshTopology::SeqTriangles& elems = this->m_topology->getTriangles();
            for (const core::topology::BaseMeshTopology::Triangle& e : elems)
            {
                const CPos& p1 = DataTypes::getCPos(x[e[0]]);
                const CPos& p2 = DataTypes::getCPos(x[e[1]]);
                const CPos& p3 = DataTypes::getCPos(x[e[2]]);
                DPos& n1 = m_normals[e[0]];
                DPos& n2 = m_normals[e[1]];
                DPos& n3 = m_normals[e[2]];
                CPos n;
                n = cross(p2-p1,p3-p1); n.normalize();
                n1 += n;
                n2 += n;
                n3 += n;
            }
        }
        if (this->m_topology->getNbQuads() > 0)
        {
            const core::topology::BaseMeshTopology::SeqQuads& elems = this->m_topology->getQuads();
            for (const core::topology::BaseMeshTopology::Quad& e : elems)
            {
                const CPos& p1 = DataTypes::getCPos(x[e[0]]);
                const CPos& p2 = DataTypes::getCPos(x[e[1]]);
                const CPos& p3 = DataTypes::getCPos(x[e[2]]);
                const CPos& p4 = DataTypes::getCPos(x[e[3]]);
                DPos& n1 = m_normals[e[0]];
                DPos& n2 = m_normals[e[1]];
                DPos& n3 = m_normals[e[2]];
                DPos& n4 = m_normals[e[3]];
                CPos n;
                n = cross(p3-p1,p4-p2); n.normalize();
                n1 += n;
                n2 += n;
                n3 += n;
                n4 += n;
            }
        }
    }

    for (DPos& n : m_normals)
    {
        const SReal l = n.norm();
        if (l > 1.0e-3)
        {
            n /= l;
        }
        else
        {
            n.clear();
        }
    }
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
