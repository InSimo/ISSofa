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
#ifndef SOFA_COMPONENT_ENGINE_BOXROI_INL
#define SOFA_COMPONENT_ENGINE_BOXROI_INL

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <SofaEngine/BoxROI.h>
#include <SofaEngine/BaseBoxROI.cpp>
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

template <class DataTypes>
BoxROI<DataTypes>::BoxROI() : BaseBoxROI<DataTypes, defaulttype::Vec<6, typename DataTypes::Real> > ()
{}

template <class DataTypes>
bool BoxROI<DataTypes>::isPointInBox(const typename DataTypes::CPos& p, const Vec6& b)
{
    return ( p[0] >= b[0] && p[0] <= b[3] && p[1] >= b[1] && p[1] <= b[4] && p[2] >= b[2] && p[2] <= b[5] );
}

template <class DataTypes>
void BoxROI<DataTypes>::updateBoxes()
{
    helper::vector<Vec6>& vb = *(this->boxes.beginEdit());

    if (vb.empty())
    {
        this->boxes.endEdit();
        return;
    }

    for (unsigned int bi=0; bi<vb.size(); ++bi)
    {
        if (vb[bi][0] > vb[bi][3]) std::swap(vb[bi][0],vb[bi][3]);
        if (vb[bi][1] > vb[bi][4]) std::swap(vb[bi][1],vb[bi][4]);
        if (vb[bi][2] > vb[bi][5]) std::swap(vb[bi][2],vb[bi][5]);
    }

    this->boxes.endEdit();
}


template <class DataTypes>
void BoxROI<DataTypes>::drawBoxes(const core::visual::VisualParams* vparams)
{
    sofa::defaulttype::Vec4f color = sofa::defaulttype::Vec4f(1.0f, 0.4f, 0.4f, 1.0f);

    vparams->drawTool()->setLightingEnabled(false);
    float linesWidth = this->p_drawSize.getValue() ? (float)this->p_drawSize.getValue() : 1;
    sofa::helper::vector<sofa::defaulttype::Vector3> vertices;
    const helper::vector<Vec6>& vb=this->boxes.getValue();
    for (unsigned int bi=0; bi<vb.size(); ++bi)
    {
        const Vec6& b=vb[bi];
        const Real& Xmin=b[0];
        const Real& Xmax=b[3];
        const Real& Ymin=b[1];
        const Real& Ymax=b[4];
        const Real& Zmin=b[2];
        const Real& Zmax=b[5];
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymin,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymin,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymin,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymin,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymin,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymax,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymax,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymax,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymax,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymax,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymax,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymin,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymin,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymin,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymin,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymax,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymin,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymin,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmin,Ymax,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymax,Zmax) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymax,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymin,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymax,Zmin) );
        vertices.push_back( sofa::defaulttype::Vector3(Xmax,Ymax,Zmax) );
        vparams->drawTool()->drawLines(vertices, linesWidth , color );
    }
}


template <class DataTypes>
void BoxROI<DataTypes>::computeBBox(const core::ExecParams*  params)
{
    const helper::vector<Vec6>& vb=this->boxes.getValue(params);
    const Real max_real = std::numeric_limits<Real>::max();
    const Real min_real = std::numeric_limits<Real>::min();
    Real maxBBox[3] = {min_real,min_real,min_real};
    Real minBBox[3] = {max_real,max_real,max_real};

    for (unsigned int bi=0; bi<vb.size(); ++bi)
    {
        const Vec6& b=vb[bi];
        if (b[0] < minBBox[0]) minBBox[0] = b[0];
        if (b[1] < minBBox[1]) minBBox[1] = b[1];
        if (b[2] < minBBox[2]) minBBox[2] = b[2];
        if (b[3] > maxBBox[0]) maxBBox[0] = b[3];
        if (b[4] > maxBBox[1]) maxBBox[1] = b[4];
        if (b[5] > maxBBox[2]) maxBBox[2] = b[5];
    }
    this->f_bbox.setValue(params,sofa::defaulttype::TBoundingBox<Real>(minBBox,maxBBox));
}

} // namespace engine

} // namespace component

} // namespace sofa

#endif
