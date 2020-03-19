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
#ifndef SOFA_COMPONENT_ENGINE_ORIENTEDBOXROI_INL
#define SOFA_COMPONENT_ENGINE_ORIENTEDBOXROI_INL

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <SofaEngine/OrientedBoxROI.h>
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
OrientedBoxROI<DataTypes>::OrientedBoxROI()
    : BaseBoxROI<DataTypes, OrientedBoxROIType > ()
    , d_centers( initData(&d_centers, "centers", "Center(s) of the Box(es)") )
    , d_dimensions( initData(&d_dimensions, "dimensions", "Height(s), length(s) and width(s) of the Box(es)") )
    , d_quaternions( initData(&d_quaternions, "quaternions", "Quaternion(s) giving Box(es) orientation(s)") )
{
    this->boxes.setReadOnly(true);
}


template <class DataTypes>
void OrientedBoxROI<DataTypes>::init()
{
    helper::WriteAccessor<Data<Boxes>> vb = this->boxes;

    helper::ReadAccessor<Data<sofa::helper::vector<Vec3>>> centers = d_centers;
    helper::ReadAccessor<Data<sofa::helper::vector<Vec3>>> dimensions = d_dimensions;
    helper::ReadAccessor<Data<sofa::helper::vector<Quat>>> quaternions = d_quaternions;

    assert((centers.size() == dimensions.size()) && (centers.size() == quaternions.size()));
    for (unsigned int i=0; i<centers.size(); ++i)
    {
        vb.push_back(OrientedBoxROIType(centers[i], dimensions[i], quaternions[i]));
    }

    Inherit1::init();
}


template <class DataTypes>
bool OrientedBoxROI<DataTypes>::isPointInBox(const typename DataTypes::CPos& p, const OrientedBoxROIType& b)
{
    const auto invRotp = b.quat.inverseRotate(Vec3(p) - b.center);
    if (std::fabs(invRotp[0]) <= b.dimensions[0]*0.5 && std::fabs(invRotp[1]) <= b.dimensions[1]*0.5 &&
        std::fabs(invRotp[2]) <= b.dimensions[2]*0.5)
    {
        return true;
    }

    return false;
}

template <class DataTypes>
void OrientedBoxROI<DataTypes>::updateBoxes()
{
    helper::WriteAccessor<Data<Boxes>> vb = this->boxes;

    helper::ReadAccessor<Data<sofa::helper::vector<Vec3>>> centers = d_centers;
    helper::ReadAccessor<Data<sofa::helper::vector<Vec3>>> dimensions = d_dimensions;
    helper::ReadAccessor<Data<sofa::helper::vector<Quat>>> quaternions = d_quaternions;

    if (vb.empty())
        return;

    for (unsigned int bi=0; bi<vb.size(); ++bi)
    {
        vb[bi].center = centers[bi];
        vb[bi].dimensions = dimensions[bi];
        vb[bi].quat = quaternions[bi];
    }
}


template <class DataTypes>
void OrientedBoxROI<DataTypes>::drawBoxes(const core::visual::VisualParams* vparams)
{
    sofa::defaulttype::Vec4f color = sofa::defaulttype::Vec4f(1.0f, 0.4f, 0.4f, 1.0f);

    vparams->drawTool()->setLightingEnabled(false);
    float linesWidth = this->p_drawSize.getValue() ? (float)this->p_drawSize.getValue() : 1;
    const helper::vector<OrientedBoxROIType>& vb = this->boxes.getValue();

    for (unsigned int bi=0; bi<vb.size(); ++bi)
    {
        const OrientedBoxROIType& b=vb[bi];

        Quat quater = b.quat;
        quater.normalize();

        sofa::helper::vector<Vec3> vertices;
        const Vec3& dx =  0.5*quater.rotate(Vec3(b.dimensions[0], 0, 0));
        const Vec3& dy =  0.5*quater.rotate(Vec3(0, b.dimensions[1], 0));
        const Vec3& dz =  0.5*quater.rotate(Vec3(0, 0, b.dimensions[2]));

        Vec3 p0 = b.center - dx - dy - dz;
        Vec3 p1 = b.center + dx - dy - dz;

        Vec3 p2 = b.center + dx + dy - dz;
        Vec3 p3 = b.center - dx + dy - dz;

        Vec3 p4 = b.center - dx - dy + dz;
        Vec3 p5 = b.center + dx - dy + dz;

        Vec3 p6 = b.center + dx + dy + dz;
        Vec3 p7 = b.center - dx + dy + dz;

        vertices.push_back(p0);
        vertices.push_back(p1);
        vertices.push_back(p1);
        vertices.push_back(p2);
        vertices.push_back(p2);
        vertices.push_back(p3);
        vertices.push_back(p3);
        vertices.push_back(p0);

        vertices.push_back(p4);
        vertices.push_back(p5);
        vertices.push_back(p5);
        vertices.push_back(p6);
        vertices.push_back(p6);
        vertices.push_back(p7);
        vertices.push_back(p7);
        vertices.push_back(p4);

        vertices.push_back(p0);
        vertices.push_back(p4);
        vertices.push_back(p1);
        vertices.push_back(p5);
        vertices.push_back(p2);
        vertices.push_back(p6);
        vertices.push_back(p3);
        vertices.push_back(p7);

        vparams->drawTool()->drawLines(vertices, linesWidth, color);
    }
}


template <class DataTypes>
void OrientedBoxROI<DataTypes>::computeBBox(const core::ExecParams*  params)
{
    sofa::helper::ReadAccessor<Data<Boxes>> vb = this->boxes;
    const Real max_real = std::numeric_limits<Real>::max();
    const Real min_real = std::numeric_limits<Real>::min();
    Real maxBBox[3] = {min_real,min_real,min_real};
    Real minBBox[3] = {max_real,max_real,max_real};

    for (unsigned int bi=0; bi<vb.size(); ++bi)
    {
        const OrientedBoxROIType& b=vb[bi];

        Quat quater = b.quat;
        quater.normalize();

        sofa::helper::vector<Vec3> vertices;
        const Vec3& dx =  0.5*quater.rotate(Vec3(b.dimensions[0], 0, 0));
        const Vec3& dy =  0.5*quater.rotate(Vec3(0, b.dimensions[1], 0));
        const Vec3& dz =  0.5*quater.rotate(Vec3(0, 0, b.dimensions[2]));

        for (unsigned int i=0; i<8; i++)
        {
            Vec3 vertex = b.center + std::pow(-1, i%2)*dx
                                   + std::pow(-1, ((i/2)%2))*dy
                                   + std::pow(-1, ((i/4)%2))*dz;

            if (vertex[0] < minBBox[0]) minBBox[0] = vertex[0];
            if (vertex[1] < minBBox[1]) minBBox[1] = vertex[1];
            if (vertex[2] < minBBox[2]) minBBox[2] = vertex[2];
            if (vertex[0] > maxBBox[0]) maxBBox[0] = vertex[0];
            if (vertex[1] > maxBBox[1]) maxBBox[1] = vertex[1];
            if (vertex[2] > maxBBox[2]) maxBBox[2] = vertex[2];
        }
    }
    this->f_bbox.setValue(params,sofa::defaulttype::TBoundingBox<Real>(minBBox,maxBBox));
}

} // namespace engine

} // namespace component

} // namespace sofa

#endif
