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
#ifndef SOFA_COMPONENT_COLLISION_RIGIDDISTANCEGRIDDISCRETEINTERSECTION_INL
#define SOFA_COMPONENT_COLLISION_RIGIDDISTANCEGRIDDISCRETEINTERSECTION_INL
#include <sofa/helper/system/config.h>
#include <SofaVolumetricData/RigidDistanceGridDiscreteIntersection.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/Intersection.inl>
//#include <sofa/component/collision/ProximityIntersection.h>
#include <sofa/helper/proximity.h>
#include <iostream>
#include <algorithm>



namespace sofa
{

namespace component
{

namespace collision
{

template<class TOutputVector>
int RigidDistanceGridDiscreteIntersection::computeIntersection(RigidDistanceGridCollisionElement& e1, Point& e2, TOutputVector* contacts)
{
    DistanceGrid* grid1 = e1.getGrid();
    bool useXForm = e1.isTransformed();
    const Vector3& t1 = e1.getTranslation();
    const Matrix3& r1 = e1.getRotation();
    const bool flipped = e1.isFlipped();

    const double d0 = e1.getProximity() + e2.getProximity() + intersection->getContactDistance();
    const SReal margin = 0.001f + (SReal)d0;


    Vector3 p2 = e2.p();
    DistanceGrid::Coord p1;

    if (useXForm)
    {
        p1 = r1.multTranspose(p2-t1);
    }
    else p1 = p2;

    if (flipped)
    {
        if (!grid1->inGrid( p1 )) return 0;
    }
    else
    {
        if (!grid1->inBBox( p1, margin )) return 0;
        if (!grid1->inGrid( p1 ))
        {
            intersection->serr << "WARNING: margin less than "<<margin<<" in DistanceGrid "<<e1.getCollisionModel()->getName()<<intersection->sendl;
            return 0;
        }
    }

    SReal d = grid1->interp(p1);
    if (flipped) d = -d;
    if (d >= margin) return 0;

    Vector3 grad = grid1->grad(p1); // note that there are some redundant computations between interp() and grad()
    if (flipped) grad = -grad;
    grad.normalize();

    //p1 -= grad * d; // push p1 back to the surface

    DetectionOutput& detection = contacts->addDetectionOutput();

    detection.point[0] = Vector3(p1) - grad * d;
    detection.point[1] = Vector3(p2);
#ifdef DETECTIONOUTPUT_BARYCENTRICINFO
    detection.baryCoords[0] = Vector3(p1);
    detection.baryCoords[1] = Vector3(0,0,0);
#endif
    detection.normal = (useXForm) ? r1 * grad : grad; // normal in global space from p1's surface
    detection.value = d - d0;
    detection.elem.first = e1;
    detection.elem.second = e2;
    detection.id = e2.getIndex();
    return 1;
}

template<class TOutputVector>
int RigidDistanceGridDiscreteIntersection::computeIntersection(RigidDistanceGridCollisionElement& e1, Triangle& e2, TOutputVector* contacts)
{

    const int f2 = e2.flags();
    if (!(f2&(TriangleModel::FLAG_POINTS|TriangleModel::FLAG_BEDGES))) return 0; // no points associated with this triangle
    DistanceGrid* grid1 = e1.getGrid();
    const bool useXForm = e1.isTransformed();
    const Vector3& t1 = e1.getTranslation();
    const Matrix3& r1 = e1.getRotation();

    const double d0 = e1.getProximity() + e2.getProximity() + intersection->getContactDistance();
    const SReal margin = 0.001f + (SReal)d0;
    int nc = 0;
    for (unsigned int iP = 0; iP < 3; ++iP)
    {
        if (!(f2&(TriangleModel::FLAG_P1 << iP))) continue;

        Vector3 p2 = e2.p(iP);
        DistanceGrid::Coord p1;

        if (useXForm)
        {
            p1 = r1.multTranspose(p2-t1);
        }
        else p1 = p2;

        if (grid1->inBBox( p1, margin ))
        {
            if (!grid1->inGrid( p1 ))
            {
                intersection->serr << "WARNING: margin less than "<<margin<<" in DistanceGrid "<<e1.getCollisionModel()->getName()<<intersection->sendl;
            }
            else
            {
                SReal d = grid1->interp(p1);
                if (d >= margin) continue;

                Vector3 grad = grid1->grad(p1); // note that there are some redundant computations between interp() and grad()
                grad.normalize();

                //p1 -= grad * d; // push p1 back to the surface

                DetectionOutput& detection = contacts->addDetectionOutput();

                detection.point[0] = Vector3(p1) - grad * d;
                detection.point[1] = Vector3(p2);
#ifdef DETECTIONOUTPUT_BARYCENTRICINFO
                detection.baryCoords[0] = Vector3(p1);
                detection.baryCoords[1] = Vector3((iP == 1)?1.0:0.0,(iP == 2)?1.0:0.0,0.0);
#endif
                detection.normal = (useXForm) ? r1 * grad : grad; // normal in global space from p1's surface
                detection.value = d - d0;
                detection.elem.first = e1;
                detection.elem.second = e2;
                detection.id = e2.getIndex()*6+iP;

                ++nc;
            }
        }
    }
    for (unsigned int iE = 0; iE < 3; ++iE)
    {
        if (!(f2&(TriangleModel::FLAG_BE23 << iE))) continue;
        unsigned int iP1 = (iE+1)%3;
        unsigned int iP2 = (iE+2)%3;
        Vector3 p2 = (e2.p(iP1)+e2.p(iP2))*0.5;

        DistanceGrid::Coord p1;

        if (useXForm)
        {
            p1 = r1.multTranspose(p2-t1);
        }
        else p1 = p2;

        if (grid1->inBBox( p1, margin ))
        {
            if (!grid1->inGrid( p1 ))
            {
                intersection->serr << "WARNING: margin less than "<<margin<<" in DistanceGrid "<<e1.getCollisionModel()->getName()<<intersection->sendl;
            }
            else
            {
                SReal d = grid1->interp(p1);
                if (d >= margin) continue;

                Vector3 grad = grid1->grad(p1); // note that there are some redundant computations between interp() and grad()
                grad.normalize();

                //p1 -= grad * d; // push p1 back to the surface

                DetectionOutput& detection = contacts->addDetectionOutput();

                detection.point[0] = Vector3(p1) - grad * d;
                detection.point[1] = Vector3(p2);
#ifdef DETECTIONOUTPUT_BARYCENTRICINFO
                detection.baryCoords[0] = Vector3(p1);
                detection.baryCoords[1] = Vector3(((iE != 1)?0.5:0.0),
                                                   ((iE != 2)?0.5:0.0),
                                                   0.0);
#endif
                detection.normal = (useXForm) ? r1 * grad : grad; // normal in global space from p1's surface
                detection.value = d - d0;
                detection.elem.first = e1;
                detection.elem.second = e2;
                detection.id = e2.getIndex()*6+(3+iE);

                ++nc;
            }
        }
    }

    return nc;
}

template<class TOutputVector>
int RigidDistanceGridDiscreteIntersection::computeIntersection(RigidDistanceGridCollisionElement& e1, Line& e2, TOutputVector* contacts)
{
    const int f2 = e2.flags();
    if (!(f2&LineModel::FLAG_POINTS)) return 0; // no points associated with this line
    DistanceGrid* grid1 = e1.getGrid();
    const bool useXForm = e1.isTransformed();
    const Vector3& t1 = e1.getTranslation();
    const Matrix3& r1 = e1.getRotation();

    const double d0 = e1.getProximity() + e2.getProximity() + intersection->getContactDistance();
    const SReal margin = 0.001f + (SReal)d0;
    int nresult = 0;
    for (unsigned int iP = 0; iP < 2; ++iP)
    {
        if (!(f2&(LineModel::FLAG_P1 << iP))) continue;

        Vector3 p2 = e2.p(iP);
        DistanceGrid::Coord p1;

        if (useXForm)
        {
            p1 = r1.multTranspose(p2-t1);
        }
        else p1 = p2;

        if (grid1->inBBox( p1, margin ))
        {
            if (!grid1->inGrid( p1 ))
            {
                intersection->serr << "WARNING: margin less than "<<margin<<" in DistanceGrid "<<e1.getCollisionModel()->getName()<<intersection->sendl;
            }
            else
            {
                SReal d = grid1->interp(p1);
                if (d >= margin) continue;

                Vector3 grad = grid1->grad(p1); // note that there are some redundant computations between interp() and grad()
                grad.normalize();

                //p1 -= grad * d; // push p1 back to the surface

                DetectionOutput& detection = contacts->addDetectionOutput();

                detection.point[0] = Vector3(p1) - grad * d;
                detection.point[1] = Vector3(p2);
#ifdef DETECTIONOUTPUT_BARYCENTRICINFO
                detection.baryCoords[0] = Vector3(p1);
                detection.baryCoords[1] = Vector3((iP == 1)?1.0:0.0,0.0,0.0);
#endif
                detection.normal = (useXForm) ? r1 * grad : grad; // normal in global space from p1's surface
                detection.value = d - d0;
                detection.elem.first = e1;
                detection.elem.second = e2;
                detection.id = e2.getIndex()*2+iP;
                ++nresult;
            }
        }
    }

    return nresult;
}

template<class TOutputVector>
int RigidDistanceGridDiscreteIntersection::computeIntersection(RigidDistanceGridCollisionElement& e1, Sphere& e2, TOutputVector* contacts)
{
    DistanceGrid* grid1 = e1.getGrid();
    bool useXForm = e1.isTransformed();
    const defaulttype::Vector3& t1 = e1.getTranslation();
    const sofa::defaulttype::Matrix3& r1 = e1.getRotation();

    const double d0 = e1.getProximity() + e2.getProximity() + intersection->getContactDistance() + e2.r();
    const SReal margin = 0.001f + (SReal)d0;

    defaulttype::Vector3 p2 = e2.center();
    DistanceGrid::Coord p1;

    if (useXForm)
    {
        p1 = r1.multTranspose(p2-t1);
    }
    else p1 = p2;

    if (!grid1->inBBox( p1, margin )) return 0;
    if (!grid1->inGrid( p1 ))
    {
        intersection->serr << "WARNING: margin less than "<<margin<<" in DistanceGrid "<<e1.getCollisionModel()->getName()<<intersection->sendl;
        return 0;
    }

    SReal d = grid1->interp(p1);
    if (d >= margin) return 0;

    defaulttype::Vector3 grad = grid1->grad(p1); // note that there are some redundant computations between interp() and grad()
    grad.normalize();

    //p1 -= grad * d; // push p1 back to the surface

    DetectionOutput& detection = contacts->addDetectionOutput();
#ifdef DETECTIONOUTPUT_BARYCENTRICINFO
    detection.baryCoords[0] = p1;
    detection.baryCoords[1].clear();
#endif
    detection.normal = (useXForm) ? r1 * grad : grad; // normal in global space from p1's surface
    detection.value = d - d0;
    detection.elem.first = e1;
    detection.elem.second = e2;
    detection.id = e2.getIndex();
    detection.point[0] = defaulttype::Vector3(p1) - grad * d;
    detection.point[1] = e2.getContactPointByNormal( detection.normal );
    return 1;
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
