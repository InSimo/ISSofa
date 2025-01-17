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
#ifndef SOFA_COMPONENT_COLLISION_RIGIDDISTANCEGRIDDISCRETEINTERSECTION_H
#define SOFA_COMPONENT_COLLISION_RIGIDDISTANCEGRIDDISCRETEINTERSECTION_H

#include <sofa/core/collision/Intersection.h>
#include <sofa/helper/FnDispatcher.h>
#include <SofaBaseCollision/SphereModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaBaseCollision/CubeModel.h>
#include <SofaUserInteraction/RayModel.h>
#include <SofaVolumetricData/DistanceGridCollisionModel.h>
#include <SofaBaseCollision/DiscreteIntersection.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/collision/DetectionOutput.h>

namespace sofa
{

namespace component
{

namespace collision
{
class SOFA_VOLUMETRIC_DATA_API RigidDistanceGridDiscreteIntersection : public core::collision::BaseIntersector
{

    template <class Elem1, class Elem2>
    using OutputContainer = DiscreteIntersection::OutputContainer<Elem1, Elem2>;
    typedef defaulttype::Matrix3 Matrix3;
    typedef defaulttype::Vector3 Vector3;
    typedef core::collision::DetectionOutput DetectionOutput;

public:
    RigidDistanceGridDiscreteIntersection(DiscreteIntersection* object);

    bool testIntersection(RigidDistanceGridCollisionElement&, RigidDistanceGridCollisionElement&);
    bool testIntersection(RigidDistanceGridCollisionElement&, Point&);
    bool testIntersection(RigidDistanceGridCollisionElement&, Sphere&);
    bool testIntersection(RigidDistanceGridCollisionElement&, Line&);
    bool testIntersection(RigidDistanceGridCollisionElement&, Triangle&);
    bool testIntersection(Ray&, RigidDistanceGridCollisionElement&);

    int computeIntersection(RigidDistanceGridCollisionElement&, RigidDistanceGridCollisionElement&, OutputContainer<RigidDistanceGridCollisionElement, RigidDistanceGridCollisionElement>*);
    template<class TOutputVector> int computeIntersection(RigidDistanceGridCollisionElement&, Point&, TOutputVector*);
    template<class TOutputVector> int computeIntersection(RigidDistanceGridCollisionElement&, Sphere&, TOutputVector*);
    template<class TOutputVector> int computeIntersection(RigidDistanceGridCollisionElement&, Line&, TOutputVector*);
    template<class TOutputVector> int computeIntersection(RigidDistanceGridCollisionElement&, Triangle&, TOutputVector*);
    int computeIntersection(Ray&, RigidDistanceGridCollisionElement&, OutputContainer<Ray, RigidDistanceGridCollisionElement>*);

protected:

    DiscreteIntersection* intersection;

};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
