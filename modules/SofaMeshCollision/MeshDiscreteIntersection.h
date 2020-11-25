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
#ifndef SOFA_COMPONENT_COLLISION_MESHDISCRETEINTERSECTION_H
#define SOFA_COMPONENT_COLLISION_MESHDISCRETEINTERSECTION_H

#include <sofa/core/collision/Intersection.h>
#include <sofa/helper/FnDispatcher.h>
#include <SofaBaseCollision/CapsuleModel.h>
#include <SofaBaseCollision/SphereModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaBaseCollision/CubeModel.h>
//#include <SofaVolumetricData/DistanceGridCollisionModel.h>
#include <SofaBaseCollision/DiscreteIntersection.h>
#include <SofaMeshCollision/MeshIntTool.h>

namespace sofa
{

namespace component
{

namespace collision
{
class SOFA_MESH_COLLISION_API MeshDiscreteIntersection : public core::collision::BaseIntersector
{

    template <class Elem1, class Elem2>
    using OutputContainer = sofa::core::collision::TDetectionOutputContainer<typename Elem1::Model, typename Elem2::Model>;

public:
    MeshDiscreteIntersection(DiscreteIntersection* object, bool addSelf=true);

    bool testIntersection(Triangle&, Line&);
    template<class T> bool testIntersection(TSphere<T>&, Triangle&);

    int computeIntersection(Triangle& e1, Line& e2, OutputContainer<Triangle, Line>* contacts);
    template<class T> int computeIntersection(TSphere<T>&, Triangle&, OutputContainer<TSphere<T>, Triangle>*);

    int computeIntersection(Triangle & e1,Capsule & e2, OutputContainer<Triangle, Capsule>* contacts);

    inline int computeIntersection(Capsule & cap,Triangle & tri,OutputContainer<Capsule, Triangle>* contacts);
    inline int computeIntersection(Capsule & cap,Line & lin,OutputContainer<Capsule, Line>* contacts);

    bool testIntersection(Capsule&,Triangle&);
    bool testIntersection(Capsule&,Line&);

protected:

    DiscreteIntersection* intersection;

};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
