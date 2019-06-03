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
#ifndef SOFA_COMPONENT_COLLISION_MESHNEWPROXIMITYINTERSECTION_H
#define SOFA_COMPONENT_COLLISION_MESHNEWPROXIMITYINTERSECTION_H

#include <SofaBaseCollision/NewProximityIntersection.h>
#include <sofa/helper/FnDispatcher.h>
#include <SofaBaseCollision/CapsuleModel.h>
#include <SofaBaseCollision/SphereModel.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaBaseCollision/CubeModel.h>
#include <SofaMeshCollision/MeshIntTool.h>
#include <SofaBaseCollision/IntrUtility3.h>
#include <SofaBaseCollision/BaseIntTool.h>

namespace sofa
{

namespace component
{

namespace collision
{

class SOFA_MESH_COLLISION_API MeshNewProximityIntersection : public core::collision::BaseIntersector
{
public:
    template <class Elem1, class Elem2>
    using OutputContainer = sofa::core::collision::TDetectionOutputContainer<typename Elem1::Model, typename Elem2::Model>;


    MeshNewProximityIntersection(NewProximityIntersection* object, bool addSelf=true);

    template <class T1,class T2>
    bool testIntersection(T1 & e1,T2 & e2){
        return BaseIntTool::testIntersection(e1,e2,intersection->getAlarmDistance());
    }


    int computeIntersection(Point&, Point&, OutputContainer<Point, Point>*);
    template <class T> int computeIntersection(TSphere<T>&, Point&, OutputContainer<TSphere<T>, Point>*);
    int computeIntersection(Line&, Point&, OutputContainer<Line, Point>*);
    template <class T> int computeIntersection(Line&, TSphere<T>&, OutputContainer<Line, TSphere<T>>*);
    int computeIntersection(Line&, Line&, OutputContainer<Line, Line>*);
    int computeIntersection(Triangle&, Point&, OutputContainer<Triangle, Point>*);

    template <class T> int computeIntersection(Triangle&, TSphere<T>&, OutputContainer<Triangle, TSphere<T>>*);
    int computeIntersection(Triangle&, Line&, OutputContainer<Triangle, Line>*);

    int computeIntersection(Triangle&, Triangle&, OutputContainer<Triangle, Triangle>*);

    template <class T1,class T2>
    int computeIntersection(T1 & e1,T2 & e2,OutputContainer<T1, T2>* contacts){
        return MeshIntTool::computeIntersection(e1,e2,e1.getProximity() + e2.getProximity() + intersection->getAlarmDistance(),e1.getProximity() + e2.getProximity() + intersection->getContactDistance(),contacts);
    }

    template<class TOutputContainer>
    static inline int doIntersectionLineLine(SReal dist2, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& q1, const defaulttype::Vector3& q2, TOutputContainer* contacts, int id);

    template<class TOutputContainer>
    static inline int doIntersectionLinePoint(SReal dist2, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& q, TOutputContainer* contacts, int id, bool swapElems = false);

    template<class TOutputContainer>
    static inline int doBarycentricIntersectionLinePoint(SReal dist2, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& barycentre, const defaulttype::Vector3& q, TOutputContainer* contacts, int id, bool swapElems = false);

    template<class TOutputContainer>
    static inline int doIntersectionTrianglePoint(SReal dist2, int flags, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& p3, const defaulttype::Vector3& n, const defaulttype::Vector3& q, TOutputContainer* contacts, int id, bool swapElems = false);

    template<class TOutputContainer>
    static inline int doIntersectionTrianglePoint2(SReal dist2, int flags, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& p3, const defaulttype::Vector3& n, const defaulttype::Vector3& q, TOutputContainer* contacts, int id, bool swapElems = false);

protected:

    NewProximityIntersection* intersection;
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif
