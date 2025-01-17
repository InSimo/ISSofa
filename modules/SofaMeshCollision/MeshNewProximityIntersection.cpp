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
#include <SofaMeshCollision/MeshNewProximityIntersection.inl>
#include <sofa/helper/system/config.h>
#include <sofa/helper/FnDispatcher.inl>
#include <sofa/core/collision/Intersection.inl>
//#include <sofa/component/collision/ProximityIntersection.h>
#include <sofa/helper/proximity.h>
#include <iostream>
#include <algorithm>
#include <sofa/core/collision/IntersectorFactory.h>


namespace sofa
{

namespace component
{

namespace collision
{

SOFA_DECL_CLASS(MeshNewProximityIntersection)

using namespace sofa::defaulttype;
using namespace sofa::core::collision;

IntersectorCreator<NewProximityIntersection, MeshNewProximityIntersection> MeshNewProximityIntersectors("Mesh");

MeshNewProximityIntersection::MeshNewProximityIntersection(NewProximityIntersection* object, bool addSelf)
    : intersection(object)
{
    if (addSelf)
    {
        intersection->intersectors.add<PointModel, PointModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<SphereModel, PointModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<LineModel, PointModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<LineModel, SphereModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<LineModel, LineModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<TriangleModel, PointModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<TriangleModel, SphereModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<TriangleModel, LineModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<TriangleModel, TriangleModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<CapsuleModel, TriangleModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<CapsuleModel, LineModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<TriangleModel, OBBModel, MeshNewProximityIntersection>(this);

        intersection->intersectors.add<RigidSphereModel, PointModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<LineModel, RigidSphereModel, MeshNewProximityIntersection>(this);
        intersection->intersectors.add<TriangleModel, RigidSphereModel, MeshNewProximityIntersection>(this);
    }
}

int MeshNewProximityIntersection::computeIntersection(Point& e1, Point& e2, OutputContainer<Point, Point>* contacts)
{
    const SReal alarmDist = intersection->getAlarmDistance() + e1.getProximity() + e2.getProximity();
    int n = intersection->doIntersectionPointPoint(alarmDist*alarmDist, e1.p(), e2.p(), contacts, (e1.getCollisionModel()->getSize() > e2.getCollisionModel()->getSize()) ? e1.getIndex() : e2.getIndex());
    if (n>0)
    {
        const SReal contactDist = intersection->getContactDistance() + e1.getProximity() + e2.getProximity();
        for (typename OutputContainer<Point, Point>::iterator detection = contacts->end()-n; detection != contacts->end(); ++detection)
        {
            detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(e1, e2);
            detection->value -= contactDist;
        }
    }

    return n;
}


int MeshNewProximityIntersection::computeIntersection(Line& e1, Point& e2, OutputContainer<Line, Point>* contacts)
{
    const SReal alarmDist = intersection->getAlarmDistance() + e1.getProximity() + e2.getProximity();
    int n = doBarycentricIntersectionLinePoint(alarmDist*alarmDist, e1.p1(),e1.p2(),Vector3(0,0,0), e2.p(), contacts, e2.getIndex());
    if (n>0)
    {
        const SReal contactDist = intersection->getContactDistance() + e1.getProximity() + e2.getProximity();
        for (OutputContainer<Line, Point>::iterator detection = contacts->end()-n; detection != contacts->end(); ++detection)
        {
            detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(e1, e2);
            detection->value -= contactDist;
        }
    }
    return n;
}


int MeshNewProximityIntersection::computeIntersection(Line& e1, Line& e2, OutputContainer<Line, Line>* contacts)
{
    const SReal alarmDist = intersection->getAlarmDistance() + e1.getProximity() + e2.getProximity();
    const SReal dist2 = alarmDist*alarmDist;
    const int id = (e1.getCollisionModel()->getSize() > e2.getCollisionModel()->getSize()) ? e1.getIndex() : e2.getIndex();
    int n = doIntersectionLineLine(dist2, e1.p1(),e1.p2(), e2.p1(),e2.p2(), contacts, id);
    if (n>0)
    {
        const SReal contactDist = intersection->getContactDistance() + e1.getProximity() + e2.getProximity();
        for (OutputContainer<Line, Line>::iterator detection = contacts->end()-n; detection != contacts->end(); ++detection)
        {
            detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(e1, e2);
            detection->value -= contactDist;
        }
    }
    return n;
}

int MeshNewProximityIntersection::computeIntersection(Triangle& e1, Point& e2, OutputContainer<Triangle, Point>* contacts)
{
    const SReal alarmDist = intersection->getAlarmDistance() + e1.getProximity() + e2.getProximity();
    const SReal dist2 = alarmDist*alarmDist;
    int n = doIntersectionTrianglePoint(dist2, e1.flags(),e1.p1(),e1.p2(),e1.p3(),e1.n(), e2.p(), contacts, e2.getIndex());
    if (n>0)
    {
        const SReal contactDist = intersection->getContactDistance() + e1.getProximity() + e2.getProximity();
        for (OutputContainer<Triangle, Point>::iterator detection = contacts->end()-n; detection != contacts->end(); ++detection)
        {
            detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(e1, e2);
            detection->value -= contactDist;
        }
    }
    return n;
}


int MeshNewProximityIntersection::computeIntersection(Triangle& e1, Line& e2, OutputContainer<Triangle, Line>* contacts)
{
    const SReal alarmDist = intersection->getAlarmDistance() + e1.getProximity() + e2.getProximity();
    const SReal    dist2 = alarmDist*alarmDist;
    const Vector3& p1 = e1.p1();
    const Vector3& p2 = e1.p2();
    const Vector3& p3 = e1.p3();
    //const Vector3& pn = e1.n();
    const Vector3& q1 = e2.p1();
    const Vector3& q2 = e2.p2();

    const int f1 = e1.flags();

    int n = 0;

    if (f1&TriangleModel::FLAG_P1)
    {
        n += doBarycentricIntersectionLinePoint(dist2, q1, q2, Vector3(0,0,0),p1, contacts, e2.getIndex(), true);
    }
    if (f1&TriangleModel::FLAG_P2)
    {
        n += doBarycentricIntersectionLinePoint(dist2, q1, q2, Vector3(1,0,0),p2, contacts, e2.getIndex(), true);
    }
    if (f1&TriangleModel::FLAG_P3)
    {
        n += doBarycentricIntersectionLinePoint(dist2, q1, q2, Vector3(0,1,0),p3, contacts, e2.getIndex(), true);
    }

    n += doIntersectionTrianglePoint(dist2, f1, p1, p2, p3, Vector3(0,0,0), q1, contacts, e2.getIndex(), false);
    n += doIntersectionTrianglePoint(dist2, f1, p1, p2, p3, Vector3(1,0,0), q2, contacts, e2.getIndex(), false);

    if (intersection->useLineLine.getValue())
    {
        if (f1&TriangleModel::FLAG_E12)
            n += doIntersectionLineLine(dist2, p1, p2, q1, q2, contacts, e2.getIndex());
        if (f1&TriangleModel::FLAG_E23)
            n += doIntersectionLineLine(dist2, p2, p3, q1, q2, contacts, e2.getIndex());
        if (f1&TriangleModel::FLAG_E31)
            n += doIntersectionLineLine(dist2, p3, p1, q1, q2, contacts, e2.getIndex());
    }

    if (n>0)
    {
        const SReal contactDist = intersection->getContactDistance() + e1.getProximity() + e2.getProximity();
        for (OutputContainer<Triangle, Line>::iterator detection = contacts->end()-n; detection != contacts->end(); ++detection)
        {
            detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(e1, e2);
            detection->value -= contactDist;
        }
    }

    return n;
}


int MeshNewProximityIntersection::computeIntersection(Triangle& e1, Triangle& e2, OutputContainer<Triangle, Triangle>* contacts)
{
    if (e1.getIndex() >= e1.getCollisionModel()->getSize())
    {
        intersection->serr << "NewProximityIntersection::computeIntersection(Triangle, Triangle): ERROR invalid e1 index "
                << e1.getIndex() << " on CM " << e1.getCollisionModel()->getName() << " of size " << e1.getCollisionModel()->getSize()<<intersection->sendl;
        return 0;
    }

    if (e2.getIndex() >= e2.getCollisionModel()->getSize())
    {
        intersection->serr << "NewProximityIntersection::computeIntersection(Triangle, Triangle): ERROR invalid e2 index "
                << e2.getIndex() << " on CM " << e2.getCollisionModel()->getName() << " of size " << e2.getCollisionModel()->getSize()<<intersection->sendl;
        return 0;
    }

    const SReal alarmDist = intersection->getAlarmDistance() + e1.getProximity() + e2.getProximity();
    const SReal dist2 = alarmDist*alarmDist;
    const Vector3& p1 = e1.p1();
    const Vector3& p2 = e1.p2();
    const Vector3& p3 = e1.p3();
//    const Vector3& pn = e1.n();
    const Vector3& q1 = e2.p1();
    const Vector3& q2 = e2.p2();
    const Vector3& q3 = e2.p3();
//    const Vector3& qn = e2.n();

    const int f1 = e1.flags();
    const int f2 = e2.flags();

    const int id1 = e1.getIndex()*3; // index of contacts involving points in e1
    const int id2 = e1.getCollisionModel()->getSize()*3 + e2.getIndex()*12; // index of contacts involving points in e2

    int n = 0;

    if (f1&TriangleModel::FLAG_P1)
        n += doIntersectionTrianglePoint(dist2, f2, q1, q2, q3, Vector3(0,0,0), p1, contacts, id1+0, true);
    if (f1&TriangleModel::FLAG_P2)
        n += doIntersectionTrianglePoint(dist2, f2, q1, q2, q3, Vector3(1,0,0), p2, contacts, id1+1, true);
    if (f1&TriangleModel::FLAG_P3)
        n += doIntersectionTrianglePoint(dist2, f2, q1, q2, q3, Vector3(0,1,0), p3, contacts, id1+2, true);

    if (f2&TriangleModel::FLAG_P1)
        n += doIntersectionTrianglePoint(dist2, f1, p1, p2, p3, Vector3(0,0,0), q1, contacts, id2+0, false);
    if (f2&TriangleModel::FLAG_P2)
        n += doIntersectionTrianglePoint(dist2, f1, p1, p2, p3, Vector3(1,0,0), q2, contacts, id2+1, false);
    if (f2&TriangleModel::FLAG_P3)
        n += doIntersectionTrianglePoint(dist2, f1, p1, p2, p3, Vector3(0,1,0), q3, contacts, id2+2, false);

    if (intersection->useLineLine.getValue())
    {
        if (f1&TriangleModel::FLAG_E12)
        {
            if (f2&TriangleModel::FLAG_E12)
                n += doIntersectionLineLine(dist2, p1, p2, q1, q2, contacts, id2+3);
            if (f2&TriangleModel::FLAG_E23)
                n += doIntersectionLineLine(dist2, p1, p2, q2, q3, contacts, id2+4);
            if (f2&TriangleModel::FLAG_E31)
                n += doIntersectionLineLine(dist2, p1, p2, q3, q1, contacts, id2+5);
        }

        if (f1&TriangleModel::FLAG_E23)
        {
            if (f2&TriangleModel::FLAG_E12)
                n += doIntersectionLineLine(dist2, p2, p3, q1, q2, contacts, id2+6);
            if (f2&TriangleModel::FLAG_E23)
                n += doIntersectionLineLine(dist2, p2, p3, q2, q3, contacts, id2+7);
            if (f2&TriangleModel::FLAG_E31)
                n += doIntersectionLineLine(dist2, p2, p3, q3, q1, contacts, id2+8);
        }

        if (f1&TriangleModel::FLAG_E31)
        {
            if (f2&TriangleModel::FLAG_E12)
                n += doIntersectionLineLine(dist2, p3, p1, q1, q2, contacts, id2+9);
            if (f2&TriangleModel::FLAG_E23)
                n += doIntersectionLineLine(dist2, p3, p1, q2, q3, contacts, id2+10);
            if (f2&TriangleModel::FLAG_E31)
                n += doIntersectionLineLine(dist2, p3, p1, q3, q1, contacts, id2+11);
        }
    }

    if (n>0)
    {
        const SReal contactDist = intersection->getContactDistance() + e1.getProximity() + e2.getProximity();
        for (int i = 0; i < n; ++i)
        {
            (*contacts)[OutputContainer<Triangle, Triangle>::DetectionOutputID(contacts->size()-n+i)].elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(e1, e2);
            (*contacts)[OutputContainer<Triangle, Triangle>::DetectionOutputID(contacts->size()-n+i)].value -= contactDist;
        }
    }

    return n;
}



} // namespace collision

} // namespace component

} // namespace sofa

