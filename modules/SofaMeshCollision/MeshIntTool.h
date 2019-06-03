#ifndef MESHINTTOOL_H
#define MESHINTTOOL_H
#include <sofa/core/collision/Intersection.h>
#include <sofa/helper/FnDispatcher.h>
#include <SofaBaseCollision/OBBModel.h>
#include <SofaBaseCollision/CapsuleModel.h>
#include <SofaBaseCollision/RigidCapsuleModel.h>
#include <SofaMeshCollision/TriangleModel.h>
#include <SofaMeshCollision/PointModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/IntrTriangleOBB.h>
#include <SofaBaseCollision/SphereModel.h>

namespace sofa
{
namespace component
{
namespace collision
{


class SOFA_MESH_COLLISION_API MeshIntTool
{
public:
    template <class Elem1, class Elem2>
    using OutputContainer = sofa::core::collision::TDetectionOutputContainer<typename Elem1::Model, typename Elem2::Model>;
    typedef sofa::core::collision::DetectionOutput DetectionOutput;

    template <class DataTypes>
    static int computeIntersection(TCapsule<DataTypes>& cap, Point& pnt,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<DataTypes>, Point>* contacts);
    ////!\ CAUTION : uninitialized fields detection->elem and detection->id
    template <class DataTypes, class TOutputContainer>
    static int doCapPointInt(TCapsule<DataTypes>& cap, const defaulttype::Vector3& q,SReal alarmDist,SReal contactDist,TOutputContainer* contacts);

    template <class DataTypes>
    static int computeIntersection(TCapsule<DataTypes>& cap, Line& lin,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<DataTypes>, Line>* contacts);

    ////!\ CAUTION : uninitialized fields detection->elem and detection->id
    template <class DataTypes, class TOutputContainer>
    static int doCapLineInt(TCapsule<DataTypes>& cap,const defaulttype::Vector3 & q1,const defaulttype::Vector3 & q2,SReal alarmDist,SReal contactDist,TOutputContainer* contacts,bool ignore_p1 = false,bool ignore_p2 = false);

    ////!\ CAUTION : uninitialized fields detection->elem and detection->id and detection->value
    template <class TOutputContainer>
    static int doCapLineInt(const defaulttype::Vector3 & p1,const defaulttype::Vector3 & p2,SReal cap_rad,
                         const defaulttype::Vector3 & q1, const defaulttype::Vector3 & q2,SReal alarmDist,SReal contactDist,TOutputContainer* contacts,bool ignore_p1 = false,bool ignore_p2 = false);

    ////!\ CAUTION : uninitialized fields detection->elem and detection->id and detection->value, you have to substract contactDist, because
    ///this function can be used also as doIntersectionTriangleSphere where the contactDist = getContactDist() + sphere_radius
    template <class TOutputContainer>
    static int doIntersectionTrianglePoint(SReal dist2, int flags, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& p3,const defaulttype::Vector3& q, TOutputContainer* contacts,bool swapElems = false);

    template <class DataTypes>
    static int computeIntersection(TCapsule<DataTypes>& cap, Triangle& tri,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<DataTypes>, Triangle>* contacts);

    static int computeIntersection(Triangle& tri,OBB & obb,SReal alarmDist,SReal contactDist,OutputContainer<Triangle, OBB>* contacts);

    static int computeIntersection(Triangle& tri,int flags,OBB & obb,SReal alarmDist,SReal contactDist,OutputContainer<Triangle, OBB>* contacts);

    //SPHERE - POINT
    template <class DataTypes>
    static int computeIntersection(TSphere<DataTypes> & sph, Point& pt,typename DataTypes::Real alarmDist,typename DataTypes::Real contactDist, OutputContainer<TSphere<DataTypes>, Point>* contacts);

    template <class TReal>
    static int computeIntersection(TSphere<defaulttype::StdVectorTypes<defaulttype::Vec<3,TReal>,defaulttype::Vec<3,TReal>,TReal> > & sph, Point& pt,TReal alarmDist,TReal contactDist, OutputContainer<TSphere<defaulttype::StdVectorTypes<defaulttype::Vec<3,TReal>,defaulttype::Vec<3,TReal>,TReal> >,  Point>* contacts);
    ///

    //LINE - SPHERE
    template <class DataTypes>
    static int computeIntersection(Line& e2, TSphere<DataTypes>& e1,typename DataTypes::Real alarmDist,typename DataTypes::Real contactDist, OutputContainer<Line, TSphere<DataTypes>>* contacts);

    template <class TReal>
    static int computeIntersection(Line& e2, TSphere<defaulttype::StdVectorTypes<defaulttype::Vec<3,TReal>,defaulttype::Vec<3,TReal>,TReal> >& e1,TReal alarmDist,TReal contactDist, OutputContainer<Line, TSphere<defaulttype::StdVectorTypes<defaulttype::Vec<3,TReal>,defaulttype::Vec<3,TReal>,TReal> >>* contacts);
    ///

    //TRIANGLE - SPHERE
    template <class DataTypes>
    static int computeIntersection(Triangle& tri, TSphere<DataTypes>& sph,typename DataTypes::Real alarmDist,typename DataTypes::Real contactDist, OutputContainer<Triangle, TSphere<DataTypes>>* contacts);

    template <class TReal>
    static int computeIntersection(Triangle& tri, TSphere<defaulttype::StdVectorTypes<defaulttype::Vec<3,TReal>,defaulttype::Vec<3,TReal>,TReal> >& sph,TReal alarmDist,TReal contactDist, OutputContainer<Triangle, TSphere<defaulttype::StdVectorTypes<defaulttype::Vec<3,TReal>,defaulttype::Vec<3,TReal>,TReal> >>* contacts);
    ///

    //flags are the flags of the Triangle and p1 p2 p3 its vertices, to_be_projected is the point to be projected on the triangle, i.e.
    //after this method, it will probably be different
    static int projectPointOnTriangle(int flags, const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& p3,defaulttype::Vector3& to_be_projected);

    //returns barycentric coords in alpha and beta so that to_be_projected = (1 - alpha - beta) * p1 + alpha * p2 + beta * p3
    static void triangleBaryCoords(const defaulttype::Vector3& to_be_projected,const defaulttype::Vector3& p1, const defaulttype::Vector3& p2, const defaulttype::Vector3& p3,SReal & alpha,SReal & beta);
};

inline int MeshIntTool::computeIntersection(Triangle& tri,OBB & obb,SReal alarmDist,SReal contactDist,OutputContainer<Triangle, OBB>* contacts){
    return computeIntersection(tri,tri.flags(),obb,alarmDist,contactDist,contacts);
}

template <class DataTypes>
int MeshIntTool::computeIntersection(TSphere<DataTypes> & e1, Point& e2,typename DataTypes::Real alarmDist,typename DataTypes::Real contactDist, OutputContainer<TSphere<DataTypes>, Point>* contacts){
    const typename DataTypes::Real myAlarmDist = alarmDist + e1.r();

    typename DataTypes::Coord P,Q,PQ;
    P = e1.center();
    Q = e2.p();
    PQ = Q-P;
    if (PQ.norm2() >= myAlarmDist*myAlarmDist)
        return 0;

    const typename DataTypes::Real  myContactDist = contactDist + e1.r();

    DetectionOutput& detection = contacts->addDetectionOutput();
    detection.elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(e1, e2);
    detection.id = (e1.getCollisionModel()->getSize() > e2.getCollisionModel()->getSize()) ? e1.getIndex() : e2.getIndex();
    detection.point[1]=Q;
    detection.normal=PQ;
    detection.value = detection.normal.norm();
    if(detection.value>1e-15)
    {
        detection.normal /= detection.value;
    }
    else
    {
        //intersection->serr<<"WARNING: null distance between contact detected"<<intersection->sendl;
        detection.normal= typename DataTypes::Coord(1,0,0);
    }
    detection.point[0] = e1.getContactPointByNormal( -detection.normal );

    detection.value -= myContactDist;
    return 1;
}


template <class DataTypes>
int MeshIntTool::computeIntersection(Line& e2, TSphere<DataTypes>& e1,typename DataTypes::Real alarmDist,typename DataTypes::Real contactDist, OutputContainer<Line, TSphere<DataTypes>>* contacts){
    const typename DataTypes::Real myAlarmDist = alarmDist + e1.r();

    const typename DataTypes::Coord x32 = e2.p1()-e2.p2();
    const typename DataTypes::Coord x31 = e1.center()-e2.p2();
    typename DataTypes::Real A;
    typename DataTypes::Real b;
    A = x32*x32;
    b = x32*x31;

    typename DataTypes::Real alpha = 0.5;
    typename DataTypes::Coord Q;

    if(alpha <= 0){
        Q = e2.p1();
    }
    else if(alpha >= 1){
        Q = e2.p2();
    }
    else{
        Q = e2.p1() - x32 * alpha;
    }

    typename DataTypes::Coord P = e1.center();
    typename DataTypes::Coord QP = P-Q;
    //typename DataTypes::Coord PQ = Q-P;

    if (QP.norm2() >= myAlarmDist*myAlarmDist)
        return 0;

    const typename DataTypes::Real myContactDist = contactDist + e1.r();

    DetectionOutput& detection = contacts->addDetectionOutput();
    detection.elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(e2, e1);
    detection.id = e1.getIndex();
    detection.point[0]=Q;
    detection.normal=QP;
    detection.value = detection.normal.norm();
    if(detection.value>1e-15)
    {
        detection.normal /= detection.value;
    }
    else
    {
        //intersection->serr<<"WARNING: null distance between contact detected"<<intersection->sendl;
        detection.normal= typename DataTypes::Coord(1,0,0);
    }
    detection.point[1]=e1.getContactPointByNormal( detection.normal );
    detection.value -= myContactDist;
    return 1;
}



template <class DataTypes>
int MeshIntTool::computeIntersection(Triangle& tri, TSphere<DataTypes>& sph,typename DataTypes::Real alarmDist,typename DataTypes::Real contactDist, OutputContainer<Triangle, TSphere<DataTypes>>* contacts){
    const typename DataTypes::Coord sph_center = sph.p();
    typename DataTypes::Coord proj_p = sph_center;
    if(projectPointOnTriangle(tri.flags(),tri.p1(),tri.p2(),tri.p3(),proj_p)){

        typename DataTypes::Coord proj_p_sph_center = sph_center - proj_p;
        typename DataTypes::Real myAlarmDist = alarmDist + sph.r();
        if(proj_p_sph_center.norm2() >= myAlarmDist*myAlarmDist)
            return 0;

        DetectionOutput& detection = contacts->addDetectionOutput();
        detection.elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(tri, sph);
        detection.id = sph.getIndex();
        detection.point[0]=proj_p;
        detection.normal = proj_p_sph_center;
        detection.value = detection.normal.norm();
        detection.normal /= detection.value;
        detection.point[1] = sph.getContactPointByNormal( detection.normal );
        detection.value -= (contactDist + sph.r());
    }
    else{
        return 0;
    }
}


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_MESH_COLLISION)
extern template SOFA_MESH_COLLISION_API int MeshIntTool::computeIntersection(TCapsule<sofa::defaulttype::Vec3Types>& cap, Point& pnt,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::Vec3Types>, Point>* contacts);
extern template SOFA_MESH_COLLISION_API int MeshIntTool::computeIntersection(TCapsule<sofa::defaulttype::Vec3Types>& cap, Line& lin,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::Vec3Types>, Line>* contacts);
extern template SOFA_MESH_COLLISION_API int MeshIntTool::computeIntersection(TCapsule<sofa::defaulttype::Vec3Types>& cap, Triangle& tri,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::Vec3Types>, Triangle>* contacts);

extern template SOFA_MESH_COLLISION_API int MeshIntTool::computeIntersection(TCapsule<sofa::defaulttype::Rigid3Types>& cap, Point& pnt,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::Rigid3Types>, Point>* contacts);
extern template SOFA_MESH_COLLISION_API int MeshIntTool::computeIntersection(TCapsule<sofa::defaulttype::Rigid3Types>& cap, Line& lin,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::Rigid3Types>, Line>* contacts);
extern template SOFA_MESH_COLLISION_API int MeshIntTool::computeIntersection(TCapsule<sofa::defaulttype::Rigid3Types>& cap, Triangle& tri,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::Rigid3Types>, Triangle>* contacts);
#endif


}
}
}
#endif
