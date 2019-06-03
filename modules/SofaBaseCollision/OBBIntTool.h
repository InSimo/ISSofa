#ifndef OBBINTTOOL_H
#define OBBINTTOOL_H
#include <SofaBaseCollision/OBBModel.h>
#include <SofaBaseCollision/IntrOBBOBB.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <SofaBaseCollision/CapsuleModel.h>
#include <SofaBaseCollision/IntrSphereOBB.h>
namespace sofa{
namespace component{
namespace collision{

class SOFA_BASE_COLLISION_API OBBIntTool{
public:
    template <class Elem1, class Elem2>
    using OutputContainer = sofa::core::collision::TDetectionOutputContainer<typename Elem1::Model, typename Elem2::Model>;
    typedef sofa::core::collision::DetectionOutput DetectionOutput;

    static int computeIntersection(OBB&, OBB&,SReal alarmDist,SReal contactDist,OutputContainer<OBB, OBB>* contacts);

    template <class DataTypes>
    static int computeIntersection(TSphere<DataTypes> &sph1, OBB &box,SReal alarmDist,SReal contactDist,OutputContainer<TSphere<DataTypes>, OBB>* contacts);
};

template <class DataTypes>
int OBBIntTool::computeIntersection(TSphere<DataTypes> & sphere,OBB & box,SReal alarmDist,SReal contactDist,OutputContainer<TSphere<DataTypes>, OBB>* contacts){
    TIntrSphereOBB<DataTypes,OBB::DataTypes> intr(sphere,box);
    //double max_time = helper::rsqrt((alarmDist * alarmDist)/((box1.lvelocity() - box0.lvelocity()).norm2()));
    if(/*intr.Find(max_time,box0.lvelocity(),box1.lvelocity())*/intr.Find()){
        OBB::Real dist = intr.distance();
        if((!intr.colliding()) && dist > alarmDist)
            return 0;

        DetectionOutput& detection = contacts->addDetectionOutput();

        detection.normal = intr.separatingAxis();
        detection.point[0] = sphere.getContactPointWithSurfacePoint( intr.pointOnFirst() );
        detection.point[1] = intr.pointOnSecond();

        if(intr.colliding())
            detection.value = -dist - contactDist;
        else
            detection.value = dist - contactDist;

        detection.elem.first = sphere;
        detection.elem.second = box;
        //detection.id = (box.getCollisionModel()->getSize() > sphere.getCollisionModel()->getSize()) ? box.getIndex() : sphere.getIndex();
        detection.id = sphere.getIndex();


        return 1;
    }

    return 0;
}


}
}
}
#endif // OBBINTTOOL_H
