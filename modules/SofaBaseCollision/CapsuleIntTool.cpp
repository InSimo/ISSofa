#include <SofaBaseCollision/CapsuleIntTool.inl>

namespace sofa
{
namespace component
{
namespace collision
{
using namespace sofa::defaulttype;
using namespace sofa::core::collision;

bool CapsuleIntTool::shareSameVertex(const Capsule & c1,const Capsule & c2){
    return c1.shareSameVertex(c2);
}


template SOFA_BASE_COLLISION_API int CapsuleIntTool::computeIntersection(TCapsule<sofa::defaulttype::Vec3Types>&, TCapsule<sofa::defaulttype::Vec3Types>&,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::Vec3Types>, TCapsule<sofa::defaulttype::Vec3Types>>* contacts);
template SOFA_BASE_COLLISION_API int CapsuleIntTool::computeIntersection(TCapsule<sofa::defaulttype::Vec3Types>&, TCapsule<sofa::defaulttype::RigidTypes>&,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::Vec3Types>, TCapsule<sofa::defaulttype::RigidTypes>>* contacts);
template SOFA_BASE_COLLISION_API int CapsuleIntTool::computeIntersection(TCapsule<sofa::defaulttype::RigidTypes>&, TCapsule<sofa::defaulttype::RigidTypes>&,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::RigidTypes>, TCapsule<sofa::defaulttype::RigidTypes>>* contacts);
template SOFA_BASE_COLLISION_API int CapsuleIntTool::computeIntersection(TCapsule<sofa::defaulttype::RigidTypes> & cap, OBB& obb,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::RigidTypes>, OBB>* contacts);
template SOFA_BASE_COLLISION_API int CapsuleIntTool::computeIntersection(TCapsule<sofa::defaulttype::Vec3Types> & cap, OBB& obb,SReal alarmDist,SReal contactDist,OutputContainer<TCapsule<sofa::defaulttype::Vec3Types>, OBB>* contacts);

class SOFA_BASE_VISUAL_API CapsuleIntTool;
}
}
}
