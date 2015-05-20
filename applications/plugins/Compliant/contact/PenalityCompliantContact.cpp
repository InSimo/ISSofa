

#include "PenalityCompliantContact.h"

#include <SofaConstraint/FrictionContact.inl>
#include <SofaMeshCollision/RigidContactMapper.inl>
#include <SofaMeshCollision/BarycentricContactMapper.inl>


namespace sofa
{

namespace component
{

namespace collision
{

using namespace defaulttype;
using namespace sofa::helper;
using simulation::Node;
using core::collision::Contact;

SOFA_DECL_CLASS(PenalityCompliantContact)


sofa::core::collision::ContactCreator< PenalityCompliantContact<PointModel, PointModel> > PointPointPenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<LineModel, SphereModel> > LineSpherePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<LineModel, PointModel> > LinePointPenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<LineModel, LineModel> > LineLinePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<TriangleModel, SphereModel> > TriangleSpherePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<TriangleModel, PointModel> > TrianglePointPenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<TriangleModel, LineModel> > TriangleLinePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<TriangleModel, TriangleModel> > TriangleTrianglePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<SphereModel, SphereModel> > SphereSpherePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<SphereModel, PointModel> > SpherePointPenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<CapsuleModel, CapsuleModel> > CapsuleCapsulePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<CapsuleModel, TriangleModel> > CapsuleTrianglePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<CapsuleModel, SphereModel> > CapsuleSpherePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<OBBModel, OBBModel> > OBBOBBPenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<SphereModel, OBBModel> > SphereOBBPenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<CapsuleModel, OBBModel> > CapsuleOBBPenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<TriangleModel, OBBModel> > TriangleOBBPenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<RigidSphereModel, RigidSphereModel> > RigidSphereRigidSpherePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<SphereModel, RigidSphereModel> > SphereRigidSpherePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<LineModel, RigidSphereModel> > LineRigidSpherePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<TriangleModel, RigidSphereModel> > TriangleRigidSpherePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<RigidSphereModel, PointModel> > RigidSpherePointPenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<CapsuleModel, RigidSphereModel> > CapsuleRigidSpherePenalityCompliantContactClass("PenalityCompliantContact",true);
sofa::core::collision::ContactCreator< PenalityCompliantContact<RigidSphereModel, OBBModel> > RigidSphereOBBPenalityCompliantContactClass("PenalityCompliantContact",true);


} // namespace collision

} // namespace component

} // namespace sofa
