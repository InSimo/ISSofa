#define SOFA_COMPONENT_CONSTRAINT_GYROSCOPICTVELOCITYIMPULSECONSTRAINT_CPP

#include "GyroscopicVelocityImpulseConstraint.inl"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace component
{
namespace constraint
{

int GyroscopicVelocityImpulseConstraintClass = sofa::core::RegisterObject("Gyroscopic impulse on angular velocities, based on an implicit formulation.")
#ifndef SOFA_FLOAT
    .add< GyroscopicVelocityImpulseConstraint< sofa::defaulttype::Rigid3dTypes > >()
#endif 
#ifndef SOFA_DOUBLE
    .add< GyroscopicVelocityImpulseConstraint< sofa::defaulttype::Rigid3fTypes > >()
#endif 
    ;

#ifndef SOFA_FLOAT
template class SOFA_BASE_MECHANICS_API GyroscopicVelocityImpulseConstraint< sofa::defaulttype::Rigid3dTypes >;
#endif 
#ifndef SOFA_DOUBLE
template class SOFA_BASE_MECHANICS_API GyroscopicVelocityImpulseConstraint< sofa::defaulttype::Rigid3fTypes >;
#endif



}

}

}
