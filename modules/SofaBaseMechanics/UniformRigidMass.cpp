#define SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_CPP

#include "UniformRigidMass.inl"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace component
{
namespace mass
{

int UniformRigidMassClass = sofa::core::RegisterObject("An implementation of rigid mass. Uniform for all dofs")
#ifndef SOFA_FLOAT
                            .add< UniformRigidMass< sofa::defaulttype::Rigid3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
                            .add< UniformRigidMass< sofa::defaulttype::Rigid3fTypes > >()
#endif 
                            ;

#ifndef SOFA_FLOAT
template class SOFA_BASE_MECHANICS_API UniformRigidMass< sofa::defaulttype::Rigid3dTypes >;
#endif 
#ifndef SOFA_DOUBLE
template class SOFA_BASE_MECHANICS_API UniformRigidMass< sofa::defaulttype::Rigid3fTypes >;
#endif

}

}

}
