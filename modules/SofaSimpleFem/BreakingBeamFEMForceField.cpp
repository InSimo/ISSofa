#define SOFA_COMPONENT_FORCEFIELD_BREAKINGBEAMFEMFORCEFIELD_CPP
#include "BreakingBeamFEMForceField.inl"
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>


namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;



SOFA_DECL_CLASS(BreakingBeamFEMForceField)

// Register in the Factory
int BreakingBeamFEMForceFieldClass = core::RegisterObject("Breaking beam finite elements")
#ifndef SOFA_FLOAT
        .add< BreakingBeamFEMForceField<Rigid3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
        .add< BreakingBeamFEMForceField<Rigid3fTypes> >()
#endif
        ;

#ifndef SOFA_FLOAT
template class SOFA_SIMPLE_FEM_API BreakingBeamFEMForceField<Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_SIMPLE_FEM_API BreakingBeamFEMForceField<Rigid3fTypes>;
#endif

} // namespace forcefield

} // namespace component

} // namespace sofa

