#include "EigenLLTDomainSolver.inl"

#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace component
{

namespace linearsolver
{

int EigenLLTDomainSolverClass = sofa::core::RegisterObject("A linear solver working on a domain whose nodes are defined by a single instance of a MechanicalObject")
.add< EigenLLTDomainSolver< sofa::defaulttype::Vec1Types > >()
.add< EigenLLTDomainSolver< sofa::defaulttype::Vec3Types > >()
.add< EigenLLTDomainSolver< sofa::defaulttype::Rigid3Types > >();

template class SOFA_BASE_LINEAR_SOLVER_API EigenLLTDomainSolver< sofa::defaulttype::Vec1Types >;
template class SOFA_BASE_LINEAR_SOLVER_API EigenLLTDomainSolver< sofa::defaulttype::Vec3Types >;
template class SOFA_BASE_LINEAR_SOLVER_API EigenLLTDomainSolver< sofa::defaulttype::Rigid3Types >;


}

}

}