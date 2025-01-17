#ifndef COMPLIANT_CGSOLVER_H
#define COMPLIANT_CGSOLVER_H

#include "KrylovSolver.h"

namespace sofa {
namespace component {
namespace linearsolver {

/// Conjugate Gradient (iterative, linear solver for symmetric, definite matrix)
// TODO add numerator threshold ? damping ?
class SOFA_Compliant_API CgSolver : public KrylovSolver {

  public:
	SOFA_CLASS(CgSolver, KrylovSolver);
	
	CgSolver();				

  protected:

	virtual void solve_schur(vec& x,
	                         const system_type& system,
	                         const vec& rhs,
							 real damping) const;
	
	virtual void solve_kkt(vec& x,
	                       const system_type& system,
	                       const vec& rhs,
						   real damping) const;

};


}
}
}

#endif
