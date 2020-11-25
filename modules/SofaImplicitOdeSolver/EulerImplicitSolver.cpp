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
// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include <SofaImplicitOdeSolver/EulerImplicitSolver.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/MechanicalOperations.h>
#include <sofa/simulation/common/VectorOperations.h>
#include <sofa/core/ObjectFactory.h>
#include <math.h>
#include <iostream>
#include <sofa/helper/system/thread/CTime.h>
#include <sofa/helper/AdvancedTimer.h>


namespace sofa
{

namespace component
{

namespace odesolver
{
using core::VecId;
using namespace sofa::defaulttype;
using namespace core::behavior;

EulerImplicitSolver::EulerImplicitSolver()
    : f_rayleighStiffness( initData(&f_rayleighStiffness,0.1,"rayleighStiffness","Rayleigh damping coefficient related to stiffness, > 0") )
    , f_rayleighMass( initData(&f_rayleighMass,0.1,"rayleighMass","Rayleigh damping coefficient related to mass, > 0"))
    , f_velocityDamping( initData(&f_velocityDamping,0.,"vdamping","Velocity decay coefficient (no decay if null)") )
    , f_firstOrder (initData(&f_firstOrder, false, "firstOrder", "Use backward Euler scheme for first order ode system."))
    , f_verbose( initData(&f_verbose,false,"verbose","Dump system state at each iteration") )
    , f_projectForce( initData(&f_projectForce,false,"projectForce","Apply projection constraints to force vector (by default, projections are only applied once aggregated with other components of the right hand term)") )
    , f_solveConstraint( initData(&f_solveConstraint,false,"solveConstraint","Apply ConstraintSolver (requires a ConstraintSolver in the same node as this solver, disabled by by default for now)") )
    , d_reverseAccumulateOrder(initData(&d_reverseAccumulateOrder, false, "reverseAccumulateOrder", "True to accumulate forces from nodes in reversed order (can be necessary when using multi-mappings or interaction constraints not following the node hierarchy)"))
{
}

void EulerImplicitSolver::init()
{
    sofa::core::behavior::OdeSolver::init();
}

void EulerImplicitSolver::cleanup()
{
    // free the locally created vector x (including eventual external mechanical states linked by an InteractionForceField)
    sofa::simulation::common::VectorOperations vop( core::ExecParams::defaultInstance(), this->getContext() );
    vop.v_free( x.id(), true, true );
}



void EulerImplicitSolver::solve(const core::ExecParams* params /* PARAMS FIRST */, double dt, sofa::core::MultiVecCoordId xResult, sofa::core::MultiVecDerivId vResult)
{

    assemble(params, dt);
    integrate(params);
    updateXandV(params, dt, xResult, vResult);
}


void EulerImplicitSolver::assemble(const core::ExecParams* params /* PARAMS FIRST */, double dt)
{

#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printNode("SolverVectorAllocation");
#endif

    sofa::simulation::common::MechanicalOperations mop( params, this->getContext() );
    sofa::simulation::common::VectorOperations vop( params, this->getContext() );
    core::behavior::MultiMatrix<simulation::common::MechanicalOperations> matrix(&mop);
    MultiVecCoord pos(&vop, core::VecCoordId::position() );
    MultiVecDeriv vel(&vop, core::VecDerivId::velocity() );
    MultiVecDeriv f(&vop, core::VecDerivId::force() );

    // dx is no longer allocated by default (but it will be deleted automatically by the mechanical objects)
    MultiVecDeriv dx(&vop, core::VecDerivId::dx() ); dx.realloc( &vop, true, true );

    x.realloc( &vop, true, true );
    b.realloc( &vop, true, true );



#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printCloseNode("SolverVectorAllocation");
#endif


    double h = dt;
    const bool verbose  = f_verbose.getValue();
    const bool firstOrder = f_firstOrder.getValue();
    const bool projectForce = f_projectForce.getValue();

    sofa::helper::AdvancedTimer::stepBegin("ComputeForce");

    // compute the net forces at the beginning of the time step
    mop.computeForce(f,true,true,true, d_reverseAccumulateOrder.getValue());
    if( verbose )
        serr<<this->getContext()->getTime() << ": f = "<< f <<sendl;

    sofa::helper::AdvancedTimer::stepNext ("ComputeForce", "ComputeRHTerm");
    if( firstOrder )
    {
        b.eq(f);
    }
    else
    {
        // new more powerful visitors

        // force in the current configuration
        b.eq(f);                                                                         // b = f0

        // add the change of force due to stiffness + Rayleigh damping
        mop.addMBKv(b, -f_rayleighMass.getValue(), 1, h+f_rayleighStiffness.getValue()); // b =  f0 + ( rm M + B + (h+rs) K ) v

        // integration over a time step
        b.teq(h);                                                                        // b = h(f0 + ( rm M + B + (h+rs) K ) v )
    }
    if ( projectForce )
    {
        mop.projectResponse(f);          // f is projected to the constrained space
        if( verbose )
            serr<<this->getContext()->getTime() << ": projected f = "<< f <<sendl;
    }

    if( verbose )
        serr<<this->getContext()->getTime() << ": b = "<< b <<sendl;

    mop.projectResponse(b);          // b is projected to the constrained space

    if( verbose )
        serr<<this->getContext()->getTime() << ": projected b = "<< b <<sendl;

    sofa::helper::AdvancedTimer::stepNext ("ComputeRHTerm", "MBKBuild");

    if (firstOrder)
        matrix = MechanicalMatrix::K * (-h) + MechanicalMatrix::M;
    else
        matrix = MechanicalMatrix::K * (-h*(h+f_rayleighStiffness.getValue())) + MechanicalMatrix::B * (-h) + MechanicalMatrix::M * (1+h*f_rayleighMass.getValue());

    /*if( verbose )
    {
        serr<<this->getContext()->getTime() << ": matrix = "<< (MechanicalMatrix::K * (-h*(h+f_rayleighStiffness.getValue())) + MechanicalMatrix::M * (1+h*f_rayleighMass.getValue())) << " = " << matrix <<sendl;
        serr<<this->getContext()->getTime() << ": Matrix K = " << MechanicalMatrix::K << sendl;
    }*/

    sofa::helper::AdvancedTimer::stepNext ("MBKBuild", "MBKSetSystem");
    matrix.setSystem(x, b); //Call to ODE resolution.
    sofa::helper::AdvancedTimer::stepEnd  ("MBKSetSystem");

    sofa::helper::AdvancedTimer::stepBegin ("MBKSetSystem", "MBKIterativeSolve");
    matrix.iterativeSolveSystem(); //Call to ODE resolution.
    sofa::helper::AdvancedTimer::stepEnd  ("MBKIterativeSolve");
}


void EulerImplicitSolver::integrate(const core::ExecParams* params /* PARAMS FIRST */)
{
    sofa::simulation::common::MechanicalOperations mop( params, this->getContext() );
    core::behavior::MultiMatrix<simulation::common::MechanicalOperations> matrix(&mop);

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("SystemSolution");
#endif

    sofa::helper::AdvancedTimer::stepBegin ("MBKSetSystem", "MBKSolve");
    matrix.directSolveSystem(); //Call to ODE resolution.
    sofa::helper::AdvancedTimer::stepEnd  ("MBKSolve");


#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("SystemSolution");
#endif
}


void EulerImplicitSolver::updateXandV(const core::ExecParams* params /* PARAMS FIRST */, double dt, sofa::core::MultiVecCoordId xResult, sofa::core::MultiVecDerivId vResult)
{

#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printNode("SolverVectorAllocation");
#endif
    sofa::simulation::common::MechanicalOperations mop( params, this->getContext() );
    sofa::simulation::common::VectorOperations vop( params, this->getContext() );
    MultiVecCoord pos(&vop, core::VecCoordId::position() );
    MultiVecDeriv vel(&vop, core::VecDerivId::velocity() );
    MultiVecDeriv f(&vop, core::VecDerivId::force() );
    MultiVecCoord newPos(&vop, xResult );
    MultiVecDeriv newVel(&vop, vResult );

#ifdef SOFA_DUMP_VISITOR_INFO
    sofa::simulation::Visitor::printCloseNode("SolverVectorAllocation");
#endif

    core::behavior::MultiMatrix<simulation::common::MechanicalOperations> matrix(&mop);
    matrix.writeSolution();

    double h = dt;
    const bool verbose  = f_verbose.getValue();
    const bool firstOrder = f_firstOrder.getValue();
    // mop.projectResponse(x);
    // x is the solution of the system

    if (verbose)
    {
        serr<<this->getContext()->getTime() << ": solution = "<< x <<sendl;
    }

    // apply the solution

    const bool solveConstraint = f_solveConstraint.getValue();

    #ifndef SOFA_NO_VMULTIOP // unoptimized version
    if (solveConstraint)
    #endif
    {
    if (firstOrder)
    {
        const char* prevStep = "UpdateV";
        sofa::helper::AdvancedTimer::stepBegin(prevStep);
    #define SOFATIMER_NEXTSTEP(s) { sofa::helper::AdvancedTimer::stepNext(prevStep,s); prevStep=s; }
        newVel.eq(x);                         // vel = x
        if (solveConstraint)
        {
            SOFATIMER_NEXTSTEP("CorrectV");
            mop.solveConstraint(newVel,core::ConstraintParams::VEL);
        }
        SOFATIMER_NEXTSTEP("UpdateX");
        newPos.eq(pos, newVel, h);            // pos = pos + h vel
        if (solveConstraint)
        {
            SOFATIMER_NEXTSTEP("CorrectX");
            mop.solveConstraint(newPos,core::ConstraintParams::POS);
        }
    #undef SOFATIMER_NEXTSTEP
        sofa::helper::AdvancedTimer::stepEnd  (prevStep);
    }
    else
    {
        const char* prevStep = "UpdateV";
        sofa::helper::AdvancedTimer::stepBegin(prevStep);
    #define SOFATIMER_NEXTSTEP(s) { sofa::helper::AdvancedTimer::stepNext(prevStep,s); prevStep=s; }
        //vel.peq( x );                       // vel = vel + x
        newVel.eq(vel, x);
        if (solveConstraint)
        {
            SOFATIMER_NEXTSTEP("CorrectV");
            mop.solveConstraint(newVel,core::ConstraintParams::VEL);
        }
        SOFATIMER_NEXTSTEP("UpdateX");
        //pos.peq( vel, h );                  // pos = pos + h vel
        newPos.eq(pos, newVel, h);
        if (solveConstraint)
        {
            SOFATIMER_NEXTSTEP("CorrectX");
            mop.solveConstraint(newPos,core::ConstraintParams::POS);
        }
    #undef SOFATIMER_NEXTSTEP
        sofa::helper::AdvancedTimer::stepEnd  (prevStep);
    }
    }
    #ifndef SOFA_NO_VMULTIOP
    else
    {
        typedef core::behavior::BaseMechanicalState::VMultiOp VMultiOp;
        VMultiOp ops;
        if (firstOrder)
        {
            ops.resize(2);
            ops[0].first = newVel;
            ops[0].second.push_back(std::make_pair(x.id(),1.0));
            ops[1].first = newPos;
            ops[1].second.push_back(std::make_pair(pos.id(),1.0));
            ops[1].second.push_back(std::make_pair(newVel.id(),h));
        }
        else
        {
            ops.resize(2);
            ops[0].first = newVel;
            ops[0].second.push_back(std::make_pair(vel.id(),1.0));
            ops[0].second.push_back(std::make_pair(x.id(),1.0));
            ops[1].first = newPos;
            ops[1].second.push_back(std::make_pair(pos.id(),1.0));
            ops[1].second.push_back(std::make_pair(newVel.id(),h));
        }

        sofa::helper::AdvancedTimer::stepBegin("UpdateVAndX");
        vop.v_multiop(ops);
        sofa::helper::AdvancedTimer::stepEnd("UpdateVAndX");
    }
    #endif

    mop.addSeparateGravity(dt, newVel);	// v += dt*g . Used if mass wants to added G separately from the other forces to v.
    if (f_velocityDamping.getValue()!=0.0)
        newVel *= exp(-h*f_velocityDamping.getValue());

    if( verbose )
    {
        mop.projectPosition(newPos);
        mop.projectVelocity(newVel);
        mop.propagateX(newPos);
        mop.propagateV(newVel);
        serr<<this->getContext()->getTime() << ": final x = "<< newPos <<sendl;
        serr<<this->getContext()->getTime() << ": final v = "<< newVel <<sendl;
        mop.computeForce(f);
        serr<<this->getContext()->getTime() << ": final f = "<< f <<sendl;

    }

}


SOFA_DECL_CLASS(EulerImplicitSolver)

int EulerImplicitSolverClass = core::RegisterObject("Implicit time integrator using backward Euler scheme")
        .add< EulerImplicitSolver >()
        .addAlias("EulerImplicit")
        .addAlias("ImplicitEulerSolver")
        .addAlias("ImplicitEuler")
#ifdef SOFA_SMP
        .addAlias("ParallelEulerImplicit")
#endif
        ;

} // namespace odesolver

} // namespace component

} // namespace sofa

