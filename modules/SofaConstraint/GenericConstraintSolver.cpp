/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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

#include <SofaConstraint/GenericConstraintSolver.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/SolveVisitor.h>
#include <sofa/simulation/common/VectorOperations.h>

#include <sofa/simulation/common/Simulation.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/gl/Axis.h>
#include <sofa/helper/gl/Cylinder.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/system/thread/CTime.h>
#include <math.h>
#include <numeric>

#include <sofa/core/ObjectFactory.h>

#include "ConstraintStoreLambdaVisitor.h"


//#define GENERIC_CONSTRAINT_SOLVER_USE_BLOCKDIAGONAL_RESIDUAL

namespace sofa
{

namespace component
{

namespace constraintset
{

namespace
{

template< typename TMultiVecId >
void clearMultiVecId(sofa::core::objectmodel::BaseContext* ctx, const sofa::core::ConstraintParams* cParams, const TMultiVecId& vid)
{
    simulation::MechanicalVOpVisitor clearVisitor(cParams, vid, core::ConstMultiVecDerivId::null(), core::ConstMultiVecDerivId::null(), 1.0);
    clearVisitor.setMapped(true);
    ctx->executeVisitor(&clearVisitor);
}

}

GenericConstraintSolver::GenericConstraintSolver()
: displayTime(initData(&displayTime, false, "displayTime","Display time for each important step of GenericConstraintSolver."))
, maxIt( initData(&maxIt, 1000, "maxIterations", "maximal number of iterations of the Gauss-Seidel algorithm"))
, tolerance( initData(&tolerance, 0.001, "tolerance", "residual error threshold for termination of the Gauss-Seidel algorithm.\
                                                       Same unit as the constraint system right hand side. \
                                                       If the constraint rhs is not homogeneous, for example if it mixes distance constraints and volumetric constraints, \
                                                       a specific tolerance value can be set by each constraint resolution to give a consistent \
                                                       tolerancing threshold when the residual vector is evaluated at each iteration."))
, useInfiniteNorm( initData(&useInfiniteNorm, false, "useInfiniteNorm", "If true, the infinite norm of the residual vector is used to evaluate convergence. \
                                                                         The infinite norm is computed per block, to take into account the possible differences \
                                                                         taken by the tolerance value between blocks.\
                                                                         Otherwise, it is the per block average of the residual vector that must be below the specified tolerance \
                                                                         to grant convergence. Note that using a per block average prevents the error of a given block be compensated \
                                                                         by other blocks.\
                                                                         Also note that if the system is only composed of blocks of size one, the infinite norm over \
                                                                         the whole residual vector will give the same result as the block averaged residual norm."))
, sor( initData(&sor, 1.0, "sor", "Successive Over Relaxation parameter (0-2)"))
, schemeCorrection( initData(&schemeCorrection, false, "schemeCorrection", "Apply new scheme where compliance is progressively corrected"))
, unbuilt(initData(&unbuilt, false, "unbuilt", "Compliance is not fully built"))
, computeGraphs(initData(&computeGraphs, false, "computeGraphs", "Compute graphs of errors and forces during resolution"))
, graphErrors( initData(&graphErrors,"graphErrors","Sum of the constraints' errors at each iteration"))
, graphConstraints( initData(&graphConstraints,"graphConstraints","Graph of each constraint's error at the end of the resolution"))
, graphForces( initData(&graphForces,"graphForces","Graph of each constraint's force at each step of the resolution"))
, graphViolations( initData(&graphViolations, "graphViolations", "Graph of each constraint's violation at each step of the resolution"))
, currentNumConstraints(initData(&currentNumConstraints, 0, "currentNumConstraints", "OUTPUT: current number of constraints"))
, currentNumConstraintGroups(initData(&currentNumConstraintGroups, 0, "currentNumConstraintGroups", "OUTPUT: current number of constraints"))
, currentIterations(initData(&currentIterations, 0, "currentIterations", "OUTPUT: current number of constraint groups"))
, currentError(initData(&currentError, 0.0, "currentError", "OUTPUT: current error"))
, reverseAccumulateOrder(initData(&reverseAccumulateOrder, false, "reverseAccumulateOrder", "True to accumulate constraints from nodes in reversed order (can be necessary when using multi-mappings or interaction constraints not following the node hierarchy)"))
, d_dumpSystem(initData(&d_dumpSystem, false, "dumpSystem", "Dump linear systems to terminal (debug purpose, to use with small systems)"))
, current_cp(&m_cpBuffer[0])
, last_cp(NULL)
{
	addAlias(&maxIt, "maxIt");

	graphErrors.setWidget("graph");
	graphErrors.setGroup("Graph");

	graphConstraints.setWidget("graph");
	graphConstraints.setGroup("Graph");

	graphForces.setWidget("graph_linear");
	graphForces.setGroup("Graph2");

	graphViolations.setWidget("graph_linear");
	graphViolations.setGroup("Graph2");

	currentNumConstraints.setReadOnly(true);
	currentNumConstraints.setGroup("Stats");
	currentNumConstraintGroups.setReadOnly(true);
	currentNumConstraintGroups.setGroup("Stats");
	currentIterations.setReadOnly(true);
	currentIterations.setGroup("Stats");
	currentError.setReadOnly(true);
	currentError.setGroup("Stats");

	maxIt.setRequired(true);
	tolerance.setRequired(true);
}

GenericConstraintSolver::~GenericConstraintSolver()
{
}

void GenericConstraintSolver::init()
{
	core::behavior::ConstraintSolver::init();

	// Prevents ConstraintCorrection accumulation due to multiple AnimationLoop initialization on dynamic components Add/Remove operations.
	if (!constraintCorrections.empty())
	{
		constraintCorrections.clear();
	}

	getContext()->get<core::behavior::BaseConstraintCorrection>(&constraintCorrections, core::objectmodel::BaseContext::SearchDown);

	context = (simulation::Node*) getContext();

    simulation::common::VectorOperations vop(sofa::core::ExecParams::defaultInstance(), this->getContext());
    {
        sofa::core::behavior::MultiVecDeriv lambda(&vop, m_lambdaId);
        lambda.realloc(&vop,false,true);
        m_lambdaId = lambda.id();
    }
    {
        sofa::core::behavior::MultiVecDeriv dx(&vop, m_dxId);
        dx.realloc(&vop,false,true);
        m_dxId = dx.id();
    }
}

void GenericConstraintSolver::cleanup()
{
    simulation::common::VectorOperations vop(sofa::core::ExecParams::defaultInstance(), this->getContext());
    vop.v_free(m_lambdaId, false, true);
    vop.v_free(m_dxId, false, true);
    core::behavior::ConstraintSolver::cleanup();
}

void GenericConstraintSolver::removeConstraintCorrection(core::behavior::BaseConstraintCorrection *s)
{
    constraintCorrections.erase(std::remove(constraintCorrections.begin(), constraintCorrections.end(), s), constraintCorrections.end());
}

bool GenericConstraintSolver::prepareStates(const core::ConstraintParams *cParams, MultiVecId /*res1*/, MultiVecId /*res2*/)
{
	sofa::helper::AdvancedTimer::StepVar vtimer("PrepareStates");

	last_cp = current_cp;

    clearConstraintProblemLocks(); // NOTE: this assumes we solve only one constraint problem per step

	time = 0.0;
	timeTotal = 0.0;
    timeScale = 1000.0 / (double)sofa::helper::system::thread::CTime::getTicksPerSec();

	simulation::common::VectorOperations vop(cParams, this->getContext());
    
        
    {
        sofa::core::behavior::MultiVecDeriv lambda(&vop, m_lambdaId);
        lambda.realloc(&vop,false,true);
        m_lambdaId = lambda.id();

        clearMultiVecId(getContext(), cParams, m_lambdaId);
    }

    {
        sofa::core::behavior::MultiVecDeriv dx(&vop, m_dxId);
        dx.realloc(&vop,false,true);
        m_dxId = dx.id();

        clearMultiVecId(getContext(), cParams, m_dxId);
        
    }

	if ( displayTime.getValue() )
	{
		time = (double) timer.getTime();
		timeTotal = (double) timerTotal.getTime();
	}

	return true;
}

bool GenericConstraintSolver::buildSystem(const core::ConstraintParams *cParams, MultiVecId /*res1*/, MultiVecId /*res2*/)
{
	unsigned int numConstraints = 0;

    sofa::helper::AdvancedTimer::stepBegin("Reset Constraint");
	// mechanical action executed from root node to propagate the constraints
	simulation::MechanicalResetConstraintVisitor(cParams).execute(context);
	// calling buildConstraintMatrix
	//simulation::MechanicalAccumulateConstraint(&cparams, cParams->j(), numConstraints).execute(context);

    sofa::helper::AdvancedTimer::stepNext("Reset Constraint", "Build Constraint");

    simulation::MechanicalBuildConstraintMatrix(cParams, cParams->j(), numConstraints).execute(context);
    
    sofa::helper::AdvancedTimer::stepNext("Build Constraint", "Accumulate Matrix");

    simulation::MechanicalAccumulateMatrixDeriv(cParams, cParams->j(), reverseAccumulateOrder.getValue()).execute(context);
    sofa::helper::AdvancedTimer::stepNext("Accumulate Matrix", "Project Jacobian");

    // suppress the constraints that are on DOFS currently concerned by projective constraint
    core::MechanicalParams mparams = core::MechanicalParams(*cParams);
    simulation::MechanicalProjectJacobianMatrixVisitor(&mparams).execute(context);

    sofa::helper::AdvancedTimer::stepEnd  ("Project Jacobian");
    sofa::helper::AdvancedTimer::valSet("numConstraints", numConstraints);

	current_cp->clear(numConstraints);

    sofa::helper::AdvancedTimer::stepBegin("Get Constraint Value");
	MechanicalGetConstraintViolationVisitor(cParams, &current_cp->dFree).execute(context);
    sofa::helper::AdvancedTimer::stepEnd ("Get Constraint Value");

    sofa::helper::AdvancedTimer::stepBegin("Get Constraint Resolutions");
	MechanicalGetConstraintResolutionVisitor(cParams, current_cp->constraintsResolutions).execute(context);
    sofa::helper::AdvancedTimer::stepEnd("Get Constraint Resolutions");

    if (this->f_printLog.getValue()) sout<<"GenericConstraintSolver: "<<numConstraints<<" constraints"<<sendl;

    if (unbuilt.getValue())
	{
		for (unsigned int i=0;i<constraintCorrections.size();i++)
		{
			core::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
			if (!cc->isActive()) continue;
			cc->resetForUnbuiltResolution(current_cp->getF(), current_cp->constraints_sequence); 
		}
 
        sofa::component::linearsolver::SparseMatrix<double>* Wdiag = &current_cp->Wdiag;
		Wdiag->resize(numConstraints, numConstraints);

		// for each contact, the constraint corrections that are involved with the contact are memorized
		current_cp->cclist_elems.clear();
		current_cp->cclist_elems.resize(numConstraints);
		int nbCC = constraintCorrections.size();
		for (unsigned int i = 0; i < numConstraints; i++)
			current_cp->cclist_elems[i].resize(nbCC, NULL);

		unsigned int nbObjects = 0;
		for (unsigned int c_id = 0; c_id < numConstraints;)
		{
			bool foundCC = false;
			nbObjects++;
			unsigned int l = current_cp->constraintsResolutions[c_id]->getNbLines();

			for (unsigned int j = 0; j < constraintCorrections.size(); j++)
			{
				core::behavior::BaseConstraintCorrection* cc = constraintCorrections[j];
				if (!cc->isActive()) continue;
				if (cc->hasConstraintNumber(c_id))
				{
					current_cp->cclist_elems[c_id][j] = cc;
					cc->getBlockDiagonalCompliance(Wdiag, c_id, c_id + l - 1);
					foundCC = true;
				}
			}

			if (!foundCC)
				serr << "WARNING: no constraintCorrection found for constraint" << c_id << sendl;

			double** w =  current_cp->getW();
			for(unsigned int m = c_id; m < c_id + l; m++)
				for(unsigned int n = c_id; n < c_id + l; n++)
					w[m][n] = Wdiag->element(m, n);

			c_id += l;
		}

		current_cp->change_sequence = false;
		if(current_cp->constraints_sequence.size() == nbObjects)
			current_cp->change_sequence=true;
	}
	else
	{
		sofa::helper::AdvancedTimer::stepBegin("Get Compliance");
		if (this->f_printLog.getValue()) sout<<" computeCompliance in "  << constraintCorrections.size()<< " constraintCorrections" <<sendl;

		for (unsigned int i=0; i<constraintCorrections.size(); i++)
		{
			core::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
			if (!cc->isActive()) continue;
			sofa::helper::AdvancedTimer::stepBegin("Object name: " + cc->getName());
			cc->addComplianceInConstraintSpace(cParams, &current_cp->W);
			sofa::helper::AdvancedTimer::stepEnd("Object name: " + cc->getName());
		}

		sofa::helper::AdvancedTimer::stepEnd  ("Get Compliance");
		if (this->f_printLog.getValue()) sout<<" computeCompliance_done "  <<sendl;
	}


	if ( displayTime.getValue() )
	{
		sout<<" build_LCP " << ( (double) timer.getTime() - time)*timeScale<<" ms" <<sendl;
		time = (double) timer.getTime();
	}

    return true;
}

void GenericConstraintSolver::rebuildSystem(double massFactor, double forceFactor)
{
    for (unsigned int i=0; i<constraintCorrections.size(); i++)
    {
            core::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
			if (!cc->isActive()) continue;
            //serr << "REBUILD " <<  cc->getName() << " m="<<massFactor << " f=" << forceFactor << sendl;
            cc->rebuildSystem(massFactor, forceFactor);
    }
}

void afficheLCP(std::ostream& file, double *q, double **M, double *f, int dim, bool printMatrix = true)
{
	file.precision(9);
	// affichage de la matrice du LCP
        if (printMatrix) {
        file << std::endl << " M = [";
	for(int compteur=0;compteur<dim;compteur++) {
		for(int compteur2=0;compteur2<dim;compteur2++) {
			file << "\t" << M[compteur][compteur2];
		}
		file << std::endl;
	}
        file << "      ];" << std::endl << std::endl;
        }

	// affichage de q
	file << " q = [";
	for(int compteur=0;compteur<dim;compteur++) {
		file << "\t" << q[compteur];
	}
	file << "      ];" << std::endl << std::endl;

	// affichage de f
	file << " f = [";
	for(int compteur=0;compteur<dim;compteur++) {
		file << "\t" << f[compteur];
	}
	file << "      ];" << std::endl << std::endl;
}

bool GenericConstraintSolver::solveSystem(const core::ConstraintParams* cParams, MultiVecId /*res1*/, MultiVecId /*res2*/)
{
	current_cp->tolerance = tolerance.getValue();
    
    if (cParams->constOrder() == core::ConstraintParams::VEL)
    {
        current_cp->tolerance /= this->getContext()->getDt();
    }

	current_cp->maxIterations = maxIt.getValue();
    current_cp->useInfiniteNorm = useInfiniteNorm.getValue();
	current_cp->sor = sor.getValue();
	current_cp->unbuilt = unbuilt.getValue();

	if (unbuilt.getValue())
	{
		sofa::helper::AdvancedTimer::stepBegin("ConstraintsUnbuiltGaussSeidel");
		current_cp->unbuiltGaussSeidel(this);
		sofa::helper::AdvancedTimer::stepEnd("ConstraintsUnbuiltGaussSeidel");
	}
	else
	{
        if (this->f_printLog.getValue())
        {
            std::cout << "---> Before Resolution" << std::endl;
            afficheLCP(std::cout, current_cp->getDfree(), current_cp->getW(), current_cp->getF(), current_cp->getDimension(), true);
        }

		sofa::helper::AdvancedTimer::stepBegin("ConstraintsGaussSeidel");
		current_cp->gaussSeidel(this);
		sofa::helper::AdvancedTimer::stepEnd("ConstraintsGaussSeidel");
	}

    // need to multiply by the time step size if in velocity mode to get the error 
    // expressed in the same unit as the tolerance
    const double currentError = cParams->constOrder() == core::ConstraintParams::VEL ?
        current_cp->currentError * this->getContext()->getDt() : current_cp->currentError;
    this->currentError.setValue(currentError);
    this->currentIterations.setValue(current_cp->currentIterations);
    this->currentNumConstraints.setValue(current_cp->getNumConstraints());
    this->currentNumConstraintGroups.setValue(current_cp->getNumConstraintGroups());

	if ( displayTime.getValue() )
	{
		sout<<" TOTAL solve_LCP " <<( (double) timer.getTime() - time)*timeScale<<" ms" <<sendl;
		time = (double) timer.getTime();
	}

    if(this->f_printLog.getValue())
    {
        std::cout << "---> After Resolution" << std::endl;
        afficheLCP(std::cout, current_cp->_d.ptr(), current_cp->getW(), current_cp->getF(), current_cp->getDimension(), false);
    }

    if (d_dumpSystem.getValue())
    {
        std::cout << "\n[ step " << this->getContext()->getTime() / this->getContext()->getDt() << " ] ====  Constraint problem solved : ==== "  << std::endl;
        std::cout << *current_cp << std::endl;
    }
	
	return true;
}

void GenericConstraintSolver::computeResidual(const core::ExecParams* eparam) {
    for (unsigned int i=0; i<constraintCorrections.size(); i++) {
        core::behavior::BaseConstraintCorrection* cc = constraintCorrections[i];
        cc->computeResidual(eparam,&current_cp->f);
    }
}


bool GenericConstraintSolver::applyCorrection(const core::ConstraintParams *cParams, MultiVecId res1, MultiVecId res2)
{
	using sofa::helper::AdvancedTimer;
	using core::behavior::BaseConstraintCorrection;

	if (this->f_printLog.getValue())
        serr << "KeepContactForces done" << sendl;

    AdvancedTimer::stepBegin("Compute And Apply Motion Correction");
    
	if (cParams->constOrder() == core::ConstraintParams::POS_AND_VEL)
	{
        core::MultiVecCoordId xId(res1);
		core::MultiVecDerivId vId(res2);
		for (unsigned int i = 0; i < constraintCorrections.size(); i++)
		{
			BaseConstraintCorrection* cc = constraintCorrections[i];
			if (!cc->isActive()) continue;

            sofa::helper::AdvancedTimer::stepBegin("ComputeCorrection on: " + cc->getName());
            cc->computeMotionCorrectionFromLambda(cParams, this->getDx(), &current_cp->f);
            sofa::helper::AdvancedTimer::stepEnd("ComputeCorrection on: " + cc->getName());

			sofa::helper::AdvancedTimer::stepBegin("ApplyCorrection on: " + cc->getName());
			cc->applyMotionCorrection(cParams, xId, vId, cParams->dx(), this->getDx() );
			sofa::helper::AdvancedTimer::stepEnd("ApplyCorrection on: " + cc->getName());
		}
	}
	else if (cParams->constOrder() == core::ConstraintParams::POS)
	{
		core::MultiVecCoordId xId(res1);
		for (unsigned int i = 0; i < constraintCorrections.size(); i++)
		{
			BaseConstraintCorrection* cc = constraintCorrections[i];
			if (!cc->isActive()) continue;

            sofa::helper::AdvancedTimer::stepBegin("ComputeCorrection on: " + cc->getName());
            cc->computeMotionCorrectionFromLambda(cParams, this->getDx(), &current_cp->f);
            sofa::helper::AdvancedTimer::stepEnd("ComputeCorrection on: " + cc->getName());

			sofa::helper::AdvancedTimer::stepBegin("ApplyCorrection on: " + cc->getName());
			cc->applyPositionCorrection(cParams, xId, cParams->dx(), this->getDx());
			sofa::helper::AdvancedTimer::stepEnd("ApplyCorrection on: " + cc->getName());
		}
	}
	else if (cParams->constOrder() == core::ConstraintParams::VEL)
	{
        core::MultiVecDerivId vId = res2.isNull() ? core::MultiVecDerivId(res1) : core::MultiVecDerivId(res2);
		for (unsigned int i = 0; i < constraintCorrections.size(); i++)
		{
			BaseConstraintCorrection* cc = constraintCorrections[i];
			if (!cc->isActive()) continue;

            sofa::helper::AdvancedTimer::stepBegin("ComputeCorrection on: " + cc->getName());
            cc->computeMotionCorrectionFromLambda(cParams, this->getDx(), &current_cp->f);
            sofa::helper::AdvancedTimer::stepEnd("ComputeCorrection on: " + cc->getName());

			sofa::helper::AdvancedTimer::stepBegin("ApplyCorrection on: " + cc->getName());
			cc->applyVelocityCorrection(cParams, vId, cParams->dx(), this->getDx() );
			sofa::helper::AdvancedTimer::stepEnd("ApplyCorrection on: " + cc->getName());
		}
	}

    AdvancedTimer::stepEnd("Compute And Apply Motion Correction");

	if (this->f_printLog.getValue())
		serr << "Compute And Apply Motion Correction in constraintCorrection done" << sendl;

    AdvancedTimer::stepBegin("Store Constraint Lambdas");

    /// Some constraint correction schemes may have written the constraint motion space lambda in the lambdaId VecId.
    /// In order to be sure that we are not accumulating things twice, we need to clear.
    clearMultiVecId(getContext(), cParams, m_lambdaId);

    /// Store lambda and accumulate.
    sofa::simulation::ConstraintStoreLambdaVisitor v(cParams, &current_cp->f);
    this->getContext()->executeVisitor(&v);
    AdvancedTimer::stepEnd("Store Constraint Lambdas");

	if (displayTime.getValue())
    {
        sout << " TotalTime " << ((double) timerTotal.getTime() - timeTotal) * timeScale << " ms" << sendl;
    }

    return true;
}


ConstraintProblem* GenericConstraintSolver::getConstraintProblem()
{
	return last_cp;
}

void GenericConstraintSolver::clearConstraintProblemLocks()
{
    for (unsigned int i = 0; i < CP_BUFFER_SIZE; ++i)
    {
        m_cpIsLocked[i] = false;
    }
}

void GenericConstraintSolver::lockConstraintProblem(sofa::core::objectmodel::BaseObject* from, ConstraintProblem* p1, ConstraintProblem* p2)
{
	if( (current_cp != p1) && (current_cp != p2) ) // The current ConstraintProblem is not locked
		return;

    for (unsigned int i = 0; i < CP_BUFFER_SIZE; ++i)
    {
        GenericConstraintProblem* p = &m_cpBuffer[i];
        if (p == p1 || p == p2)
        {
            m_cpIsLocked[i] = true;
        }
        if (!m_cpIsLocked[i]) // ConstraintProblem i is not locked
        {
            current_cp = p;
            return;
        }
    }
    // All constraint problems are locked
    serr << "All constraint problems are locked, request from " << (from ? from->getName() : "NULL") << " ignored" << sendl;
}

void GenericConstraintProblem::clear(int nbC)
{
	ConstraintProblem::clear(nbC);

	freeConstraintResolutions();
	constraintsResolutions.resize(nbC);
	_d.resize(nbC);
}

void GenericConstraintProblem::freeConstraintResolutions()
{
	for(unsigned int i=0; i<constraintsResolutions.size(); i++)
	{
		if (constraintsResolutions[i] != NULL)
		{
			delete constraintsResolutions[i];
			constraintsResolutions[i] = NULL;
		}
	}
}
int GenericConstraintProblem::getNumConstraints()
{
    return dimension;
}

int GenericConstraintProblem::getNumConstraintGroups()
{
    int n = 0;
    for(int i=0; i<dimension; )
    {
        if(!constraintsResolutions[i])
        {
            break;
        }
        ++n;
        i += constraintsResolutions[i]->getNbLines();
    }
    return n;
}

void GenericConstraintProblem::solveTimed(double tol, int maxIt, double timeout)
{
// TODO : for the unbuild version to work in the haptic thread, we have to duplicate the ConstraintCorrections first...
/*	if(unbuilt)
		unbuiltGaussSeidel(timeout);
	else
*/
    std::pair<int,double> res = gaussSeidel(NULL, tol, maxIt, timeout, getDfree(), _d.ptr(), getF());
    currentIterations = res.first;
    currentError = res.second;
}

std::pair<int,double> GenericConstraintProblem::solveTimed(double tol, int maxIt, double timeout, const double* localDFree, double* localD, double* localF) const
{
    std::pair<int,double> res(0,0); // TODO: nlcp_gaussseidelTimed does not return iterations and error

// TODO : for the unbuild version to work in the haptic thread, we have to duplicate the ConstraintCorrections first...
/*	if(unbuilt)
		unbuiltGaussSeidel(timeout);
	else
*/
    res = gaussSeidel(NULL, tol, maxIt, timeout, localDFree, localD, localF);
    return res;
}

void GenericConstraintProblem::gaussSeidel(GenericConstraintSolver* solver, ConstraintResolutionFunctor&& functor )
{
    std::pair<int,double> res = gaussSeidel(solver, tolerance, maxIterations, 0, getDfree(), _d.ptr(), getF(), std::move(functor) );
    currentIterations = res.first;
    currentError = res.second;
}

/// Block gauss seidel method
/// We are looking for \f$ f \f$ which solves \f$ W f = \delta^{free} \f$ 
/// the equality case here is for bilateral constraints, inequalities (unilateral) constraints are also supported
/// If we consider only blocks of size one, we end up writing at iteration \f$ k \$
/// \f[
///    f_{i}^{k} = \frac{1}{w_{ii}} \left [  \delta^{free}_i - \sum_{j=0}^{i-1}w_{ij}f_j^k - \sum_{j=i+1}^{n}w_{ij}f_j^{k-1} \right ]
/// \f]
/// If the system only contains equality constraints The residual vector \f$ r^k \f$ can be defined as 
/// \f[
///     r^k = \delta^{free} - W f^k
/// \f]
/// However since we are dealing also with inequalities we use instead 
/// \f[
///     r^k = W ( f^k - f^{k-1} )
/// \f]
/// From there different norm can be used to evaluate the residual vector.
// Debug is only available when called directly by the solver (not in haptic thread)
std::pair<int,double> GenericConstraintProblem::gaussSeidel(GenericConstraintSolver* solver, double tol, int maxIt, double timeout, const double* dfree, double* d, double* force,
    ConstraintResolutionFunctor&& functor) const
{
	if(!dimension)
    {
        return std::make_pair(0,0.0);
    }

    double t0 = (double)sofa::helper::system::thread::CTime::getTime() ;
    double timeScale = 1.0 / (double)sofa::helper::system::thread::CTime::getTicksPerSec();

	const double * const * const w = getW();
    int dim = dimension;

	bool convergence = true;

	if(solver)
	{
		for(int i=0; i<dim; )
		{
			if(!constraintsResolutions[i])
			{
				std::cerr << "Bad size of constraintsResolutions in GenericConstraintProblem" << std::endl;
				dim = i;
				break;
			}
            constraintsResolutions[i]->addLocalCompliance(i, const_cast<double**>(w));
			constraintsResolutions[i]->init(i, const_cast<double**>(w), force);
			i += constraintsResolutions[i]->getNbLines();
		}
	}

	bool showGraphs = false;
	sofa::helper::vector<double>* graph_residuals = NULL;
	std::map < std::string, sofa::helper::vector<double> > *graph_forces = NULL, *graph_violations = NULL;
	sofa::helper::vector<double> tabErrors;

	if(solver)
	{
		showGraphs = solver->computeGraphs.getValue();

		if(showGraphs)
		{
			graph_forces = solver->graphForces.beginEdit();
			graph_forces->clear();

			graph_violations = solver->graphViolations.beginEdit();
			graph_violations->clear();

			graph_residuals = &(*solver->graphErrors.beginEdit())["Error"];
			graph_residuals->clear();
		}
	
		tabErrors.resize(dim);
	}

    // used for SOR scheme, and for residual evaluation.
    std::vector<double> force_previous(dim);
    std::copy(&force[0], &force[dim], force_previous.begin());

    std::vector<double> dforce(dim,0);
    std::vector<double> residual(dim, 0);

    int i=0;
    double error= 0.0;
    for(; i<maxIt; ++i)
	{		
        for(int j=0; j<dim; )
		{
			//nbLines provides the dimension of the constraint block
			const unsigned blockSize = constraintsResolutions[j]->getNbLines();

			// update d entries for the constraint block, ( ie the constraint rhs to use for the block), 
            // given the value of the constraint force
            // Note that this loop also contains the contribution coming from the force of the current constraint block
            // which will need to be cancelled in the constraint resolution functor ( see remark below ) ...
            for(unsigned l=0; l<blockSize; ++l)
			{
                d[j+l] = dfree[j+l];
                for (int c=0; c<dim; ++c)
                {
                    d[j+l] += w[j+l][c] * force[c];
                }
			}

			// the functor updates the constraint force from d, the current constraint rhs
            // internally this method must cancel the contribution of the displacement that arised from the constraint 
            // force from the previous iteration. 
            // ie compute at iterate \f$ k \f$
            /// \f[
            ///  d_{offset} = w_{block} f_{block}^{k-1} //
            ///  d = d - d_{offset}
            /// \f]
            functor( constraintsResolutions[j], j, const_cast<double**>(w), d, force, const_cast<double*>(dfree) );

			j += blockSize;
		}

        for (int line =0; line < dim; ++line)
        {
            dforce[line] = force[line] - force_previous[line];
        }

#ifdef GENERIC_CONSTRAINT_SOLVER_USE_BLOCKDIAGONAL_RESIDUAL
        // old residual computation
        for (int line =0; line < dim; )
        {
            const ConstraintResolution* cr = constraintsResolutions[line];
            const std::size_t blockSize    = cr->getNbLines();

            for (int l = 0; l<blockSize; ++l)
            {
                residual[line + l] = 0;

                for (int c = 0; c<blockSize; ++c)
                {
                    residual[line+l] += w[line+l][line+c] * dforce[line+c];
                }
                residual[line+l] *=  residual[line+l];
            }
            line += blockSize;
        }
#else
        // Compute the residual vector as \f$ W ( f^k - f^{k-1} ) \f$
        for (int line=0; line<dim; ++line)
        {
            residual[line] = 0;
            for (int col=0; col<dim; ++col)
            {
                residual[line] += w[line][col] * dforce[col];
            }
            residual[line] *= residual[line];
        }
#endif
        // Due to the fact that each constraint block can define its own tolerance value, 
        // we need to iterate over the blocks to evaluate the convergence criteria.
        error = 0.0;
        convergence = true;
        for (int line =0; line<dim; )
        {
            const ConstraintResolution* cr = constraintsResolutions[line];
            const std::size_t blockSize    = cr->getNbLines();
            
            const double tolerance = cr->getTolerance() != 0 ? cr->getTolerance() : tol;
            const double tolerance2 = tolerance * tolerance;
            const auto residualBegin = residual.begin()+line;
            const auto residualEnd   = residualBegin + blockSize;
            const double res = useInfiniteNorm ? *std::max_element(residualBegin, residualEnd) :
                                                  std::accumulate(residualBegin, residualEnd, double(0)) / double(blockSize);

            if (res > tolerance2)
            {
                convergence = false;
            }

            if (res > error)
            {
                error = res;
            }

            line += blockSize;
        }

        error = std::sqrt(error);

		if(showGraphs)
		{
			for(int j=0; j<dim; ++j)
			{
				std::ostringstream oss;
				oss << "f" << j;

				sofa::helper::vector<double>& graph_force = (*graph_forces)[oss.str()];
				graph_force.push_back(force[j]);

				sofa::helper::vector<double>& graph_violation = (*graph_violations)[oss.str()];
				graph_violation.push_back(d[j]);
			}
		
			graph_residuals->push_back(error);
		}

		if(sor != 1.0)
		{
			for(int j=0; j<dim; ++j)
				force[j] = sor * force[j] + (1-sor) * force_previous[j];
		}

        std::copy(&force[0], &force[dim], force_previous.begin());

        double t1 = (double)sofa::helper::system::thread::CTime::getTime();
		double dt = (t1 - t0)*timeScale;

		if(timeout && dt > timeout)
		{
			if(solver && solver->f_printLog.getValue())
				std::cout << "TimeOut" << std::endl;
            
			return std::make_pair(i+1, error);
		}

        if (convergence) {
            break;
        }
	}

    std::pair<int,double> result(i+1, error);

	sofa::helper::AdvancedTimer::valSet("GS iterations", currentIterations);

	if(solver)
	{
		if(!convergence)
		{
			if(solver->f_printLog.getValue())
				solver->serr << "No convergence : error = " << std::sqrt(error) << solver->sendl;
			else
				solver->sout << "No convergence : error = " << std::sqrt(error) << solver->sendl;
		}
		else if(solver->displayTime.getValue())
			solver->sout<<" Convergence after " << i+1 << " iterations " << solver->sendl;

		for(i=0; i<dimension; i += constraintsResolutions[i]->getNbLines())
			constraintsResolutions[i]->store(i, force, convergence);
	}

/*
    if(schemeCorrection)
    {
        ///////// scheme correction : step 3 => the corrective motion is only based on the diff of the force value: compute this diff
        for(j=0; j<dim; j++)
        {
            df[j] += force[j];
        }
    }	*/

	if(showGraphs)
	{
		solver->graphErrors.endEdit();

		sofa::helper::vector<double>& graph_constraints = (*solver->graphConstraints.beginEdit())["Constraints"];
		graph_constraints.clear();

		for(int j=0; j<dim; )
		{
			const unsigned int nb = constraintsResolutions[j]->getNbLines();

			if(tabErrors[j])
				graph_constraints.push_back(tabErrors[j]);
			else if(constraintsResolutions[j]->getTolerance())
				graph_constraints.push_back(constraintsResolutions[j]->getTolerance());
			else
				graph_constraints.push_back(tol);

			j += nb;
		}
		solver->graphConstraints.endEdit();

		solver->graphForces.endEdit();
	}

    return result;
}


void GenericConstraintSolver::postStabilize(const core::ConstraintParams* cParams, MultiVecCoordId xId, MultiVecDerivId /*vId*/)
{
    using sofa::helper::AdvancedTimer;
    using core::behavior::BaseConstraintCorrection;

    AdvancedTimer::stepBegin("PostStabilize");

    // gather the positional error
    {
        core::ConstraintParams stabilizationParams(*cParams);
        stabilizationParams.setOrder(core::ConstraintParams::POS);
        MechanicalGetConstraintViolationVisitor(&stabilizationParams, &current_cp->dFree).execute(context);
    }

    if (cParams->constOrder() == core::ConstraintParams::VEL)
    {
        const double invDt = 1. / this->getContext()->getDt();
        current_cp->dFree *= invDt;
    }

    current_cp->f.clear();
    current_cp->gaussSeidel(this, std::mem_fn(&core::behavior::ConstraintResolution::postStabilize));

    for (unsigned int i = 0; i < constraintCorrections.size(); i++)
    {
        BaseConstraintCorrection* cc = constraintCorrections[i];
        if (!cc->isActive()) continue;
        cc->computeMotionCorrectionFromLambda(cParams, this->getDx(), &current_cp->f);
        cc->applyPositionCorrection(cParams, xId, cParams->dx(), this->getDx());
    }

    AdvancedTimer::stepEnd("PostStabilize");
}


void GenericConstraintProblem::unbuiltGaussSeidel(GenericConstraintSolver* solver, double timeout)
{
	if(!dimension)
    {
        currentError = 0.0;
        currentIterations = 0;
        return;
    }

    double t0 = (double)sofa::helper::system::thread::CTime::getTime();
    double timeScale = 1.0 / (double)sofa::helper::system::thread::CTime::getTicksPerSec();

	double *dfree = getDfree();
	double *force = getF();
	double **w = getW();
	double tol = tolerance;

	double *d = _d.ptr();

	int i, j, l, nb;

	double errF[6];
	double error=0.0;

	bool convergence = false;
    sofa::helper::vector<double> tempForces;
	if(sor != 1.0) tempForces.resize(dimension);

    tol *= dimension;

	if(solver)
	{
        memset(force, 0, dimension * sizeof(double));	// Erase previous forces for the time being

		for(i=0; i<dimension; )
		{
			if(!constraintsResolutions[i])
			{
				std::cerr << "Bad size of constraintsResolutions in GenericConstraintProblem" << std::endl;
				dimension = i;
				break;
			}
			constraintsResolutions[i]->init(i, w, force);
			i += constraintsResolutions[i]->getNbLines();
		}
	}
	
	bool showGraphs = false;
	sofa::helper::vector<double>* graph_residuals = NULL;
	std::map < std::string, sofa::helper::vector<double> > *graph_forces = NULL, *graph_violations = NULL;
	sofa::helper::vector<double> tabErrors;

	if(solver)
	{
		showGraphs = solver->computeGraphs.getValue();

		if(showGraphs)
		{
			graph_forces = solver->graphForces.beginEdit();
			graph_forces->clear();

			graph_violations = solver->graphViolations.beginEdit();
			graph_violations->clear();

			graph_residuals = &(*solver->graphErrors.beginEdit())["Error"];
			graph_residuals->clear();
		}
	
		tabErrors.resize(dimension);
	}

	for(i=0; i<maxIterations; i++)
	{
		//bool constraintsAreVerified = true;
        if(sor != 1.0)
		{
			for(j=0; j<dimension; j++)
				tempForces[j] = force[j];
		}

		error=0.0;
		for(j=0; j<dimension; ) // increment of j realized at the end of the loop
		{
			//1. nbLines provide the dimension of the constraint  (max=6)
			nb = constraintsResolutions[j]->getNbLines();

			//2. for each line we compute the actual value of d
			//   (a)d is set to dfree
			for(l=0; l<nb; l++)
			{
				errF[l] = force[j+l];
				d[j+l] = dfree[j+l];
			}

			//   (b) contribution of forces are added to d
			for (ConstraintCorrectionIterator iter=cclist_elems[j].begin(); iter!=cclist_elems[j].end(); ++iter)
			{
				if(*iter)
					(*iter)->addConstraintDisplacement(d, j, j+nb-1);
			}

			//3. the specific resolution of the constraint(s) is called
			constraintsResolutions[j]->resolution(j, w, d, force, dfree);

			//4. the error is measured (displacement due to the new resolution (i.e. due to the new force))
			double contraintError = 0.0;
			if(nb > 1)
			{
				for(l=0; l<nb; l++)
				{
					double lineError = 0.0;
					for (int m=0; m<nb; m++)
					{
						double dofError = w[j+l][j+m] * (force[j+m] - errF[m]);
						lineError += dofError * dofError;
					}
					lineError = sqrt(lineError);
					//if(lineError > tol)
					//	constraintsAreVerified = false;

					contraintError += lineError;
				}
			}
			else
			{
				contraintError = fabs(w[j][j] * (force[j] - errF[0]));
				//if(contraintError > tol)
				//	constraintsAreVerified = false;
			}

			if(constraintsResolutions[j]->getTolerance())
			{
				//if(contraintError > constraintsResolutions[j]->getTolerance())
				//	constraintsAreVerified = false;
				contraintError *= tol / constraintsResolutions[j]->getTolerance();
			}

			error += contraintError;
			if(solver)
				tabErrors[j] = contraintError;

			//5. the force is updated for the constraint corrections
			bool update = false;
			for(l=0; l<nb; l++)
				update |= (force[j+l] || errF[l]);

			if(update)
			{
				double tempF[6];
				for(l=0; l<nb; l++)
				{
					tempF[l] = force[j+l];
					force[j+l] -= errF[l]; // DForce
				}

				for (ConstraintCorrectionIterator iter=cclist_elems[j].begin(); iter!=cclist_elems[j].end(); ++iter)
				{
					if(*iter)
						(*iter)->setConstraintDForce(force, j, j+nb-1, update);
				}

				for(l=0; l<nb; l++)
					force[j+l] = tempF[l];
			}

			j += nb;
		}

		if(showGraphs)
		{
			for(j=0; j<dimension; j++)
			{
				std::ostringstream oss;
				oss << "f" << j;

				sofa::helper::vector<double>& graph_force = (*graph_forces)[oss.str()];
				graph_force.push_back(force[j]);

				sofa::helper::vector<double>& graph_violation = (*graph_violations)[oss.str()];
				graph_violation.push_back(d[j]);
			}

			graph_residuals->push_back(error);
		}

		if(sor != 1.0)
		{
			for(j=0; j<dimension; j++)
				force[j] = sor * force[j] + (1-sor) * tempForces[j];
		}
        if(timeout)
        {
            double t1 = (double)sofa::helper::system::thread::CTime::getTime();
            double dt = (t1 - t0)*timeScale;

            if(dt > timeout)
            {
                currentError = error;
                currentIterations = i+1;
                return;
            }
        }
        else if (error < tol/* && i>0*/) // do not stop at the first iteration (that is used for initial guess computation)
		{
			convergence = true;
			break;
		}
	}
    
    currentError = error;
    currentIterations = i+1;

	sofa::helper::AdvancedTimer::valSet("GS iterations", currentIterations);

	if(solver)
	{
		if(!convergence)
		{
			if(solver->f_printLog.getValue())
				solver->serr << "No convergence : error = " << error << solver->sendl;
			else
				solver->sout << "No convergence : error = " << error << solver->sendl;
		}
		else if(solver->displayTime.getValue())
			solver->sout<<" Convergence after " << i+1 << " iterations " << solver->sendl;

		for(i=0; i<dimension; i += constraintsResolutions[i]->getNbLines())
			constraintsResolutions[i]->store(i, force, convergence);
	}

	if(showGraphs)
	{
		solver->graphErrors.endEdit();

		sofa::helper::vector<double>& graph_constraints = (*solver->graphConstraints.beginEdit())["Constraints"];
		graph_constraints.clear();

		for(j=0; j<dimension; )
		{
			nb = constraintsResolutions[j]->getNbLines();

			if(tabErrors[j])
				graph_constraints.push_back(tabErrors[j]);
			else if(constraintsResolutions[j]->getTolerance())
				graph_constraints.push_back(constraintsResolutions[j]->getTolerance());
			else
				graph_constraints.push_back(tol);

			j += nb;
		}
		solver->graphConstraints.endEdit();

		solver->graphForces.endEdit();
	}
}



int GenericConstraintSolverClass = core::RegisterObject("A Generic Constraint Solver using the Linear Complementarity Problem formulation to solve Constraint based components")
.add< GenericConstraintSolver >();

SOFA_DECL_CLASS(GenericConstraintSolver)


} // namespace constraintset

} // namespace component

} // namespace sofa
