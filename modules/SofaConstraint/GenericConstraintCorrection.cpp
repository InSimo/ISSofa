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
#define SOFA_COMPONENT_CONSTRAINT_GENERICCONSTRAINTCORRECTION_CPP

#include "GenericConstraintCorrection.h"
#include <sofa/simulation/common/MechanicalMatrixVisitor.h>
#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa 
{

namespace component 
{

namespace constraintset 
{

GenericConstraintCorrection::GenericConstraintCorrection()
: solverName( initData(&solverName, "solverName", "name of the constraint solver") )
, d_complianceFactor(initData(&d_complianceFactor, 1.0, "complianceFactor", "Factor applied to the position factor and velocity factor used to calculate compliance matrix"))
{
    odesolver = NULL;
}

GenericConstraintCorrection::~GenericConstraintCorrection() {}

void GenericConstraintCorrection::bwdInit()
{
    core::objectmodel::BaseContext* c = this->getContext();

    c->get(odesolver, core::objectmodel::BaseContext::SearchUp);

    linearsolvers.clear();

    const helper::vector<std::string>& solverNames = solverName.getValue();
    if (solverNames.size() == 0) 
    {
        core::behavior::LinearSolver* s = NULL;
        c->get(s);
        if (s)
        {
            if (s->getTemplateName() == "GraphScattered") 
            {
                serr << "ERROR GenericConstraintCorrection cannot use the solver " << s->getName() << " because it is templated on GraphScatteredType" << sendl;
            }
            else
            {
                linearsolvers.push_back(s);
            }
        }
    }
    else 
    {
        for (unsigned int i=0; i<solverNames.size(); ++i)
        {
            core::behavior::LinearSolver* s = NULL;
            c->get(s, solverNames[i]);

            if (s)
            {
                if (s->getTemplateName() == "GraphScattered")
                {
                    serr << "ERROR GenericConstraintCorrection cannot use the solver " << solverNames[i] << " because it is templated on GraphScatteredType" << sendl;
                }
                else
                {
                    linearsolvers.push_back(s);
                }
            } 
            else serr << "Solver \"" << solverNames[i] << "\" not found." << sendl;
        }
    }

    if (odesolver == NULL)
    {
        serr << "GenericConstraintCorrection: ERROR no OdeSolver found."<<sendl;
        return;
    }

    if (linearsolvers.size() == 0)
    {
        serr << "GenericConstraintCorrection: ERROR no LinearSolver found."<<sendl;
        return;
    }

    sout << "Found " << linearsolvers.size() << " linearsolvers" << sendl;
    for (unsigned i = 0; i < linearsolvers.size(); i++)
    {
        sout << linearsolvers[i]->getName() << sendl;
    }
}

void GenericConstraintCorrection::rebuildSystem(double massFactor, double forceFactor)
{
    for (unsigned i = 0; i < linearsolvers.size(); i++)
    {
        linearsolvers[i]->rebuildSystem(massFactor, forceFactor);
    }
}

void GenericConstraintCorrection::addComplianceInConstraintSpace(const core::ConstraintParams *cparams, defaulttype::BaseMatrix* W)
{
    if (!odesolver) return;
    const double complianceFactor = d_complianceFactor.getValue();

    // use the OdeSolver to get the position integration factor
    double factor = 1.0;

    switch (cparams->constOrder())
    {
        case core::ConstraintParams::POS_AND_VEL :
        case core::ConstraintParams::POS :
            factor = odesolver->getPositionIntegrationFactor();
            break;

        case core::ConstraintParams::ACC :
        case core::ConstraintParams::VEL :
            factor = odesolver->getVelocityIntegrationFactor();
            break;

        default :
            break;
    }

    factor *= complianceFactor;
    // use the Linear solver to compute J*inv(M)*Jt, where M is the mechanical linear system matrix
    for (unsigned i = 0; i < linearsolvers.size(); i++)
    {
        linearsolvers[i]->buildComplianceMatrix(cparams, W, factor);
    }
}

void GenericConstraintCorrection::computeMotionCorrectionFromLambda(const core::ConstraintParams* cparams, core::MultiVecDerivId dx, const defaulttype::BaseVector * lambda)
{
    for (std::size_t i = 0; i < linearsolvers.size(); ++i)
    {
        linearsolvers[i]->applyConstraintForce(cparams, dx, lambda);
    }
}

void GenericConstraintCorrection::applyMotionCorrection(const core::ConstraintParams* cparams, core::MultiVecCoordId xId, core::MultiVecDerivId vId, 
    core::MultiVecDerivId dxId, core::ConstMultiVecDerivId correction, double positionFactor, double velocityFactor)
{
    for (std::size_t i = 0; i < linearsolvers.size(); ++i)
    {
        simulation::MechanicalIntegrateConstraintsVisitor v(cparams, positionFactor, velocityFactor, correction, dxId, xId, vId, linearsolvers[i]->getSystemMultiMatrixAccessor());
        linearsolvers[i]->getContext()->executeVisitor(&v);
    }
}

void GenericConstraintCorrection::applyMotionCorrection(const core::ConstraintParams * cparams, core::MultiVecCoordId xId, core::MultiVecDerivId vId, core::MultiVecDerivId dxId, core::ConstMultiVecDerivId correction)
{
    if (!odesolver) return;
    const double complianceFactor = d_complianceFactor.getValue();

    const double positionFactor = odesolver->getPositionIntegrationFactor() * complianceFactor;
    const double velocityFactor = odesolver->getVelocityIntegrationFactor() * complianceFactor;

    applyMotionCorrection(cparams, xId, vId, dxId, correction, positionFactor, velocityFactor);
}

void GenericConstraintCorrection::applyPositionCorrection(const core::ConstraintParams * cparams, core::MultiVecCoordId xId, core::MultiVecDerivId dxId, core::ConstMultiVecDerivId correctionId)
{
    if (!odesolver) return;

    const double complianceFactor = d_complianceFactor.getValue();
    const double positionFactor = odesolver->getPositionIntegrationFactor() * complianceFactor;

    applyMotionCorrection(cparams, xId, sofa::core::VecDerivId::null(), dxId, correctionId, positionFactor, 0);
}

void GenericConstraintCorrection::applyVelocityCorrection(const core::ConstraintParams * cparams, core::MultiVecDerivId vId, core::MultiVecDerivId dvId, core::ConstMultiVecDerivId correctionId)
{
    if (!odesolver) return;

    const double complianceFactor = d_complianceFactor.getValue();
    const double velocityFactor = odesolver->getVelocityIntegrationFactor() * complianceFactor;

    applyMotionCorrection(cparams, sofa::core::VecCoordId::null(), vId, dvId, correctionId, 0, velocityFactor);
}

void GenericConstraintCorrection::applyContactForce(const defaulttype::BaseVector *f)
{
    if (!odesolver) return;

    sofa::core::ConstraintParams cparams(*sofa::core::ExecParams::defaultInstance());


    computeMotionCorrectionFromLambda(&cparams, cparams.dx(), f);
    applyMotionCorrection(&cparams, sofa::core::VecCoordId::position(), sofa::core::VecDerivId::velocity(), cparams.dx(), cparams.lambda());
}

void GenericConstraintCorrection::computeResidual(const core::ExecParams* params, defaulttype::BaseVector *lambda)
{
    for (unsigned i = 0; i < linearsolvers.size(); i++)
    {
        linearsolvers[i]->computeResidual(params, lambda);
    }
}


void GenericConstraintCorrection::getComplianceMatrix(defaulttype::BaseMatrix* Minv) const
{
    if (!odesolver) return;

    sofa::core::ConstraintParams cparams(*sofa::core::ExecParams::defaultInstance());
    const_cast<GenericConstraintCorrection*>(this)->addComplianceInConstraintSpace(&cparams, Minv);
}

void GenericConstraintCorrection::applyPredictiveConstraintForce(const core::ConstraintParams * /*cparams*/, core::MultiVecDerivId /*f*/, const defaulttype::BaseVector * /*lambda*/)
{
//    printf("GenericConstraintCorrection::applyPredictiveConstraintForce not implemented\n");
//    if (mstate)
//    {
//        Data< VecDeriv > *f_d = f[mstate].write();

//        if (f_d)
//        {
//            applyPredictiveConstraintForce(cparams, *f_d, lambda);
//        }
//    }
}

void GenericConstraintCorrection::resetContactForce()
{
//    printf("GenericConstraintCorrection::resetContactForce not implemented\n");
//    Data<VecDeriv>& forceData = *this->mstate->write(core::VecDerivId::force());
//    VecDeriv& force = *forceData.beginEdit();
//    for( unsigned i=0; i<force.size(); ++i )
//        force[i] = Deriv();
//    forceData.endEdit();
}


SOFA_DECL_CLASS(GenericConstraintCorrection)

int GenericConstraintCorrectionClass = core::RegisterObject("")
.add< GenericConstraintCorrection >()
;

class SOFA_CONSTRAINT_API GenericConstraintCorrection;

} // namespace constraintset

} // namespace component

} // namespace sofa
