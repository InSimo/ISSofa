/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CORE_BEHAVIOR_BASECONSTRAINTCORRECTION_H
#define SOFA_CORE_BEHAVIOR_BASECONSTRAINTCORRECTION_H

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/ConstraintSolver.h>
#include <sofa/core/ConstraintParams.h>

namespace sofa
{
namespace defaulttype
{
class BaseMatrix;
class BaseVector;
} // namespace defaulttype
} // namespace sofa

namespace sofa
{

namespace core
{

namespace behavior
{

/// @todo All methods in this class need to be commented

/**
 *  \brief Component computing contact forces within a simulated body using the compliance method.
 */
class SOFA_CORE_API BaseConstraintCorrection : public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS_EXTERNAL((BaseConstraintCorrection), ((objectmodel::BaseObject)));
protected:
    BaseConstraintCorrection();
    virtual ~BaseConstraintCorrection();

private:
	BaseConstraintCorrection(const BaseConstraintCorrection& n) ;
	BaseConstraintCorrection& operator=(const BaseConstraintCorrection& n) ;

	
public:

	virtual bool isActive() { return this->getContext()->isActive(); }

    virtual void init() override;

	/// @name Compliance Matrix API
    /// @{

    virtual void addComplianceInConstraintSpace(const ConstraintParams *, defaulttype::BaseMatrix* W) = 0;

    /// Fill the matrix m with the full Compliance Matrix
    virtual void getComplianceMatrix(defaulttype::BaseMatrix* m) const = 0;

    /// For multigrid approach => constraints are merged
    virtual void getComplianceWithConstraintMerge(defaulttype::BaseMatrix* /*Wmerged*/, std::vector<int> & /*constraint_merge*/)
    {
        sout << "getComplianceWithConstraintMerge is not implemented yet " << sendl;
    }

    /// @}


    /// Compute the corrective motion from the constraint space lambda
    ///
    /// @param cparams the ConstraintParams relative to the constraint solver
    /// @param dx the VecId where to store the corrective motion
    /// @param lambda is the constraint space force vector
    virtual void computeMotionCorrectionFromLambda(const core::ConstraintParams* cparams, core::MultiVecDerivId dx, const defaulttype::BaseVector * lambda) = 0;


    /// Compute motion correction from the constraint resolution (LCP) calculated force
    ///
    /// @param cparams
    /// @param x is the position result VecId
    /// @param v is the velocity result VecId
    /// @param dx if the corrective motion result VecId
    virtual void applyMotionCorrection(const ConstraintParams * cparams, MultiVecCoordId x, MultiVecDerivId v, MultiVecDerivId dx, ConstMultiVecDerivId correction) = 0;

    /// Compute position correction from the constraint resolution (LCP) calculated force
    ///
    /// @param cparams
    /// @param x is the position result VecId
    /// @param v is the velocity result VecId
    /// @param dx if the corrective position result VecId
    virtual void applyPositionCorrection(const ConstraintParams * cparams, MultiVecCoordId x, MultiVecDerivId dx, ConstMultiVecDerivId correction) = 0;

    /// Compute velocity correction from the constraint resolution (LCP) calculated force
    ///
    /// @param cparams
    /// @param x is the position result VecId
    /// @param v is the velocity result VecId
    /// @param dv if the corrective velocity result VecId
    /// @param correction is the corrective motion computed from the constraint lambda
    virtual void applyVelocityCorrection(const ConstraintParams * cparams, MultiVecDerivId v, MultiVecDerivId dv, ConstMultiVecDerivId correction) = 0;

    /// Apply predictive constraint force
    ///
    /// @param cparams
    /// @param f is the motion space force vector
    /// @param lambda is the constraint space force vector
    virtual void applyPredictiveConstraintForce(const ConstraintParams * cparams, MultiVecDerivId f, const defaulttype::BaseVector * lambda) = 0;

    /// Rebuild the system using a mass and force factor
    /// Experimental API used to investigate convergence issues.
    virtual void rebuildSystem(double /*massFactor*/, double /*forceFactor*/){}

    /// Compute the residual in the newton iterations due to the constraints forces
    /// i.e. compute Vecid::force() += J^t lambda
    /// the result is accumulated in Vecid::force()
    virtual void computeResidual(const core::ExecParams* /*params*/ /* PARAMS FIRST */, defaulttype::BaseVector * /*lambda*/) { std::cerr << "ComputeResidual is not implemented in " << this->getName() << std::endl; }

    /// @name Deprecated API
    /// @{

    virtual void applyContactForce(const defaulttype::BaseVector *f) = 0;

    virtual void resetContactForce() = 0;

    /// @}


    /// @name Unbuilt constraint system during resolution
    /// @{

    virtual bool hasConstraintNumber(int /*index*/) {return true;}

    virtual void resetForUnbuiltResolution(double * /*f*/, std::list<unsigned int>& /*renumbering*/) {}

    virtual void addConstraintDisplacement(double * /*d*/, int /*begin*/, int /*end*/) {}

    virtual void setConstraintDForce(double * /*df*/, int /*begin*/, int /*end*/, bool /*update*/) {}	  // f += df

    virtual void getBlockDiagonalCompliance(defaulttype::BaseMatrix* /*W*/, int /*begin*/,int /*end*/)
    {
        sout << "warning : getBlockDiagonalCompliance(defaulttype::BaseMatrix* W) is not implemented in " << this->getTypeName() << sendl;
    }

    /// @}

protected:
    ConstraintSolver::SPtr m_constraintSolverPtr;
};

} // namespace behavior

} // namespace core

} // namespace sofa

#endif
