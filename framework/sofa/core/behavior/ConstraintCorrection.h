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
#ifndef SOFA_CORE_BEHAVIOR_CONSTRAINTCORRECTION_H
#define SOFA_CORE_BEHAVIOR_CONSTRAINTCORRECTION_H

#include <sofa/core/behavior/BaseConstraintCorrection.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>


namespace sofa
{

namespace core
{

namespace behavior
{

/**
 * Component computing contact forces within a simulated body using the compliance method.
 */
template<class TDataTypes>
class ConstraintCorrection : public BaseConstraintCorrection
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((ConstraintCorrection<TDataTypes>), ((BaseConstraintCorrection)));

    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
    typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;
    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;

protected:
    /// Default Constructor
    ConstraintCorrection(MechanicalState< DataTypes > *ms = NULL)
        : mstate(ms)
    {
    };

    /// Default Destructor
    virtual ~ConstraintCorrection()
    {
    };
public:
    virtual void init() override;

 

    /// Compute the motion coming from the contact space lambda  
    /// dx = A^-1 x J^t x lambda 
    /// where :
    /// - J is the constraint jacobian matrix ( ^t denotes the transposition operator )
    /// - A is the dynamic matrix. Usually for implicit integration A = M - h^2 x K with
    /// -- M the mass matrix 
    /// -- K the stiffness matrix
    /// -- h the step size.
    /// Usually this computation will be delegated to a LinearSolver instance 
    /// 
    /// @param cparams the ConstraintParams relative to the constraint solver
    /// @param dx the VecId where to store the corrective motion
    /// @param lambda is the constraint space force vector
    virtual void computeMotionCorrectionFromLambda(const core::ConstraintParams* cparams, core::MultiVecDerivId dx, const defaulttype::BaseVector * lambda) override;


    /// Compute the corrective motion coming from the motion space force
   
    /// @param cparams the ConstraintParams relative to the constraint solver
    /// @param dx the VecId where to store the corrective motion
    /// @param f  is the VecId where the motion space force : f = J^t x lambda
    virtual void computeMotionCorrection(const core::ConstraintParams* cparams, core::MultiVecDerivId dx, core::MultiVecDerivId f) = 0;


    /// Compute motion correction from the constraint resolution (LCP) calculated force
    ///
    /// @param cparams
    /// @param x is the position result VecId
    /// @param v is the velocity result VecId
    /// @param dx if the corrective motion result VecId
    /// @param f is the motion space force vector
    virtual void applyMotionCorrection(const core::ConstraintParams * cparams, core::MultiVecCoordId x, core::MultiVecDerivId v, core::MultiVecDerivId dx, core::ConstMultiVecDerivId correction) override;

    virtual void applyMotionCorrection(const core::ConstraintParams * cparams, Data< VecCoord > &x, Data< VecDeriv > &v, Data< VecDeriv > &dx, const Data< VecDeriv > & correction) = 0;

    /// Compute position correction from the constraint resolution (LCP) calculated force
    ///
    /// @param cparams
    /// @param x is the position result VecId
    /// @param dx id the corrective position result VecId
    /// @param f is the motion space force vector
    virtual void applyPositionCorrection(const core::ConstraintParams * cparams, core::MultiVecCoordId x, core::MultiVecDerivId dx, core::ConstMultiVecDerivId correction) override;

    virtual void applyPositionCorrection(const core::ConstraintParams * cparams, Data< VecCoord > &x, Data<VecDeriv>& dx,  const Data< VecDeriv > &correction)  = 0;

    /// Compute velocity correction from the constraint resolution (LCP) calculated force
    ///
    /// @param cparams
    /// @param v is the velocity result VecId
    /// @param dv is the corrective velocity result VecId
    /// @param f is the motion space force vector
    virtual void applyVelocityCorrection(const core::ConstraintParams * cparams, core::MultiVecDerivId v, core::MultiVecDerivId dv, core::ConstMultiVecDerivId correction) override;

    virtual void applyVelocityCorrection(const core::ConstraintParams * cparams, Data< VecDeriv > &v, Data<VecDeriv>& dv , const Data< VecDeriv > &correction) = 0;

    /// Apply predictive constraint force
    ///
    /// @param cparams
    /// @param f is the motion space force vector
    /// @param lambda is the constraint space force vector
    virtual void applyPredictiveConstraintForce(const core::ConstraintParams * cparams, core::MultiVecDerivId f, const defaulttype::BaseVector *lambda);

 
    /// Pre-construction check method called by ObjectFactory.
    template< class T >
    static bool canCreate(T*& obj, objectmodel::BaseContext* context, objectmodel::BaseObjectDescription* arg)
    {
        if (MechanicalState<DataTypes>::DynamicCast(context->getMechanicalState()) == NULL)
            return false;

        return BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const ConstraintCorrection<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

    MechanicalState<DataTypes> *getMState() const
    {
        return mstate;
    }

    void setMState(MechanicalState<DataTypes> *_mstate)
    {
        mstate = _mstate;
    }

protected:
    MechanicalState<DataTypes> *mstate;

private:
    /// Converts constraint force from the constraints space to the motion space and accumulates it in f vector
    ///
    /// @param f is the motion space force vector
    /// @param lambda is the constraint space force vector
    void addConstraintForceInMotionSpace(const core::ConstraintParams* cparams, core::MultiVecDerivId f, core::ConstMultiMatrixDerivId j, const defaulttype::BaseVector * lambda);

    void addConstraintForceInMotionSpace(const core::ConstraintParams* cparams, Data< VecDeriv > &f, const Data<MatrixDeriv>& j, const defaulttype::BaseVector * lambda);
};


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_CORE)
#ifndef SOFA_FLOAT
extern template class SOFA_CORE_API ConstraintCorrection< sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_CORE_API ConstraintCorrection< sofa::defaulttype::Vec1dTypes >;
extern template class SOFA_CORE_API ConstraintCorrection< sofa::defaulttype::Rigid3dTypes >;
#endif

#ifndef SOFA_DOUBLE
extern template class SOFA_CORE_API ConstraintCorrection< sofa::defaulttype::Vec1fTypes >;
extern template class SOFA_CORE_API ConstraintCorrection< sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_CORE_API ConstraintCorrection< sofa::defaulttype::Rigid3fTypes >;
#endif
#endif

} // namespace behavior

} // namespace core

} // namespace sofa

#endif // SOFA_CORE_BEHAVIOR_CONSTRAINTCORRECTION_H
