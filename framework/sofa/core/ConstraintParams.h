/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_CORE_CONSTRAINTPARAMS_H
#define SOFA_CORE_CONSTRAINTPARAMS_H

#include <sofa/core/ExecParams.h>
#include <sofa/core/MultiVecId.h>


namespace sofa
{

namespace core
{

/// Class gathering parameters use by constraint components methods, and transmitted by visitors
/// read the velocity and position
/// and where the 
class SOFA_CORE_API ConstraintParams : public sofa::core::ExecParams
{
public:

    /// Description of the order of the constraint
    enum ConstOrder
    {
        POS = 0,
        VEL,
        ACC,
        POS_AND_VEL
    };

    /// @name Flags and parameters getters
    /// @{

    ConstOrder constOrder() const { return m_constOrder; }

    ConstraintParams& setOrder(ConstOrder o) { m_constOrder = o;   return *this; }

	/// Smooth contribution factor (for smooth constraints resolution)
    double smoothFactor() const { return m_smoothFactor; }

    /// @}

    std::string getName() const
    {
        std::string result;
        switch ( m_constOrder )
        {
        case POS :
            result += "POSITION";
            break;
        case VEL :
            result += "VELOCITY";
            break;
        case ACC :
            result += "ACCELERATION";
            break;
        case POS_AND_VEL :
            result += "POSITION AND VELOCITY";
            break;
        default :
            assert(false);
        }
        return result;
    }

    /// @name Access to vectors from a given state container (i.e. State or MechanicalState)
    /// @{

    /// Read access to the free (unconstrained) position vector
    template<class S>
    const Data<typename S::VecCoord>* readX(const S* state) const
    {   return m_x[state].read();    }

    /// Read access to the free (unconstrained) velocity vector
    template<class S>
    const Data<typename S::VecDeriv>* readV(const S* state) const
    {   return m_v[state].read();    }

    /// Read access to the constraint jacobian matrix
    template<class S>
    const Data<typename S::MatrixDeriv>* readJ(const S* state) const
    {
        return m_j[state].read();
    }

    /// Read access to the constraint force vector
    template<class S>
    const Data<typename S::VecDeriv>* readLambda(S* state) const
    {
        return m_lambda[state].read();
    }

    /// Read access to the constraint corrective motion vector
    template<class S>
    const Data<typename S::VecDeriv>* readDx(S* state) const
    {
        return m_dx[state].read();
    }




    /// @name Access to vectors from a given SingleLink to a state container (i.e. State or MechanicalState)
    /// @{

    /// Read access to the free (unconstrained) position
    template<class Owner, class S, unsigned int flags>
    const Data<typename S::VecCoord>* readX(const SingleLink<Owner,S,flags>& state) const
    {   return m_x[state.get(this)].read();    }

    /// Read access to the free (unconstrained) velocity vector
    template<class Owner, class S, unsigned int flags>
    const Data<typename S::VecDeriv>* readV(const SingleLink<Owner,S,flags>& state) const
    {   return m_v[state.get(this)].read();    }

    /// Read access to the constraint jacobian matrix
    template<class Owner, class S, unsigned int flags>
    const Data<typename S::MatrixDeriv>* readJ(const SingleLink<Owner, S, flags>& state) const
    {
        return m_j[state.get(this)].read();
    }

    /// Read access to the constraint force vector
    template<class Owner, class S, unsigned int flags>
    const Data<typename S::VecDeriv>* readLambda(SingleLink<Owner, S, flags>& state) const
    {
        return m_lambda[state.get(this)].read();
    }

    /// Read access to the constraint corrective motion vector
    template<class Owner, class S, unsigned int flags>
    const Data<typename S::VecDeriv>* readDx(SingleLink<Owner, S, flags>& state) const
    {
        return m_dx[state.get(this)].read();
    }


    /// @}

    /// @name Setup methods
    /// Called by the OdeSolver from which the mechanical computations originate.
    /// They all return a reference to this MechanicalParam instance, to ease chaining multiple setup calls.

    /// @{

	/// Set smooth contribution factor (for smooth constraints resolution)
    ConstraintParams& setSmoothFactor(double v) { m_smoothFactor = v; return *this; }

    const ConstMultiVecCoordId& x() const { return m_x; }
    ConstMultiVecCoordId& x()       { return m_x; }

    const ConstMultiVecDerivId& v() const { return m_v; }
    ConstMultiVecDerivId& v()       { return m_v; }

    const ConstMultiMatrixDerivId&  j() const { return m_j; }
    ConstMultiMatrixDerivId& j()              { return m_j; }

    const MultiVecDerivId& dx() const { return m_dx;  }
    MultiVecDerivId&  dx()            { return m_dx;  }

    const MultiVecDerivId& lambda() const { return m_lambda; }
    MultiVecDerivId&  lambda()            { return m_lambda; }

    /// Set the IDs where to read the free position vector
    ConstraintParams& setX(                   ConstVecCoordId v) { m_x.assign(v);   return *this; }
    ConstraintParams& setX(                   ConstMultiVecCoordId v) { m_x = v;   return *this; }
    template<class StateSet>
    ConstraintParams& setX(const StateSet& g, ConstVecCoordId v) { m_x.setId(g, v); return *this; }

    /// Set the IDs where to read the free velocity vector
    ConstraintParams& setV(                   ConstVecDerivId v) { m_v.assign(v);   return *this; }
    ConstraintParams& setV(                   ConstMultiVecDerivId v) { m_v = v;   return *this; }
    template<class StateSet>
    ConstraintParams& setV(const StateSet& g, ConstVecDerivId v) { m_v.setId(g, v); return *this; }

    /// Set the IDs where to read the constraint jacobian matrix
    ConstraintParams& setJ(ConstMatrixDerivId j) { m_j.assign(j); return *this; }
    ConstraintParams& setJ(ConstMultiMatrixDerivId j) { m_j = j; return *this; }
    template<class StateSet>
    ConstraintParams& setJ(const StateSet& g, ConstMatrixDerivId j) { m_j.setId(g, j); return *this; }

    /// Set the IDs where to write corrective displacement vector
    ConstraintParams& setDx(VecDerivId dx)      { m_dx.assign(dx); return *this; }
    ConstraintParams& setDx(MultiVecDerivId dx) { m_dx = dx;   return *this; }
    template<class StateSet>
    ConstraintParams& setDx(const StateSet& g, MultiVecDerivId v) { m_dx.setId(g, dx); return *this; }

    /// Set the IDs where to write the constraint force vector
    ConstraintParams& setLambda(VecDerivId lambda) { m_lambda.assign(lambda); return *this; }
    ConstraintParams& setLambda(MultiVecDerivId lambda) { m_lambda = lambda;   return *this; }
    template<class StateSet>
    ConstraintParams& setLambda(const StateSet& g, MultiVecDerivId v) { m_lambda.setId(g, dx); return *this; }

    /// @}


    /// Constructor, initializing all VecIds to default values, implicit and energy flags to false
    ConstraintParams(const sofa::core::ExecParams& p = sofa::core::ExecParams())
        : sofa::core::ExecParams(p)
        , m_x(ConstVecCoordId::position())
        , m_v(ConstVecDerivId::velocity())
        , m_j(ConstMatrixDerivId::holonomicC())
        , m_dx(VecDerivId::dx())
        , m_lambda(VecDerivId::externalForce())
        , m_constOrder (POS_AND_VEL)
		, m_smoothFactor (1)
    {
    }

    /// Get the default MechanicalParams, to be used to provide a default values for method parameters
    static const ConstraintParams* defaultInstance();

    ConstraintParams& setExecParams(const core::ExecParams* params)
    {
        sofa::core::ExecParams::operator=(*params);
        return *this;
    }

protected:
    /// Ids of position vector
    ConstMultiVecCoordId m_x;

    /// Ids of velocity vector
    ConstMultiVecDerivId m_v;

    /// Ids of the constraint jacobian matrix
    ConstMultiMatrixDerivId m_j;

    /// Ids of contraint correction vector
    MultiVecDerivId      m_dx;

    /// Ids of constraint lambda vector
    MultiVecDerivId      m_lambda;

    /// Description of the order of the constraint
    ConstOrder m_constOrder;

	/// Smooth contribution factor (for smooth constraints resolution)
    double m_smoothFactor;
};

} // namespace core

} // namespace sofa

#endif // SOFA_CORE_CONSTRAINT_PARAMS_H
