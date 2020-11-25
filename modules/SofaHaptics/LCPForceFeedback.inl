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
#ifndef SOFA_COMPONENT_CONTROLLER_LCPFORCEFEEDBACK_INL
#define SOFA_COMPONENT_CONTROLLER_LCPFORCEFEEDBACK_INL

#include <SofaHaptics/LCPForceFeedback.h>

#include <SofaConstraint/ConstraintSolverImpl.h>

#include <sofa/simulation/common/AnimateEndEvent.h>

#include <algorithm>

namespace sofa
{

namespace component
{

namespace controller
{

template <class DataTypes>
LCPForceFeedback<DataTypes>::LCPForceFeedback()
    : forceCoef(initData(&forceCoef, 0.03, "forceCoef","multiply haptic force by this coef."))
    , solverTimeout(initData(&solverTimeout, 0.0008, "solverTimeout","max time to spend solving constraints."))
    , d_derivRotations(initData(&d_derivRotations, false, "derivRotations", "if true, deriv the rotations when updating the violations"))
    , d_localHapticConstraintAllFrames(initData(&d_localHapticConstraintAllFrames, false, "localHapticConstraintAllFrames", "Flag to enable/disable constraint haptic influence from all frames"))
    , localHapticConstraintRigidBilateral(initData(&localHapticConstraintRigidBilateral, true, "localHapticConstraintRigidBilateral", "Flag to enable/disable haptic on rigid/rigid bilateral constraints created in grasp actions"))
    , l_rigidRigidConstraints(initLink("RigidRigidConstraints","Stores active rigid/rigid constraints corresponding to linked grasped objects"))
    , mState(NULL)
    , mNextBufferId(0)
    , mCurBufferId(0)
    , mIsCuBufferInUse(false)
    , constraintSolver(NULL)
    , _timer(NULL)
    , time_buf(0)
    , timer_iterations(0)
    , haptic_freq(0.0)
    , num_constraints(0)
{
    localHapticConstraintRigidBilateral.setGroup("LocalHapticConstraints");

    this->f_listening.setValue(true);
    mCP[0] = NULL;
    mCP[1] = NULL;
    mCP[2] = NULL;
    _timer = new helper::system::thread::CTime();
    time_buf = _timer->getTime();
    timer_iterations = 0;
}


template <class DataTypes>
void LCPForceFeedback<DataTypes>::init()
{
    core::objectmodel::BaseContext* c = this->getContext();

    this->ForceFeedback::init();
    if(!c)
    {
        serr << "LCPForceFeedback has no current context. Initialisation failed." << sendl;
        return;
    }

    c->get(constraintSolver);

    if (!constraintSolver)
    {
        serr << "LCPForceFeedback has no binding ConstraintSolver. Initialisation failed." << sendl;
        return;
    }

    mState = core::behavior::MechanicalState<DataTypes>::DynamicCast(c->getMechanicalState());
    if (!mState)
    {
        serr << "LCPForceFeedback has no binding MechanicalState. Initialisation failed." << sendl;
        return;
    }
}


template <class DataTypes>
void LCPForceFeedback<DataTypes>::computeForce(const VecCoord& state,  VecDeriv& forces)
{
    if (!this->f_activate.getValue())
    {
        return;
    }
    updateStats();
    updateConstraintProblem();
    doComputeForce(state, forces);
}

template <class DataTypes>
void LCPForceFeedback<DataTypes>::updateStats()
{
    using namespace helper::system::thread;

    ctime_t actualTime = _timer->getTime();
    ++timer_iterations;
    if (actualTime - time_buf >= CTime::getTicksPerSec())
    {
        haptic_freq = (double)(timer_iterations*CTime::getTicksPerSec())/ (double)( actualTime - time_buf) ;
        time_buf = actualTime;
        timer_iterations = 0;
    }
}

template <class DataTypes>
bool LCPForceFeedback<DataTypes>::updateConstraintProblem()
{
    int prevId = mCurBufferId;

    //
    // Retrieve the last LCP and constraints computed by the Sofa thread.
    //
    mIsCuBufferInUse = true;

    {
        // TODO: Lock and/or memory barrier HERE
        mCurBufferId = mNextBufferId;
    }

    bool changed = (prevId != mCurBufferId);

    component::constraintset::ConstraintProblem* cp = mCP[mCurBufferId];

    if(!cp)
    {
        mIsCuBufferInUse = false;
    }

    return changed;
}

template <class DataTypes>
void LCPForceFeedback<DataTypes>::doComputeForce(const VecCoord& state,  VecDeriv& forces)
{
    const int stateSize = int(state.size());
    forces.resize(stateSize);
    for (unsigned int i = 0; i < forces.size(); ++i)
    {
        forces[i].clear();
    }

    if(!constraintSolver||!mState)
        return;

    const MatrixDeriv& constraints = mConstraints[mCurBufferId];
    VecCoord &val = mVal[mCurBufferId];
    component::constraintset::ConstraintProblem* cp = mCP[mCurBufferId];

    const std::set<int>& constraintToSkip = mConstraintToSkip[mCurBufferId];

    if(!cp)
    {
        return;
    }

    std::vector<double> localF;
    localF.resize(cp->getDimension());

    const double forceScale = forceCoef.getValue();

    if(!constraints.empty() &&
       constraints.size() != constraintToSkip.size() )
    {
        VecDeriv dx;

        std::fill(localF.begin(), localF.end(), 0);

        derivVectors< DataTypes >(dx, val, state, d_derivRotations.getValue());

        const bool localHapticConstraintAllFrames = d_localHapticConstraintAllFrames.getValue();

        // Modify Dfree
        MatrixDerivRowConstIterator rowItEnd = constraints.end();
        num_constraints = constraints.size();

        for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
        {
            MatrixDerivColConstIterator colItEnd = rowIt.end();

            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
            {
                cp->getDfree()[rowIt.index()] += computeDot(colIt.val(), dx[localHapticConstraintAllFrames ? 0 : colIt.index()]);
            }
        }

        // Solving constraints
        cp->solveTimed(cp->tolerance * 0.001, 100, solverTimeout.getValue());	// tol, maxIt, timeout

        if (!constraintToSkip.empty())
        {
            auto it = constraintToSkip.begin();

            while (it != constraintToSkip.end())
            {
                int cIndex = *it;
                auto* cr = cp->constraintsResolutions[cIndex];
                if (cr == nullptr)
                {
                    serr << "constraintsResolutions[" << cIndex << "] == nullptr ! Dimension: " << cp->getDimension() << sendl;
                    break;
                }
                else
                {
                    // get the local force for this constraint resolution, 
                    // neglecting the coupling with other constraints
                    for (unsigned i = 0; i < cr->getNbLines(); ++i)
                    {
                        cp->_d[cIndex + i] = cp->getDfree()[cIndex + i];
                    }
                    cr->resolution(cIndex, cp->getW(), cp->_d.ptr(), &localF[0], cp->getDfree());

                    cIndex += cr->getNbLines();
                    std::advance(it, cr->getNbLines());
                }
            }
        }


        // Restore Dfree
        for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
        {
            MatrixDerivColConstIterator colItEnd = rowIt.end();

            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
            {
                cp->getDfree()[rowIt.index()] -= computeDot(colIt.val(), dx[localHapticConstraintAllFrames ? 0 : colIt.index()]);
            }
        }

        for (MatrixDerivRowConstIterator rowIt = constraints.begin(); rowIt != rowItEnd; ++rowIt)
        {
            if (cp->getF()[rowIt.index()] != 0.0)
            {
                MatrixDerivColConstIterator colItEnd = rowIt.end();
                cp->getF()[rowIt.index()] -= localF[rowIt.index()];

                for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
                {
                    const Deriv f   = colIt.val() * cp->getF()[rowIt.index()] * forceScale;
                    const int index = localHapticConstraintAllFrames ? 0 : colIt.index();
                    if (index < stateSize)
                    {
                        forces[index] += f;
                    }
                }
            }
        }

    }
    mIsCuBufferInUse = false;
}


template <typename DataTypes>
void LCPForceFeedback<DataTypes>::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (!sofa::simulation::AnimateEndEvent::DynamicCast(event))
        return;

    if (!constraintSolver)
        return;

    if (!mState)
        return;

    component::constraintset::ConstraintProblem* new_cp = constraintSolver->getConstraintProblem();
    
    if (!new_cp)
        return;

    // Find available buffer

    unsigned char buf_index=0;
    unsigned char cbuf_index=mCurBufferId;
    unsigned char nbuf_index=mNextBufferId;

    if (buf_index == cbuf_index || buf_index == nbuf_index)
        buf_index++;

    if (buf_index == cbuf_index || buf_index == nbuf_index)
        buf_index++;

    // Compute constraints, id_buf lcp and val for the current lcp.


    //	std::vector<int>& id_buf = mId_buf[buf_index];
    VecCoord& val = mVal[buf_index];

    // Update LCP
    component::constraintset::ConstraintProblem*& cp = mCP[buf_index];
    cp = new_cp;
    // Update Val
    val = mState->read(sofa::core::VecCoordId::freePosition())->getValue();


    //	id_buf.clear();

    const bool skipRigidBilateral = !localHapticConstraintRigidBilateral.getValue();
    
    const MatrixDeriv& c            = mState->read(core::ConstMatrixDerivId::constraintJacobian())->getValue()   ;
    MatrixDeriv& constraints        = mConstraints[buf_index];
    MatrixDeriv& localConstraints   = mLocalConstraints[buf_index];
    std::set<int>& constraintToSkip = mConstraintToSkip[buf_index];

    constraints = c;
    constraintToSkip.clear();
    
    if (skipRigidBilateral)
    {
        localConstraints.clear();
        for (unsigned int rrci = 0; rrci < this->l_rigidRigidConstraints.size(); ++rrci)
        {
            core::behavior::MechanicalState<DataTypes> *toolMappedState = this->l_rigidRigidConstraints[rrci]->getMState2();
            const MatrixDeriv& c = toolMappedState->read(core::ConstMatrixDerivId::holonomicC())->getValue();

            for (MatrixDerivRowConstIterator rowIt = c.begin(), rowItEnd = c.end(); rowIt != rowItEnd; ++rowIt)
            {
                constraintToSkip.insert(rowIt.index());
            }
        }

        for (MatrixDerivRowConstIterator row = constraints.begin(); row != constraints.end(); ++row)
        {
            const int cIndex = row.index();
            if (constraintToSkip.count(cIndex) > 0)
            {
                continue;
            }

            localConstraints.addLine(cIndex, row.row());
        }
        
        localConstraints.compress();
    }
    else
    {
        localConstraints = c;
    }

    
    // valid buffer
    {
        // TODO: Lock and/or memory barrier HERE
        mNextBufferId = buf_index;
    }

    // Lock lcp to prevent its use by the SOFA thread while it is used by haptic thread
    if(mIsCuBufferInUse)
        constraintSolver->lockConstraintProblem(this, mCP[mCurBufferId], mCP[mNextBufferId]);
    else
        constraintSolver->lockConstraintProblem(this, mCP[mNextBufferId]);
}

template <typename DataTypes>
typename LCPForceFeedback<DataTypes>::VecCoord* LCPForceFeedback<DataTypes>::getmVal()
{
    return &mVal[0];
}

template <typename DataTypes>
typename LCPForceFeedback<DataTypes>::MatrixDeriv* LCPForceFeedback<DataTypes>::getmConstraints()
{
    return &mConstraints[0];
}

template <typename DataTypes>
component::constraintset::ConstraintProblem** LCPForceFeedback<DataTypes>::getmCP()
{
    return &mCP[0];
}

//
// Those functions are here for compatibility with the sofa::component::controller::Forcefeedback scheme
//

template <typename DataTypes>
void LCPForceFeedback<DataTypes>::computeForce(SReal , SReal, SReal, SReal, SReal, SReal, SReal, SReal&, SReal&, SReal&)
{

}


template <typename DataTypes>
void LCPForceFeedback<DataTypes>::computeWrench(const sofa::defaulttype::SolidTypes<SReal>::Transform &,
        const sofa::defaulttype::SolidTypes<SReal>::SpatialVector &,
        sofa::defaulttype::SolidTypes<SReal>::SpatialVector & )
{

}


template <typename DataTypes>
bool LCPForceFeedback<DataTypes>::registerRigidRigidBilateralConstraint(typename core::behavior::PairInteractionConstraint<DataTypes>::SPtr c)
{
    for (unsigned int ci = 0; ci < l_rigidRigidConstraints.size(); ++ci)
    {
        if (l_rigidRigidConstraints[ci] == c)
        {
            return false;
        }
    }

    l_rigidRigidConstraints.add(c);

    return true;
}

template <typename DataTypes>
bool LCPForceFeedback<DataTypes>::unregisterRigidRigidBilateralConstraint(typename core::behavior::PairInteractionConstraint<DataTypes>::SPtr c)
{
    for (unsigned int ci = 0; ci < l_rigidRigidConstraints.size(); ++ci)
    {
        if (l_rigidRigidConstraints[ci] == c)
        {
            l_rigidRigidConstraints.remove(c);
            return true;
        }
    }

    return false;
}


#ifndef SOFA_DOUBLE

template <>
void SOFA_HAPTICS_API LCPForceFeedback< sofa::defaulttype::Rigid3fTypes >::computeForce(SReal x, SReal y, SReal z, SReal, SReal, SReal, SReal, SReal& fx, SReal& fy, SReal& fz);

#endif // SOFA_DOUBLE

#ifndef SOFA_FLOAT

template <>
void SOFA_HAPTICS_API LCPForceFeedback< sofa::defaulttype::Rigid3dTypes >::computeForce(double x, double y, double z, double, double, double, double, double& fx, double& fy, double& fz);

template <>
void SOFA_HAPTICS_API LCPForceFeedback< sofa::defaulttype::Rigid3dTypes >::computeWrench(const sofa::defaulttype::SolidTypes<double>::Transform &world_H_tool,
        const sofa::defaulttype::SolidTypes<double>::SpatialVector &/*V_tool_world*/,
        sofa::defaulttype::SolidTypes<double>::SpatialVector &W_tool_world );

#endif // SOFA_FLOAT


} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_LCPFORCEFEEDBACK_INL
