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
#ifndef SOFA_COMPONENT_CONTROLLER_LCPFORCEFEEDBACK_H
#define SOFA_COMPONENT_CONTROLLER_LCPFORCEFEEDBACK_H

#include <SofaHaptics/MechanicalStateForceFeedback.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/PairInteractionConstraint.h>

#include <sofa/helper/system/thread/CTime.h>

#include <sofa/SofaGeneral.h>

namespace sofa
{

namespace component
{

namespace constraintset { class ConstraintProblem; class ConstraintSolverImpl; }


namespace controller
{

namespace
{

template <int N, typename real>
void deriv2Coord(sofa::defaulttype::Vec<N, real>& d, const sofa::defaulttype::Vec<N, real>& x0, const sofa::defaulttype::Vec<N, real>& x1, bool /*derivRotation*/)
{
    d = x1 - x0;
}

template <int N, typename real>
void deriv1Coord(sofa::defaulttype::Vec<N, real>& d, const sofa::defaulttype::Vec<N, real>& x0, bool /*derivRotation*/)
{
    d = - x0;
}

template <typename real>
void deriv2Coord(sofa::defaulttype::RigidDeriv<3, real>& d, const sofa::defaulttype::RigidCoord<3, real>& x0, const sofa::defaulttype::RigidCoord<3, real>& x1, bool derivRotation)
{
    getVCenter(d) = x1.getCenter() - x0.getCenter();
    if (derivRotation)
    {
        // rotations are taken into account to compute the violations
        sofa::defaulttype::Quat q;
        getVOrientation(d) = x0.rotate(q.angularDisplacement(x1.getOrientation(), x0.getOrientation(), true ) );
    }
    else
        getVOrientation(d) *= 0;
}

template <typename real>
void deriv1Coord(sofa::defaulttype::RigidDeriv<3, real>& d, const sofa::defaulttype::RigidCoord<3, real>& x0, bool derivRotation)
{
    getVCenter(d) = - x0.getCenter();

    if (derivRotation)
    {
        // rotations are taken into account to compute the violations
        sofa::defaulttype::Quat q= x0.getOrientation();
        getVOrientation(d) = -x0.rotate( q.getLog(true) );
    }
    else
        getVOrientation(d) *= 0;
}

template <typename DataTypes>
void derivVectors(typename DataTypes::VecDeriv& d, const typename DataTypes::VecCoord& x0, const typename DataTypes::VecCoord& x1, bool derivRotation)
{
    unsigned int sz0 = x0.size();
    unsigned int szmin = std::min(sz0,(unsigned int)x1.size());

    d.resize(sz0);
    for(unsigned int i=0; i<szmin; ++i)
    {
        deriv2Coord(d[i], x0[i], x1[i], derivRotation);
    }
    for(unsigned int i=szmin; i<sz0; ++i) // not sure in what case this is applicable...
    {
        deriv1Coord(d[i], x0[i], derivRotation);
    }
}

template <int N, typename real>
real computeDot(const sofa::defaulttype::Vec<N, real>& v0, const sofa::defaulttype::Vec<N, real>& v1)
{
    return dot(v0,v1);
}

template <typename real>
real computeDot(const sofa::defaulttype::RigidDeriv<3, real>& v0, const sofa::defaulttype::RigidDeriv<3, real>& v1)
{
    return dot(getVCenter(v0),getVCenter(v1)) + dot(getVOrientation(v0), getVOrientation(v1));
}

} // anonymous namespace

/**
* LCP force field
*/
template <class TDataTypes>
class LCPForceFeedback : public sofa::component::controller::MechanicalStateForceFeedback<TDataTypes>
{
public:

    SOFA_CLASS(SOFA_TEMPLATE(LCPForceFeedback,TDataTypes),sofa::component::controller::MechanicalStateForceFeedback<TDataTypes>);

    typedef TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
    typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;
    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;


    void init();

    void draw()
    {
        // draw the haptic_freq in the openGL window

        //std::cout << "num_constraints = " << std::fixed << num_constraints << " " << '\xd';

        if (this->f_printLog.getValue()) std::cout << "haptic_freq = " << std::fixed << haptic_freq << " Hz   " << '\xd';
    }

    Data< double > forceCoef;
    //Data< double > momentCoef;

    Data< double > solverTimeout;

    // deriv (or not) the rotations when updating the violations 
    Data <bool> d_derivRotations; 

    // Enable/disable constraint haptic influence from all frames
    Data< bool > d_localHapticConstraintAllFrames;
    Data< bool > localHapticConstraintRigidBilateral;


    sofa::MultiLink< LCPForceFeedback<DataTypes>, core::behavior::PairInteractionConstraint<DataTypes>, sofa::BaseLink::FLAG_STRONGLINK > l_rigidRigidConstraints;
    bool registerRigidRigidBilateralConstraint(typename core::behavior::PairInteractionConstraint<DataTypes>::SPtr c);
    bool unregisterRigidRigidBilateralConstraint(typename core::behavior::PairInteractionConstraint<DataTypes>::SPtr c);

    virtual void computeForce(SReal x, SReal y, SReal z, SReal u, SReal v, SReal w, SReal q, SReal& fx, SReal& fy, SReal& fz);
    virtual void computeWrench(const sofa::defaulttype::SolidTypes<SReal>::Transform &world_H_tool, const sofa::defaulttype::SolidTypes<SReal>::SpatialVector &V_tool_world, sofa::defaulttype::SolidTypes<SReal>::SpatialVector &W_tool_world );
    virtual void computeForce(const  VecCoord& state,  VecDeriv& forces);

    //void computeForce(double pitch0, double yaw0, double roll0, double z0, double pitch1, double yaw1, double roll1, double z1, double& fpitch0, double& fyaw0, double& froll0, double& fz0, double& fpitch1, double& fyaw1, double& froll1, double& fz1);
protected:
    LCPForceFeedback();
    virtual ~LCPForceFeedback()
    {
        delete(_timer);
    }

    virtual void updateStats();
    virtual bool updateConstraintProblem();
    virtual void doComputeForce(const  VecCoord& state,  VecDeriv& forces);


public:
    void handleEvent(sofa::core::objectmodel::Event *event);


    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        if (core::behavior::MechanicalState<DataTypes>::DynamicCast(context->getMechanicalState()) == NULL)
            return false;
        return core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const LCPForceFeedback<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

    /*Deriv m_dxRotation;
    Coord m_valRotation;*/

    VecCoord* getmVal();
    MatrixDeriv* getmConstraints();
    constraintset::ConstraintProblem **getmCP();

protected:
    //component::constraintset::LCP* lcp, *next_lcp;
    core::behavior::MechanicalState<DataTypes> *mState; ///< The device try to follow this mechanical state.
    VecCoord mVal[3];
    MatrixDeriv mConstraints[3];
    MatrixDeriv mLocalConstraints[3];
    std::set<int> mConstraintToSkip[3];
    std::vector<int> mId_buf[3];
    component::constraintset::ConstraintProblem* mCP[3];
    /* 	std::vector<int> *id_buf; */
    /* 	typename DataType::VecCoord *val; */
    unsigned char mNextBufferId; // Next buffer id to be use
    unsigned char mCurBufferId; // Current buffer id in use
    bool mIsCuBufferInUse; // Is current buffer currently in use right now

    //core::behavior::MechanicalState<defaulttype::Vec1dTypes> *mState1d; ///< The device try to follow this mechanical state.
    sofa::component::constraintset::ConstraintSolverImpl* constraintSolver;
    // timer: verifies the time rates of the haptic loop
    helper::system::thread::CTime *_timer;
    helper::system::thread::ctime_t time_buf;
    int timer_iterations;
    double haptic_freq;
    unsigned int num_constraints;
};


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_CONTROLLER_LCPFORCEFEEDBACK_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_HAPTICS_API LCPForceFeedback<defaulttype::Vec1dTypes>;
extern template class SOFA_HAPTICS_API LCPForceFeedback<defaulttype::Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_HAPTICS_API LCPForceFeedback<defaulttype::Vec1fTypes>;
extern template class SOFA_HAPTICS_API LCPForceFeedback<defaulttype::Rigid3fTypes>;
#endif
#endif

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_LCPFORCEFEEDBACK_H
