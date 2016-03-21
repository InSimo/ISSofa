#ifndef SOFA_COMPONENT_CONSTRAINT_GYROSCOPICTVELOCITYIMPULSECONSTRAINT_INL
#define SOFA_COMPONENT_CONSTRAINT_GYROSCOPICTVELOCITYIMPULSECONSTRAINT_INL

#include "GyroscopicVelocityImpulseConstraint.h"

namespace sofa
{
namespace component
{
namespace constraint
{

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
/// Solve33 is from Box2d, credits Erin Catto.
template<class Real>
bool matrix33Solve(const sofa::defaulttype::Mat<3,3,Real>& A, const sofa::defaulttype::Vec<3,Real>& b, sofa::defaulttype::Vec<3,Real>& x, Real epsilon = Real(1e-6) )
{

    const sofa::defaulttype::Vec<3,Real> col0 = A.col(0);
	const sofa::defaulttype::Vec<3,Real> col1 = A.col(1);
	const sofa::defaulttype::Vec<3,Real> col2 = A.col(2);
		
    const Real det = sofa::defaulttype::dot(col0, sofa::defaulttype::cross(col1, col2));
	if (std::abs(det) > epsilon)
	{
		const Real invDet = Real(1.0) / det;
        x[0] = invDet * sofa::defaulttype::dot(b, sofa::defaulttype::cross(col1, col2));
        x[1] = invDet * sofa::defaulttype::dot(col0, sofa::defaulttype::cross(b, col2));
        x[2] = invDet * sofa::defaulttype::dot(col0, sofa::defaulttype::cross(col1, b));
        
        return true;
	}
    else
    {
        return false;
    }
}


/// skewSymmetricMatrix represent a cross product as a matrix multiplication
/// ie cross(a,b) = skewSymmetricMatrix(a)*b
/// it may be more convenient to use this formulation of cross product to compute derivatives.
template<class Real>
sofa::defaulttype::Mat<3, 3, Real> skewSymmetricMatrix(const sofa::defaulttype::Vec<3, Real>& v)
{
    sofa::defaulttype::Mat<3, 3, Real> res(sofa::defaulttype::NOINIT);
    res[0][0]=0;
    res[0][1]=-v[2];
    res[0][2]=v[1];
    res[1][0]=v[2];
    res[1][1]=0;
    res[1][2]=-v[0];
    res[2][0]=-v[1];
    res[2][1]=v[0];
    res[2][2]=0;
    return res;
}

/// returns an implicit computation of the gyroscopic force. 
/// see http://box2d.org/files/GDC2015/ErinCatto_NumericalMethods.pdf slide 71.
/// in an implicit setting the gyroscopic force is formulated as an implicit function of the angular velocity
/// we need to solve f( w ) = 0. Here we use only one step of newton raphson descent.  
/// with f( w ) =  Ibody.(w - w0) + h w x Ibody.w 
/// df/dw = Ibody + h.[ skew(w).Ibody - skew( Ibody.w )
/// which gives the gyroscopic force in the body frame.
/// - w0      is the angular velocity at the begining of the time step in body frame coordinates
/// - Ibody   is the inertia mass matrix in body coordinates.
/// - skew(.) is the skew symmetric matrix operator which turns a vector 
///           into its cross product matrix counter part
/// the gyroscopic force in then converted back to world coordinates.
template< typename TRigidCoord, typename TVec3, typename TRigidMass, typename Real  >
TVec3 computeGyroscopicAngularImpulseImplicit(const TRigidCoord& x, const TVec3& w, 
                                              const TRigidMass& mass, Real dt)
{
    const typename TRigidMass::Mat3x3& Ib = mass.inertiaMassMatrix;
    TVec3    wb         = x.getOrientation().inverseRotate( w );
    const TVec3   Iwb   = Ib * wb;
    const TVec3  wbxIwb = sofa::defaulttype::cross( wb, Iwb );
    const TVec3  f      = wbxIwb * dt; // Ib( wb - wb ) + wbxIwb * dt  

    const typename TRigidMass::Mat3x3 skew_wb  = skewSymmetricMatrix( wb  );
    const typename TRigidMass::Mat3x3 skew_Iwb = skewSymmetricMatrix( Iwb );

    const typename TRigidMass::Mat3x3  J = Ib + ( skew_wb * Ib - skew_Iwb ) * dt;
    typename TRigidMass::Mat3x3 Jinv(sofa::defaulttype::NOINIT);

    TVec3 gf;
    TVec3 w_step(sofa::defaulttype::NOINIT);

    if( matrix33Solve(J,f,w_step) )
    {
        wb -= w_step;

        const TVec3 w1 = x.getOrientation().rotate( wb );
        gf = w1 - w;
    }
    else
    {
        std::cerr<<__FUNCTION__<<std::endl;
    }

    return gf;
}

template< class RigidDataTypes >
GyroscopicVelocityImpulseConstraint<RigidDataTypes>::GyroscopicVelocityImpulseConstraint ()
:l_rigidMass(initLink("rigidMass","Path to the UniformRigidMass component"))
,d_active(initData(&d_active,true,"active","True if this component is active"))
,d_projectFreeMotion(initData(&d_projectFreeMotion,true,"projectFreeMotion","Apply the impulse at the end of the freeMotion step. \
                                                                             Otherwise applied after the constraint solving step."))
{
}

template< class RigidDataTypes >
void GyroscopicVelocityImpulseConstraint<RigidDataTypes>::init()
{
    Inherit1::init();

    if(!l_rigidMass)
    {
        l_rigidMass.setPath("@./");
        l_rigidMass.updateLinks();

        if(!l_rigidMass )
        {
            this->serr<<l_rigidMass.getName() <<" not found! "<< this->sendl;
            d_active.setValue(false);
        }
    }

}


template< class RigidDataTypes >
void GyroscopicVelocityImpulseConstraint<RigidDataTypes>::projectVelocity( const sofa::core::MechanicalParams* mparams,  DataVecDeriv& vData)
{
    bool isFreeMotion = (mparams->v().getId(this->getMState()) == sofa::core::VecDerivId::freeVelocity());
    bool active       = d_active.getValue(mparams);
    bool apply        = active && ( isFreeMotion == d_projectFreeMotion.getValue() );
    if(apply)
    {
        sofa::helper::WriteAccessor< DataVecDeriv > v( mparams, vData );
        sofa::helper::ReadAccessor< DataVecCoord  > x( mparams, mparams->readX( this->getMState() ) );
        sofa::helper::ReadAccessor< sofa::Data< sofa::helper::vector<TRigidMass> > > rigidMass(mparams,l_rigidMass->d_mass);

        for(std::size_t i=0;i<x.size();++i)
        {
            const TRigidMass& mass = i<rigidMass.size() ? rigidMass[i] : rigidMass[0];
            v[i].getAngular() += computeGyroscopicAngularImpulseImplicit( x[i], v[i].getAngular(), mass, Real(this->getContext()->getDt()) );
        }
    }
}


}

}

}

#endif // SOFA_COMPONENT_CONSTRAINT_GYROSCOPICTVELOCITYIMPULSECONSTRAINT_INL
