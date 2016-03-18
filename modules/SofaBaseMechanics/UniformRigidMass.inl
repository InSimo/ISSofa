#ifndef SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_INL
#define SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_INL

#include "UniformRigidMass.h"
#include <SofaBaseLinearSolver/BlocMatrixWriter.h>
#include <sofa/helper/gl/Axis.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/template.h>

namespace sofa
{
namespace component
{
namespace mass
{

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

/// Convert to global coordinates the local matrix using the given orientation quaternion.
/// local is a diagonal matrix ( either the local inertia mass matrix, or its inverse )
template< typename TQuat , typename TReal >
sofa::defaulttype::Mat<3,3,TReal> localToGlobal(const TQuat& orientation, const sofa::defaulttype::Mat<3,3,TReal>& local )
{
    sofa::defaulttype::Mat<3,3,TReal> rotation(sofa::defaulttype::NOINIT);
    orientation.toMatrix(rotation);

    const typename sofa::defaulttype::Mat<3,3,TReal>::Line diag = sofa::defaulttype::diagonal(local);
    sofa::defaulttype::Mat<3,3,TReal> global = rotation.multDiagonal( diag ) * rotation.transposed();

    return global;
}

template< typename TRigidCoord, typename TRigidMass >
typename TRigidMass::Mat3x3 computeInertiaMassMatrixWorld( const TRigidCoord& x, const TRigidMass& mass)
{
    const typename TRigidMass::Mat3x3 inertiaMassMatrixLocal = mass.inertiaMatrix * mass.mass;
    return localToGlobal( x.getOrientation(), inertiaMassMatrixLocal );
}

template< typename TRigidCoord, typename TRigidMass >
typename TRigidMass::Mat3x3 computeInvInertiaMassMatrixWorld( const TRigidCoord& x, const TRigidMass& mass)
{
    const typename TRigidMass::Mat3x3 invInertiaMassMatrixLocal = mass.invInertiaMatrix * mass.mass;
    return localToGlobal( x.getOrientation(), invInertiaMassMatrixLocal );
}


/// returns an explicit computation of the gyroscopic force.
/// in an explicit setting the gyroscopic force is gf = w x I.w  where : 
/// - w is the angular velocity
/// - I is the inertia mass matrix in world coordinates
template< typename TRigidCoord, typename TVec3, typename TRigidMass >
TVec3 computeGyroscopicForceExplicit(const TRigidCoord& x, 
                                     const TVec3&       w, 
                                     const TRigidMass&  mass )
{
    const typename TRigidMass::Mat3x3  intertiaMassMatrixWorld = computeInertiaMassMatrixWorld(x,mass);
    const TVec3 Iw   = intertiaMassMatrixWorld * w;
    const TVec3 wxIw = sofa::defaulttype::cross( w, Iw );
    return wxIw;
}


/// returns an implicit computation of the gyroscopic force. 
/// see http://box2d.org/files/GDC2015/ErinCatto_NumericalMethods.pdf slide 71.
/// in an implicit setting the gyroscopic force is formulated as an implicit function of the angular velocity
/// we need to solve f( w ) = 0, using one step of newton raphson descent.  
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

    if( sofa::defaulttype::invertMatrix(Jinv, J ) )
    {
        wb -= Jinv*f;

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
UniformRigidMass<RigidDataTypes>::UniformRigidMass()
:d_mass(initData(&d_mass,sofa::helper::vector<TRigidMass>(1,TRigidMass() ),"mass","The uniform mass for each dof that compose the rigid object"))
,d_useGyroscopicExplicit(initData(&d_useGyroscopicExplicit,false,"useGyroscopicExplicit","Specify if the gyroscopic term should be computed using explicit formulation"))
,d_useGyroscopicImplicit(initData(&d_useGyroscopicImplicit,false,"useGyroscopicImplicit","Specify if the gyroscopic term should be computed using implicit formulation"))
,d_maxGyroscopicForce(initData(&d_maxGyroscopicForce,Real(0.0),"maxGyroscopicForce","Used only for explicit computation of gyroscopic term. Clamp value that must be specified, since explicit computation diverges"))
,d_drawAxisFactor(initData(&d_drawAxisFactor,1.0f,"drawAxisFactor","Draw: the factor applied on the size of the axis"))
{
}

template< class RigidDataTypes >
void UniformRigidMass<RigidDataTypes>::init()
{
    Inherit1::init();
    Inherit2::init();
    reinit();
}

template< class RigidDataTypes >
void UniformRigidMass<RigidDataTypes>::reinit()
{
    Inherit1::reinit();
    Inherit2::reinit();

    sofa::helper::WriteAccessor< sofa::Data< sofa::helper::vector< TRigidMass > > > rigidMass = d_mass;

    for(std::size_t i=0;i<rigidMass.size();++i)
    {
        rigidMass[i].recalc();
    }

    if( d_useGyroscopicImplicit.getValue() && d_useGyroscopicExplicit.getValue() )
    {
        serr<<"Cannot set both implicit and explicit gyroscopic term. Force implicit computation only" << sendl;
        d_useGyroscopicExplicit.setValue(false);
    }

    this->sout << "mass= " << d_mass.getValue() << this->sendl;

}

template< class RigidDataTypes >
void UniformRigidMass<RigidDataTypes>::addForce( const sofa::core::MechanicalParams* mparams, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v )
{
    sofa::helper::ReadAccessor< DataVecCoord > x(mparams,d_x);
    sofa::helper::ReadAccessor< DataVecDeriv > v(mparams,d_v);
    sofa::helper::WriteAccessor< DataVecDeriv> f(mparams,d_f);

    f.resize( x.size() );

    sofa::helper::ReadAccessor< sofa::Data< sofa::helper::vector< TRigidMass > > > rigidMass(mparams,d_mass);

    for(std::size_t i=0;i<f.size();++i)
    {
        const TRigidMass& m = i < rigidMass.size() ? rigidMass[i] : rigidMass[0];
        typename DataTypes::DPos mg = this->getContext()->getGravity()*rigidMass[i].mass;
        f[i].getLinear() += mg;
    }

    if(d_useGyroscopicExplicit.getValue(mparams) )
    {
        for(std::size_t i=0;i<f.size(); ++i)
        {
            const TRigidMass& m = i < rigidMass.size() ? rigidMass[i] : rigidMass[0];
            typename RigidDeriv::Vec3 gyroscopicForce = computeGyroscopicForceExplicit(x[i],v[i].getAngular(),m );
            const Real  maxGyroNorm  = d_maxGyroscopicForce.getValue(mparams);
            const Real  maxGyroNorm2 = maxGyroNorm*maxGyroNorm  ;
            const Real  gyroNorm     = gyroscopicForce.norm();
            const Real  gyroNorm2    = gyroNorm*gyroNorm;
            if( gyroNorm2 > maxGyroNorm2 )
            {
                Real factor = maxGyroNorm / gyroNorm;
                gyroscopicForce *= factor;
            }

            f[i].getAngular() -= gyroscopicForce;
        }
    }
}

template< class RigidDataTypes >
void UniformRigidMass<RigidDataTypes>::projectVelocity( const sofa::core::MechanicalParams* mparams,  DataVecDeriv& vData)
{
    if( d_useGyroscopicImplicit.getValue(mparams) )
    {
        bool isFreeMotion = (mparams->v().getId(getMState()) == sofa::core::VecDerivId::freeVelocity());
        sofa::helper::ReadAccessor< sofa::Data< sofa::helper::vector< TRigidMass > > > rigidMass(mparams,d_mass);

        if(!isFreeMotion)
        {
            sofa::helper::WriteAccessor< DataVecDeriv > v( mparams, vData );
            sofa::helper::ReadAccessor< DataVecCoord  > x(mparams, mparams->readX( getMState() ) );
            
            for(std::size_t i=0;i<x.size();++i)
            {
                const TRigidMass& mass = i<rigidMass.size() ? rigidMass[i] : rigidMass[0];
                v[i].getAngular() += computeGyroscopicAngularImpulseImplicit( x[i], v[i].getAngular(), mass, Real( this->getContext()->getDt() ) );
            }
        }
    }

}

template< class RigidDataTypes >
void UniformRigidMass<RigidDataTypes>::addMDx( const sofa::core::MechanicalParams* mparams, DataVecDeriv& d_df, const DataVecDeriv& d_dx, double factor )
{
    if( factor == 0 )
    {
        return;
    }

    sofa::helper::ReadAccessor< DataVecDeriv > dx(mparams,d_dx);
    sofa::helper::WriteAccessor< DataVecDeriv> df(mparams,d_df);
    df.resize( dx.size() );
    sofa::helper::ReadAccessor< DataVecCoord > x( mparams, mparams->readX( getMState()  ) );
    sofa::helper::ReadAccessor< sofa::Data< sofa::helper::vector< TRigidMass > > > rigidMass(mparams,d_mass);
    const bool verbose = this->f_printLog.getValue();

    for(std::size_t i=0;i<df.size();++i)
    {
        const TRigidMass& mass = i < rigidMass.size() ? rigidMass[i] : rigidMass[0];
        df[i].getLinear() +=  dx[i].getLinear() * mass.mass * factor ;

        typename TRigidMass::Mat3x3 rotation;
        x[i].getOrientation().toMatrix(rotation);

        typename TRigidMass::Mat3x3 inertiaMassMatrixWorld  = computeInertiaMassMatrixWorld(x[i], mass );
        inertiaMassMatrixWorld *= (Real)factor;
        df[i].getAngular() += inertiaMassMatrixWorld * dx[i].getAngular();
    }
}

template< class RigidDataTypes >
void UniformRigidMass<RigidDataTypes>::addMToMatrix(const core::MechanicalParams *mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    //sofa::component::linearsolver::BlocMatrixWriter<MatBloc> writer;
    //writer.addMToMatrix(this, mparams, matrix->getMatrix(this->mstate));

    sofa::core::behavior::MultiMatrixAccessor::MatrixRef r = matrix->getMatrix( getMState() );
    const unsigned size = sofa::defaulttype::DataTypeInfo<typename DataTypes::Deriv>::size();

    sofa::helper::ReadAccessor< DataVecCoord > x( mparams, mparams->readX( getMState()  ) );
    sofa::helper::ReadAccessor< sofa::Data< sofa::helper::vector< TRigidMass > > > rigidMass(mparams,d_mass);

    Real mFactor = Real( mparams->mFactor() );

    for(std::size_t i=0; i< (std::size_t)getMState()->getSize(); ++i)
    {
        TRigidMass mass = i < rigidMass.size() ? rigidMass[i] : rigidMass[0];
        mass *= mFactor;
        // translation
        for(unsigned j = 0; j < 3; ++j) 
        {
            r.matrix->add(r.offset + size * i + j,
                          r.offset + size * i + j,
                          mass.mass);
        }

        typename TRigidMass::Mat3x3 inertiaMassMatrixWorld = computeInertiaMassMatrixWorld(x[i], mass );

        // rotation
        for(unsigned j = 0; j < 3; ++j) 
        {
            for(unsigned k = 0; k < 3; ++k) 
            {
                r.matrix->add(r.offset + size * i + 3 + j,
                              r.offset + size * i + 3 + k,
                              inertiaMassMatrixWorld[j][k]  );
            }
        }
    }
}

//template<class MatrixWriter>
//template< class RigidDataTypes >
//void UniformRigidMass<RigidDataTypes>::addMToMatrixT(const core::MechanicalParams* mparams, MatrixWriter mwriter)
//{
//
//}

template< class RigidDataTypes >
void UniformRigidMass<RigidDataTypes>::draw(const sofa::core::visual::VisualParams* vparams)
{
   if (!vparams->displayFlags().getShowBehaviorModels())
   {
        return;
   }
    sofa::helper::ReadAccessor< DataVecCoord > x = getMState()->readPositions();
    sofa::helper::ReadAccessor< sofa::Data< sofa::helper::vector< TRigidMass > > > rigidMass(vparams,d_mass);

    typename RigidDataTypes::CPos gravityCenter;
    typename RigidDataTypes::CPos len;

    // The moment of inertia of a box is:
    //   m->_I(0,0) = M/REAL(12.0) * (ly*ly + lz*lz);
    //   m->_I(1,1) = M/REAL(12.0) * (lx*lx + lz*lz);
    //   m->_I(2,2) = M/REAL(12.0) * (lx*lx + ly*ly);
    // So to get lx,ly,lz back we need to do
    //   lx = sqrt(12/M * (m->_I(1,1)+m->_I(2,2)-m->_I(0,0)))
    // Note that RigidMass inertiaMatrix is already divided by M


    for (unsigned int i=0; i< x.size(); ++i)
    {
        const TRigidMass& mass = i<rigidMass.size() ? rigidMass[i] : rigidMass[0];
        Real m00 = mass.inertiaMatrix[0][0];
        Real m11 = mass.inertiaMatrix[1][1];
        Real m22 = mass.inertiaMatrix[2][2];
        len[0] = sqrt(m11+m22-m00);
        len[1] = sqrt(m00+m22-m11);
        len[2] = sqrt(m00+m11-m22);
        vparams->drawTool()->drawFrame(x[i].getCenter(), x[i].getOrientation(), len*d_drawAxisFactor.getValue() );
        gravityCenter += (x[i].getCenter());
    }

    gravityCenter /= x.size();
    glColor3f (1,1,0);
    glBegin (GL_LINES);
    helper::gl::glVertexT(gravityCenter - typename RigidDataTypes::CPos(d_drawAxisFactor.getValue(),0,0) );
    helper::gl::glVertexT(gravityCenter + typename RigidDataTypes::CPos(d_drawAxisFactor.getValue(),0,0) );
    helper::gl::glVertexT(gravityCenter - typename RigidDataTypes::CPos(0,d_drawAxisFactor.getValue(),0) );
    helper::gl::glVertexT(gravityCenter + typename RigidDataTypes::CPos(0,d_drawAxisFactor.getValue(),0) );
    helper::gl::glVertexT(gravityCenter - typename RigidDataTypes::CPos(0,0,d_drawAxisFactor.getValue()) );
    helper::gl::glVertexT(gravityCenter + typename RigidDataTypes::CPos(0,0,d_drawAxisFactor.getValue()) );
    glEnd();
    
}


}

}

}

#endif // SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_INL
