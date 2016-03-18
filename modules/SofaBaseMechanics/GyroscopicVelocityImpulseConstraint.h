#ifndef SOFA_COMPONENT_CONSTRAINT_GYROSCOPICTVELOCITYIMPULSECONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINT_GYROSCOPICTVELOCITYIMPULSECONSTRAINT_H

#include <sofa/core/behavior/Mass.h>
#include <sofa/core/behavior/ProjectiveConstraintSet.h>
#include <sofa/defaulttype/RigidTypes.h>
#include "UniformRigidMass.h"
#include <sofa/SofaBase.h>

namespace sofa
{
namespace component
{
namespace constraint
{

template < class RigidDataTypes > 
class GyroscopicVelocityImpulseConstraint : public sofa::core::behavior::ProjectiveConstraintSet< RigidDataTypes >
{
public:
    SOFA_CLASS( SOFA_TEMPLATE( GyroscopicVelocityImpulseConstraint , RigidDataTypes), 
                SOFA_TEMPLATE( sofa::core::behavior::ProjectiveConstraintSet, RigidDataTypes) );
    
    enum { spatial_dimensions = RigidDataTypes::spatial_dimensions };

    typedef Inherit1 Inherit;

    typedef typename Inherit::DataVecCoord DataVecCoord;
    typedef typename Inherit::DataVecDeriv DataVecDeriv;
    typedef typename Inherit::DataMatrixDeriv  DataMatrixDeriv;

    typedef typename RigidDataTypes::Real        Real;
    typedef typename RigidDataTypes::Coord       RigidCoord;
    typedef typename RigidDataTypes::Deriv       RigidDeriv;
    typedef typename RigidDataTypes::MatrixDeriv RigidMatrixDeriv;

    typedef typename sofa::defaulttype::RigidMass< spatial_dimensions, Real > TRigidMass;
    typedef sofa::component::mass::UniformRigidMass<RigidDataTypes> TUniformRigidMass;
    
    SingleLink<MyType,TUniformRigidMass,BaseLink::FLAG_STRONGLINK> l_rigidMass;

    sofa::Data<bool> d_active;
    sofa::Data<bool> d_projectFreeMotion;

    virtual void init();
    
    /// ProjectiveConstraintSet API. Only projectVelocity has an implementation, 
    /// to add the velocity term relative to gyroscopic effect when we are in an implicit solver context. 
    /// 
    void projectVelocity(const core::MechanicalParams* mparams, DataVecDeriv& vData);

    
    void projectPosition(const core::MechanicalParams* /*mparams*/, DataVecCoord& /*xData*/) {};
    void projectResponse(const sofa::core::MechanicalParams* /*mparams*/, DataVecDeriv&   /*resData*/) {};
    void projectJacobianMatrix(const core::MechanicalParams* /*mparams*/, Data<RigidMatrixDeriv>& /*cData*/) {} ;

    void applyConstraint(const sofa::core::MechanicalParams* /*mparams*/ , const sofa::core::behavior::MultiMatrixAccessor* /*matrix*/) {};
    void applyConstraint(sofa::defaulttype::BaseMatrix * /*mat*/, unsigned int /*offset*/) {};
    void applyConstraint(sofa::defaulttype::BaseVector * /*vect*/, unsigned int /*offset*/) {};

    void projectMatrix( sofa::defaulttype::BaseMatrix* /*M*/, unsigned /*offset*/ ) {};

protected:

    GyroscopicVelocityImpulseConstraint();
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_CONSTRAINT_GYROSCOPICTVELOCITYIMPULSECONSTRAINT_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_BASE_MECHANICS_API GyroscopicVelocityImpulseConstraint<sofa::defaulttype::Rigid3dTypes>;
#endif 
#ifndef SOFA_DOUBLE
extern template class SOFA_BASE_MECHANICS_API GyroscopicVelocityImpulseConstraint<sofa::defaulttype::Rigid3fTypes>;
#endif 
#endif 



}

}

}

#endif //SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_H
