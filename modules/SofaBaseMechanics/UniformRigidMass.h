#ifndef SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_H
#define SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_H

#include <sofa/core/behavior/Mass.h>
#include <sofa/core/behavior/ProjectiveConstraintSet.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/SofaBase.h>

namespace sofa
{
namespace component
{
namespace mass
{

template < class RigidDataTypes > 
class UniformRigidMass : public virtual sofa::core::behavior::Mass< RigidDataTypes >,
                         private virtual sofa::core::behavior::ProjectiveConstraintSet< RigidDataTypes >
{
public:
    SOFA_CLASS2( SOFA_TEMPLATE( UniformRigidMass, RigidDataTypes ), 
                 SOFA_TEMPLATE( sofa::core::behavior::Mass, RigidDataTypes ), 
                 SOFA_TEMPLATE( sofa::core::behavior::ProjectiveConstraintSet, RigidDataTypes) );
    
    enum { spatial_dimensions = RigidDataTypes::spatial_dimensions };

    typedef Inherit1 Inherit;

    typedef typename Inherit::DataVecCoord DataVecCoord;
    typedef typename Inherit::DataVecDeriv DataVecDeriv;
    typedef typename Inherit::VecCoord     VecCoord;
    typedef typename Inherit::VecDeriv     VecDeriv;

    typedef typename RigidDataTypes::Real        Real;
    typedef typename RigidDataTypes::Coord       RigidCoord;
    typedef typename RigidDataTypes::Deriv       RigidDeriv;
    typedef typename RigidDataTypes::MatrixDeriv RigidMatrixDeriv;

    typedef typename sofa::defaulttype::RigidMass< spatial_dimensions, Real > TRigidMass;

    

    sofa::Data< sofa::helper::vector<TRigidMass> > d_mass;
    sofa::Data< bool  >      d_useGyroscopicExplicit;
    sofa::Data< bool  >      d_useGyroscopicImplicit;
    sofa::Data< Real  >      d_maxGyroscopicForce;
    sofa::Data< float >      d_drawAxisFactor;

    virtual void init();

    virtual void reinit();

    /// Mass API 

    virtual void addForce(const sofa::core::MechanicalParams* mparams, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v);

    virtual void addMDx( const sofa::core::MechanicalParams* mparams, DataVecDeriv& d_f, const DataVecDeriv& d_dx, double factor );

    virtual void addMToMatrix(const core::MechanicalParams *mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix);

    //template<class MatrixWriter>
    //void addMToMatrixT(const core::MechanicalParams* mparams, MatrixWriter m);

    virtual SReal getKineticEnergy( const sofa::core::MechanicalParams* mparams, const DataVecDeriv& v) const
    {
        return SReal(0);
    }

    virtual SReal getPotentialEnergy( const sofa::core::MechanicalParams* mparams, const DataVecCoord& x ) const
    {
        return SReal(0);
    }


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

    virtual void draw(const sofa::core::visual::VisualParams* vparams);

    // apparently this needs to be redefined because of a bug in multiple inheritance in msvc 2010 
    // commenting the methods below result in error C2250 "ambiguous inheritance".
    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const UniformRigidMass<RigidDataTypes>* = NULL)
    {
        return RigidDataTypes::Name();
    }

    template<class T>
    static bool canCreate(T*& obj, sofa::core::objectmodel::BaseContext* context, sofa::core::objectmodel::BaseObjectDescription* arg)
    {
        return Inherit1::canCreate(obj, context, arg) && Inherit2::canCreate(obj,context,arg);
    }

    sofa::core::behavior::MechanicalState<RigidDataTypes>* getMState()
    {
        return Inherit1::getMState();
    }




protected:

    UniformRigidMass();

};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_BASE_MECHANICS_API UniformRigidMass<sofa::defaulttype::Rigid3dTypes>;
#endif 
#ifndef SOFA_DOUBLE
extern template class SOFA_BASE_MECHANICS_API UniformRigidMass<sofa::defaulttype::Rigid3fTypes>;
#endif 
#endif 

}

}

}

#endif //SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_H
