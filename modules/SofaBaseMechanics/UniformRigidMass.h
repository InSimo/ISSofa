#ifndef SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_H
#define SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_H

#include <sofa/core/behavior/Mass.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <SofaBaseTopology/TopologyData.h>
#include <sofa/SofaBase.h>

namespace sofa
{
namespace component
{
namespace mass
{

template < class RigidDataTypes > 
class UniformRigidMass : public virtual sofa::core::behavior::Mass< RigidDataTypes >
{
public:
    SOFA_CLASS( SOFA_TEMPLATE( UniformRigidMass, RigidDataTypes ), 
                 SOFA_TEMPLATE( sofa::core::behavior::Mass, RigidDataTypes ) );
    
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


    sofa::component::topology::PointData< sofa::helper::vector<TRigidMass> > d_mass;
    sofa::Data< bool  >      d_useGyroscopicExplicit;
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

    virtual SReal getKineticEnergy( const sofa::core::MechanicalParams* /*mparams*/, const DataVecDeriv& /*v*/) const
    {
        return SReal(0);
    }

    virtual SReal getPotentialEnergy( const sofa::core::MechanicalParams* /*mparams*/, const DataVecCoord& /*x*/ ) const
    {
        return SReal(0);
    }


    virtual void draw(const sofa::core::visual::VisualParams* vparams);

protected:

    UniformRigidMass();

    // handlers
    class PointInfosHandler : public sofa::component::topology::TopologyDataHandler<sofa::core::topology::Topology::Point, sofa::helper::vector<TRigidMass> >
    {
    public:
        PointInfosHandler(sofa::component::topology::PointData<sofa::helper::vector<TRigidMass> >* d)
            : sofa::component::topology::TopologyDataHandler<sofa::core::topology::Topology::Point, sofa::helper::vector<TRigidMass> >(d)
        {}

        void applyCreateFunction(unsigned int /*p*/, TRigidMass& rigidMass,
            const sofa::core::topology::Topology::Point & /*point*/,
            const sofa::helper::vector< unsigned int > & /*ancestors*/,
            const sofa::helper::vector< double > & /*coefs*/)
        {
            // add null mass for new vertex
            rigidMass.mass = 0;
        }
    };
    PointInfosHandler* m_pointInfosHandler;

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
