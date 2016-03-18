#ifndef SOFACONSTRAINT_RIGIDMASSCOMPLIANCEENGINE_H
#define SOFACONSTRAINT_RIGIDMASSCOMPLIANCEENGINE_H

#include <sofa/core/DataEngine.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/VecId.h>

namespace sofa
{
namespace component
{
namespace engine
{

template< class TRigidDataTypes >
class RigidMassComplianceEngine : public sofa::core::DataEngine
{
public:
    SOFA_CLASS( SOFA_TEMPLATE(RigidMassComplianceEngine, TRigidDataTypes ),
                sofa::core::DataEngine );

    enum { spatial_dimensions = TRigidDataTypes::spatial_dimensions };
    typedef typename TRigidDataTypes::VecReal                                 VecReal;
    typedef typename TRigidDataTypes::Real                                    Real;
    typedef typename TRigidDataTypes::Coord                                   TRigidCoord;
    typedef typename TRigidDataTypes::VecCoord                                TRigidVecCoord;
    typedef typename sofa::defaulttype::RigidMass< spatial_dimensions, Real > TRigidMass;
    typedef sofa::defaulttype::Vec<7,Real>                                    RigidCompliance;

    sofa::Data< TRigidVecCoord >                         d_rigidPosition;
    sofa::Data< sofa::helper::vector<TRigidMass> >       d_rigidMass;
    sofa::Data< sofa::helper::vector<RigidCompliance> >  d_rigidCompliance;
    sofa::Data< VecReal >                                d_rigidComplianceUnzipped;

    void init();

    void update();

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const RigidMassComplianceEngine<TRigidDataTypes>* = NULL)
    {
        return TRigidDataTypes::Name();
    }

protected:
    RigidMassComplianceEngine();

    RigidCompliance rigidComplianceFromInertiaMassMatrix( const typename TRigidMass::Mat3x3& invInertiaGlobal, Real mass, Real h2 ) const;


};

}

}

}

#endif // SOFACONSTRAINT_RIGIDMASSCOMPLIANCEENGINE_H
