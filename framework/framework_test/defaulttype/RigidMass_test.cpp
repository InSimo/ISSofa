#include <gtest/gtest.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace 
{

typedef sofa::defaulttype::Rigid3Mass Rigid3Mass;
typedef sofa::defaulttype::Rigid3Types::Deriv RigidDeriv;
typedef Rigid3Mass::Mat3x3 Mat3x3;
typedef Rigid3Mass::Real Real;

TEST( RigidMassTest, checkAssignementOperatorGivesConsistentResult )
{
    Rigid3Mass mass;
    mass.mass          = Real(100);
    mass.volume        = Real(1.0);
    mass.inertiaMatrix = Mat3x3::Identity()*Real(100);

    mass.recalc();

    Rigid3Mass otherMass = mass;

    EXPECT_EQ( mass.mass, otherMass.mass );

    for(std::size_t i=0;i<3;++i)
    {
        for(std::size_t j=0; j<3;++j)
        {
            EXPECT_EQ(mass.inertiaMassMatrix[i], otherMass.inertiaMassMatrix[i] );
        }
    }


}

TEST( RigidMassTest, checkOperatorMultiplicationWithScalarGivesConsistentResult )
{
    Rigid3Mass mass;
    mass.mass          = Real(100);
    mass.volume        = Real(1.0);
    mass.inertiaMatrix = Mat3x3::Identity()*Real(100);

    mass.recalc();

    Real factor = Real(-22);

    // mimic what is done in to compute the damping term -r_m * M * v using addMdx.

    Rigid3Mass massEqMassMultFactor = mass*factor;

    Rigid3Mass massMultEqFactor = mass;
    massMultEqFactor*=factor;


    EXPECT_EQ( massEqMassMultFactor.mass, massMultEqFactor.mass );

    for(std::size_t i=0;i<3;++i)
    {
        for(std::size_t j=0; j<3;++j)
        {
            EXPECT_EQ(massEqMassMultFactor.inertiaMassMatrix[i], massMultEqFactor.inertiaMassMatrix[i] );
        }
    }

    //RigidDeriv dx;
    //dx.getLinear()  = RigidDeriv::Vec3( -5.26601e-005, 7.911e-006, -0.00146761 );
    //dx.getAngular() = RigidDeriv::Vec3( 0.604655, -1.12543e-005, 5.2351e-006 );
    //
    //RigidDeriv df1 = massEqMassMultFactor * dx;
    //RigidDeriv df2 = massMultEqFactor * dx;

}


}
