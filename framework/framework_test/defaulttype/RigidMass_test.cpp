#include <gtest/gtest.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/helper/system/config.h>
#include <math.h>

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
            EXPECT_EQ(massEqMassMultFactor.inertiaMassMatrix[i][j], massMultEqFactor.inertiaMassMatrix[i][j] );
        }
    }
}

TEST( RigidMassTest, checkInertiaLocalToWorldOperation )
{
    Rigid3Mass mass;
    mass.mass          = Real(1);
    mass.volume        = Real(1);
    mass.inertiaMatrix[0][0] = Real(0.008416676);
    mass.inertiaMatrix[1][1] = Real(0.041416667);
    mass.inertiaMatrix[2][2] = Real(0.035166666);
    mass.recalc();

    Rigid3Mass::Mat3x3 rotation(sofa::defaulttype::NOINIT);
    sofa::defaulttype::Quat q( sofa::defaulttype::Vec3d(1,0,0), Real(M_PI/2) );
    q.toMatrix(rotation);

    const Rigid3Mass::Mat3x3::Line inertiaDiagonalMassMatrix = sofa::defaulttype::diagonal(mass.inertiaMassMatrix);
    const Rigid3Mass::Mat3x3 inertiaMassMatrixWorld = rotation.multDiagonal( inertiaDiagonalMassMatrix ) * rotation.transposed();

    const Rigid3Mass::Mat3x3 inertiaMassMatrixWorld2 = rotation *  mass.inertiaMassMatrix  * rotation.transposed();
   
    for(std::size_t i=0;i<3;++i)
    {
        for(std::size_t j=0; j<3;++j)
        {
            EXPECT_EQ(inertiaMassMatrixWorld[i][j], inertiaMassMatrixWorld2[i][j] );
        }
    }
}


}
