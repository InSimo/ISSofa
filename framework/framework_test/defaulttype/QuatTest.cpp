#include <gtest/gtest.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/helper/system/config.h>
#include <math.h>
#include <fstream>

namespace 
{

typedef sofa::defaulttype::Quat Quat;
typedef sofa::defaulttype::RigidCoord<3, double> RigidCoord;
typedef sofa::defaulttype::RigidDeriv<3, double> RigidDeriv;
typedef sofa::defaulttype::Vec3d Vec3d;

TEST(QuaternionTest, integrateRigidCompareLargeDisplacementIntegrationWithSmallIncrements)
{
    const std::size_t factor = 1000;
    const double angle = 2.0*M_PI / 3.0;

    RigidCoord  x0(Vec3d( 0,0,0 ), Quat(0, 0, 0, 1));
    RigidDeriv dx0(Vec3d( 0,0,0 ), Vec3d( angle / double(factor), 0.0, 0.0 ) );


    for (std::size_t i = 0; i < factor; ++i)
    {
        x0 += dx0;
    }

    RigidCoord  x1( Vec3d( 0,0,0 ), Quat( 0,0,0,1 ) );
    RigidDeriv dx1( Vec3d( 0,0,0 ), Vec3d( angle, 0.0, 0.0 ) );
    x1 += dx1;

    EXPECT_NEAR(x0.getOrientation().norm(),double(1), 1e-10); // make sure we still have a unit quaternion

    Quat orientationDiff = Quat::elementWiseDifference(x1.getOrientation(),x0.getOrientation());

    Vec3d linearDiff = x1.getCenter() - x0.getCenter();

    EXPECT_NEAR(linearDiff.norm2(), 0, 1e-10);
    EXPECT_NEAR(orientationDiff.norm2(), 0, 1e-10);

    RigidCoord rigidDiff = x1-x0;

    EXPECT_NEAR(rigidDiff.getCenter().norm2(), 0, 1e-10);

    for (std::size_t i=0; i<3; ++i)
    {
        EXPECT_NEAR(rigidDiff.getOrientation()[i], 0, 1e-10);
    }
    EXPECT_NEAR(rigidDiff.getOrientation()[3], 1, 1e-10);

    EXPECT_NEAR(rigidDiff.getOrientation().norm(), 1, 1e-10);

}

TEST(QuaternionTest, integrateLargeAngularDisplacementTest)
{
    const double epsilon = 1e-6;

    const double angle = 2.0*M_PI / 3.0;
    Vec3d rotation(angle, 0 , 0);
    
    Quat x0 = Quat::identity();
    x0.integrateExponentialMap(rotation);

    Quat x1 = Quat::identity();
    x1.integrate(rotation);

    Quat result(0.8660254,0,0,0.5);

    for(std::size_t i=0;i<Quat::size(); ++i)
    {
        EXPECT_TRUE(std::fabs(x0[i] - result[i]) < epsilon);
    }

    EXPECT_FALSE(std::fabs(x1[0] - result[0]) < epsilon);

}


TEST(QuaternionTest, checkLogOfExponentialIsIdentity)
{
    const double epsilon = 1e-6;

    for (std::size_t i=0; i<3; ++i)
    {
        Vec3d direction(0, 0, 0);
        direction[i] = 1.0;  // i == 0 : x axis, i == 1 : y axis, i == 3 z axis; 

        const std::size_t jMax = 10;
        const double angle_step = M_PI/jMax;

        for (std::size_t j=0; j<jMax; ++j)
        {
            const double angle_expected = angle_step*jMax;
            const Quat exp(direction, angle_expected);
            const Vec3d log = exp.getLog();

            double angle = log.norm();
            Vec3d axis   = log;
            axis /= angle;

            EXPECT_NEAR(angle, angle_expected, epsilon);
            EXPECT_NEAR((axis-direction).norm(), 0, epsilon);
        }
    }
}

}
