#include <gtest/gtest.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/helper/system/config.h>
#include <math.h>
#include <fstream>
#include <random>

// How many loop iterations to run for each randomized test
// Increase to stress test after changes in Quater.
#define NITERATIONS 10000

namespace 
{

typedef double Real;
typedef sofa::helper::Quater<Real> Quat;
typedef sofa::defaulttype::RigidCoord<3, Real> RigidCoord;
typedef sofa::defaulttype::RigidDeriv<3, Real> RigidDeriv;
typedef sofa::defaulttype::Vec<3,Real> Vec3d;
typedef sofa::defaulttype::Mat<3,3,Real> Mat3x3;


TEST(QuaternionTest, testEpsilonSetupForUnitQuaternion)
{
    Quat q(0, -0.707, 0, 0.707);// rotation of -90 degrees on the y axis
    Real qnorm2 = q.norm2();

    EXPECT_FALSE( std::abs(qnorm2-1) < 1e-6);
    EXPECT_TRUE ( std::abs(qnorm2-1) < 1e-3);
}

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

TEST(QuaternionTest, checkAxisToQuatAndRelatedMethods)
{
    const Real epsilon = 1e-6;
    constexpr std::size_t nTests = NITERATIONS;

    std::ranlux48_base rgen;
    std::uniform_real_distribution<Real> rrnd(-1.0f,1.0f);

    // loop many times, but stop after the first failure to not spam the log with thousands of errors
    for (std::size_t ti = 0; ti < nTests && !::testing::Test::HasNonfatalFailure(); ++ti)
    {
        Vec3d axis(0, 0, 0);
        for (std::size_t i=0; i<3; ++i)
        {
            axis[i] = rrnd(rgen);
        }
        axis.normalize();

        Real angle = rrnd(rgen)*M_PI; // angles between -Pi and Pi

        Quat q1;
        q1.axisToQuat(axis, angle);

        // expected properties of output quaternion
        {
            EXPECT_NEAR(std::fabs(q1[3]), cos(angle*0.5), epsilon);
            EXPECT_NEAR(q1.norm(), 1.0, epsilon);
            Vec3d rotated_axis = q1.rotate(axis);
            EXPECT_NEAR((rotated_axis-axis).norm(), 0, epsilon);
        }

        // check that input axis does not have to be normalized
        {
            Real norm = pow(10.0f,rrnd(rgen)); // random norms between 0.1 and 10.0
            Quat q2;
            q2.axisToQuat(axis*norm, angle);
            Quat qDiff = Quat::elementWiseDifference(q1,q2);
            EXPECT_NEAR(qDiff.norm(), 0, epsilon);
        }

        // check that quatToAxis(axisToQuat()) results in the input values
        {
            Vec3d out_axis;
            Real out_angle = 0;
            q1.quatToAxis(out_axis, out_angle, false);
            if (angle < 0)
            {
                // we check negative input angles, but in this case the output of quatToAxis
                // should be the opposite axis
                out_angle = -out_angle;
                out_axis = -out_axis;
            }
            EXPECT_NEAR(out_angle, angle, epsilon);
            if (std::fabs(angle) > epsilon)
            {
                EXPECT_NEAR(out_axis.norm(), 1.0, epsilon);
                EXPECT_NEAR((out_axis-axis).norm(), 0, epsilon);
            }
        }

        // check equivalence with exponentialMap(axis*angle)
        {
            Quat q2 = Quat::exponentialMap(axis*angle);
            Quat qDiff = Quat::elementWiseDifference(q1,q2);
            EXPECT_NEAR(qDiff.norm2(), 0, 1e-10);
        }

        // check that getLog() gives axis*angle
        {
            Vec3d log = q1.getLog();
            EXPECT_NEAR(log.norm(), fabs(angle), epsilon);
            EXPECT_NEAR((log-axis*angle).norm(), 0, epsilon);
        }
    }
}

TEST(QuaternionTest, checkPlusIsSameAsMult)
{
    // we have 4 operators that have different implementations but they should be equivalent
    // (at least for normalized quaternions, and with a flipped operands order).
    // Check that this is the case before factoring the code.

    const Real epsilon = 1e-6;
    constexpr std::size_t nTests = NITERATIONS;

    std::ranlux48_base rgen;
    std::uniform_real_distribution<Real> rrnd(-1.0f,1.0f);

    // loop many times, but stop after the first failure to not spam the log with thousands of errors
    for (std::size_t ti = 0; ti < nTests && !::testing::Test::HasNonfatalFailure(); ++ti)
    {
        Quat q1, q2;
        for (std::size_t i=0; i<4; ++i)
        {
            q1[i] = rrnd(rgen);
            q2[i] = rrnd(rgen);
        }
        q1.normalize();
        q2.normalize();

        Quat qmul = q2 * q1;
        Quat qmuleq = q2; qmuleq *= q1;
        Quat qadd = q1 + q2;
        Quat qaddeq = q1; qaddeq += q2;
        Quat qmulinv = (q1.inverse() * q2.inverse()).inverse();

        // expected properties of output quaternions
        {
            EXPECT_NEAR(qmul.norm(), 1.0, epsilon);
            EXPECT_NEAR(qadd.norm(), 1.0, epsilon);
            EXPECT_NEAR(qmuleq.norm(), 1.0, epsilon);
            EXPECT_NEAR(qaddeq.norm(), 1.0, epsilon);
        }

        // expected equivalence to applying the two quaternions in sequence
        for (std::size_t tj = 0; tj < 5; ++tj)
        {
            Vec3d v0;
            for (std::size_t i=0; i<3; ++i)
            {
                v0[i] = rrnd(rgen)*10.0;
            }
            Vec3d v1 = q1.rotate(v0);
            Vec3d v2 = q2.rotate(v1);
            Vec3d v12 = qmul.rotate(v0);
            EXPECT_NEAR((v2-v12).norm(), 0.0, epsilon);
            Vec3d v0b = qmul.inverseRotate(v12);
            EXPECT_NEAR((v0-v0b).norm(), 0.0, epsilon);
            Mat3x3 m;
            qmul.toMatrix(m);
            Vec3d v12m = m*v0;
            EXPECT_NEAR((v12m-v12).norm(), 0.0, epsilon);
        }

        // compare qmul and qmuleq
        {
            EXPECT_NEAR(std::fabs(qmuleq[3]), std::fabs(qmul[3]), epsilon);
            if (qmuleq.dot(qmul) < 0) qmuleq *= -1;
            EXPECT_NEAR(Quat::elementWiseDifference(qmuleq,qmul).norm(), 0, epsilon);
        }

        // compare qadd and qaddeq
        {
            EXPECT_NEAR(std::fabs(qaddeq[3]), std::fabs(qadd[3]), epsilon);
            if (qaddeq.dot(qadd) < 0) qaddeq *= -1;
            EXPECT_NEAR(Quat::elementWiseDifference(qaddeq,qadd).norm(), 0, epsilon);
        }

        // compare qmul and qadd
        {
            EXPECT_NEAR(std::fabs(qadd[3]), std::fabs(qmul[3]), epsilon);
            if (qadd.dot(qmul) < 0) qadd *= -1;
            EXPECT_NEAR(Quat::elementWiseDifference(qadd,qmul).norm(), 0, epsilon);
        }

        // compare qmul and qmulinv
        {
            EXPECT_NEAR(std::fabs(qmulinv[3]), std::fabs(qmul[3]), epsilon);
            if (qmulinv.dot(qmul) < 0) qmulinv *= -1;
            EXPECT_NEAR(Quat::elementWiseDifference(qmulinv,qmul).norm(), 0, epsilon);
        }
    }
}

TEST(QuaternionTest, checkRotationsAndMatrices)
{
    const Real epsilon = 1e-6;
    constexpr std::size_t nTests = NITERATIONS;

    std::ranlux48_base rgen;
    std::uniform_real_distribution<Real> rrnd(-1.0f,1.0f);

    // loop many times, but stop after the first failure to not spam the log with thousands of errors
    for (std::size_t ti = 0; ti < nTests && !::testing::Test::HasNonfatalFailure(); ++ti)
    {
        Quat q1;
        for (std::size_t i=0; i<4; ++i)
        {
            q1[i] = rrnd(rgen);
        }
        q1.normalize();
        Mat3x3 m;
        Real m44[4][4];
        float mgl[16];
        q1.toMatrix(m);
        q1.buildRotationMatrix(m44);
        q1.writeOpenGlMatrix(mgl);
        for(std::size_t i = 0; i < 3; ++i)
        {
            for(std::size_t j = 0; j < 3; ++j)
            {
                EXPECT_EQ(m[i][j], m44[i][j]);
                EXPECT_EQ((float)m[i][j], mgl[j*4+i]);
            }
        }
        for (std::size_t tj = 0; tj < 10; ++tj)
        {
            Vec3d v0;
            for (std::size_t i=0; i<3; ++i)
            {
                v0[i] = rrnd(rgen)*10.0;
            }
            Vec3d v1 = q1.rotate(v0);
            Vec3d v0b = q1.inverseRotate(v1);
            EXPECT_NEAR((v0-v0b).norm(), 0.0, epsilon);
            Vec3d v1m = m*v0;
            EXPECT_NEAR((v1m-v1).norm(), 0.0, epsilon);
            Vec3d v0m = m.multTranspose(v1);
            EXPECT_NEAR((v0m-v0).norm(), 0.0, epsilon);
        }
    }
}

TEST(QuaternionTest, CheckSlerpConservativeness)
{
    constexpr std::size_t nTests = NITERATIONS;
    std::ranlux48_base rgen;
    std::uniform_real_distribution<Real> rrnd_t(0.0f,1.0f);

    Quat a = {0,0,0,1};
    Quat b, c;
    Quat s0 = a, s1 = a;

    const Vec3d& axis = {0,0,1};
    const Real& maxAngleLinearised = std::acos(0.999)*180./M_PI*2.;

    std::uniform_real_distribution<Real> rrnd_a(-maxAngleLinearised,maxAngleLinearised);
    Real angle = maxAngleLinearised;

    // loop many times, but stop after the first failure to not spam the log with thousands of errors
    for (std::size_t ti = 0; ti < nTests && !::testing::Test::HasNonfatalFailure(); ++ti)
    {
        angle = rrnd_a(rgen);
        b.axisToQuat(axis, angle*M_PI/180., 0.);

        //ensure slerp(a,b) is conservative for small angles
        const Real& t = rrnd_t(rgen);
        s0.slerp(b,t);
        EXPECT_TRUE(s0.isUnit());
    }
}

}
