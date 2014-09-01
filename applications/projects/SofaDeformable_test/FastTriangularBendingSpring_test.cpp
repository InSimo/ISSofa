#include <gtest/gtest.h>

#include <sofa/component/forcefield/FastTriangularBendingSprings.h>

#include "VecTypes.h"

namespace
{

// Types

template<typename DataTypes>
struct StubFastTriangularBendingSprings : public sofa::component::forcefield::FastTriangularBendingSprings<DataTypes>
{
    typedef typename StubFastTriangularBendingSprings<DataTypes>::EdgeSpring EdgeSpring;
    
    typedef typename DataTypes::Real Real;

    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecDeriv VecDeriv;
};

// Tests

TEST(AddForce_test, checkThatPlanarTrianglesWithNoDisplacementsDontCreateForces)
{
    typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3fTypes > BendingSpring;
    BendingSpring::EdgeSpring edgeSpring;

    {
        BendingSpring::VecCoord p;
        p.push_back(BendingSpring::Coord(0,0,0));
        p.push_back(BendingSpring::Coord(5,0,0));
        p.push_back(BendingSpring::Coord(5,5,0));
        p.push_back(BendingSpring::Coord(0,5,0));

        edgeSpring.setEdgeSpring(p, 0, 1, 2, 3, 0);
    }

    BendingSpring::VecDeriv df(4);
        
    {        
        BendingSpring::VecDeriv dp(4);
        dp[0] = dp[1] = dp[2] = dp[3] = BendingSpring::Deriv(0, 0, 0);
        df[0] = df[1] = df[2] = df[3] = BendingSpring::Deriv(0, 0, 0);
        edgeSpring.addDForce(df, dp, 1);
    }

    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[0]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[1]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[2]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[3]);
}

TEST(AddForce_test, checkThatPlanarTrianglesWithDisplacementsAndNoStiffnessDontCreateForces)
{
    typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3fTypes > BendingSpring;
    BendingSpring::EdgeSpring edgeSpring;

    {
        BendingSpring::VecCoord p;
        p.push_back(BendingSpring::Coord(0,0,0));
        p.push_back(BendingSpring::Coord(5,0,0));
        p.push_back(BendingSpring::Coord(5,5,0));
        p.push_back(BendingSpring::Coord(0,5,0));

        edgeSpring.setEdgeSpring(p, 0, 1, 2, 3, 0);
    }

    BendingSpring::VecDeriv df(4);
        
    {        
        BendingSpring::VecDeriv dp(4);
        dp[0] = BendingSpring::Deriv(0, 0, 1);
        dp[1] = dp[2] = dp[3] = BendingSpring::Deriv(0, 0, 0);
        df[0] = df[1] = df[2] = df[3] = BendingSpring::Deriv(0, 0, 0);
        edgeSpring.addDForce(df, dp, 1);
    }

    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[0]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[1]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[2]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[3]);
}

TEST(AddForce_test, checkThatNonPlanarTrianglesWithNoDisplacementsDontCreateForces)
{
    typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3fTypes > BendingSpring;
    BendingSpring::EdgeSpring edgeSpring;

    {
        BendingSpring::VecCoord p;
        p.push_back(BendingSpring::Coord(0,3,1));
        p.push_back(BendingSpring::Coord(25,-5,3));
        p.push_back(BendingSpring::Coord(5,8,5));
        p.push_back(BendingSpring::Coord(0,5,1));

        edgeSpring.setEdgeSpring(p, 0, 1, 2, 3, 1000);
    }

    BendingSpring::VecDeriv df(4);
        
    {        
        BendingSpring::VecDeriv dp(4);
        dp[0] = dp[1] = dp[2] = dp[3] = BendingSpring::Deriv(0, 0, 0);
        df[0] = df[1] = df[2] = df[3] = BendingSpring::Deriv(0, 0, 0);
        edgeSpring.addDForce(df, dp, 1);
    }

    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[0]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[1]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[2]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[3]);
}

//TEST(AddForce_test, checkThatPlanarTrianglesWithDisplacementsInPlaneDontCreateForces)
//{
//    typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3fTypes > BendingSpring;
//    BendingSpring::EdgeSpring edgeSpring;
//
//    {
//        BendingSpring::VecCoord p;
//        p.push_back(BendingSpring::Coord(0,0,0));
//        p.push_back(BendingSpring::Coord(5,0,0));
//        p.push_back(BendingSpring::Coord(5,5,0));
//        p.push_back(BendingSpring::Coord(0,5,0));
//
//        edgeSpring.setEdgeSpring(p, 0, 1, 2, 3, 1);
//    }
//
//    BendingSpring::VecDeriv df(4);
//        
//    {        
//        BendingSpring::VecDeriv dp(4);
//        dp[0] = BendingSpring::Deriv(10, 0, 0);
//        dp[1] = BendingSpring::Deriv(10, 10, 0);
//        dp[2] = BendingSpring::Deriv(100, 50, 0);
//        dp[3] = BendingSpring::Deriv(0, 10, 0);
//        df[0] = df[1] = df[2] = df[3] = BendingSpring::Deriv(0, 0, 0);
//        edgeSpring.addDForce(df, dp, 1);
//    }
//
//    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[0]);
//    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[1]);
//    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[2]);
//    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[3]);
//}

TEST(AddForce_test, checkThatNonPlanarTrianglesWithEqualDisplacementsDontCreateForces)
{
    typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3fTypes > BendingSpring;
    BendingSpring::EdgeSpring edgeSpring;

    {
        BendingSpring::VecCoord p;
        p.push_back(BendingSpring::Coord(0,0,5));
        p.push_back(BendingSpring::Coord(10,0,10));
        p.push_back(BendingSpring::Coord(5,50,100));
        p.push_back(BendingSpring::Coord(3,5,50));

        edgeSpring.setEdgeSpring(p, 0, 1, 2, 3, 1000);
    }

    BendingSpring::VecDeriv df(4);
        
    {        
        BendingSpring::VecDeriv dp(4);
        dp[0] = dp[1] = dp[2] = dp[3] = BendingSpring::Deriv(100, 100, 100);
        df[0] = df[1] = df[2] = df[3] = BendingSpring::Deriv(0, 0, 0);
        edgeSpring.addDForce(df, dp, 1000);
    }

    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[0]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[1]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[2]);
    EXPECT_EQ(BendingSpring::Coord(0, 0, 0), df[3]);
}

} // namespace
