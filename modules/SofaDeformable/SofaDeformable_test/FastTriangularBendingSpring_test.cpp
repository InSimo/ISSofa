#include <gtest/gtest.h>

#include <SofaDeformable/FastTriangularBendingSprings.h>

#include <sofa/defaulttype/VecTypes.h>

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
    typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3dTypes  > BendingSpring;
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
    typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3dTypes  > BendingSpring;
    BendingSpring::EdgeSpring edgeSpring;

    {
        BendingSpring::VecCoord p;
        p.push_back(BendingSpring::Coord(0,0,0));
        p.push_back(BendingSpring::Coord(5,0,0));
        p.push_back(BendingSpring::Coord(5,5,0));
        p.push_back(BendingSpring::Coord(0,5,0));

        edgeSpring.setEdgeSpring(p, 0, 1, 2, 3, 0, true);
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
    typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3dTypes > BendingSpring;
    BendingSpring::EdgeSpring edgeSpring;

    {
        BendingSpring::VecCoord p;
        p.push_back(BendingSpring::Coord(0,3,1));
        p.push_back(BendingSpring::Coord(25,-5,3));
        p.push_back(BendingSpring::Coord(5,8,5));
        p.push_back(BendingSpring::Coord(0,5,1));

        edgeSpring.setEdgeSpring(p, 0, 1, 2, 3, 1000, true);
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


TEST(AddForce_test, checkThatNonPlanarTrianglesWithEqualDisplacementsDontCreateForces)
{
    typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3dTypes  > BendingSpring;
    BendingSpring::EdgeSpring edgeSpring;

    {
        BendingSpring::VecCoord p;
        p.push_back(BendingSpring::Coord(0,0,5));
        p.push_back(BendingSpring::Coord(10,0,10));
        p.push_back(BendingSpring::Coord(5,50,100));
        p.push_back(BendingSpring::Coord(3,5,50));

        edgeSpring.setEdgeSpring(p, 0, 1, 2, 3, 1000, true);
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



TEST(BendingVector_test, checkDifferencesBetweenFloatAndDouble)
{
	typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3fTypes  > BendingSpringFloat;
	typedef StubFastTriangularBendingSprings<sofa::defaulttype::Vec3dTypes  > BendingSpringDouble;
	BendingSpringFloat::EdgeSpring edgeSpringFloat;
	BendingSpringDouble::EdgeSpring edgeSpringDouble;

	{
		BendingSpringFloat::VecCoord pFloat;
		pFloat.push_back(BendingSpringFloat::Coord(0, 0, 5));
		pFloat.push_back(BendingSpringFloat::Coord(10, 0, 10));
		pFloat.push_back(BendingSpringFloat::Coord(5, 50, 100));
		pFloat.push_back(BendingSpringFloat::Coord(3, 5, 50));

		BendingSpringDouble::VecCoord pDouble;
		std::copy(pFloat.begin(), pFloat.end(), std::back_inserter(pDouble));

		edgeSpringFloat.setEdgeSpring(pFloat, 0, 1, 2, 3, 1000, true);
		edgeSpringDouble.setEdgeSpring(pDouble, 0, 1, 2, 3, 1000, true);

		sofa::defaulttype::Vec3fTypes::VecDeriv dpFloat;
		dpFloat.resize(4);
		dpFloat[0] = dpFloat[1] = dpFloat[2] = dpFloat[3] = sofa::defaulttype::Vec3fTypes::Deriv(100, 100, 100);

		sofa::defaulttype::Vec3dTypes::Deriv bendingVectorFloat = edgeSpringFloat.computeBendingVector(dpFloat);

		sofa::defaulttype::Vec3dTypes::VecDeriv dpDouble;
		std::copy(dpFloat.begin(), dpFloat.end(), std::back_inserter(dpDouble));

		sofa::defaulttype::Vec3dTypes::Deriv bendingVectorDouble = edgeSpringDouble.computeBendingVector(dpDouble);

		std::cout << "bendingDouble: " << edgeSpringDouble.lambda << " alphaDouble: " << edgeSpringDouble.alpha<<" bendingVectorDouble: " << bendingVectorDouble << std::endl;
		std::cout << "bendingFloat: " << edgeSpringFloat.lambda << " alphaFloat: " << edgeSpringFloat.alpha << " bendingVectorFloat: "  << bendingVectorFloat  << std::endl;

		EXPECT_LT((bendingVectorFloat - bendingVectorDouble).norm2(), 1e-5);
		EXPECT_NEAR(edgeSpringDouble.lambda, edgeSpringFloat.lambda, 1e-5);
		EXPECT_LT( (edgeSpringFloat.alpha - edgeSpringDouble.alpha).norm2(), 1e-5);

	}
}

} // namespace
