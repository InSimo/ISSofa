/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#include <gtest/gtest.h>
#include <sofa/core/topology/TopologyElementInfo.h>
#include <sofa/core/topology/Topology.h>




namespace topologyelementinfo_test
{

using namespace sofa::core::topology;
struct TopologyElementInfo_test : public ::testing::Test
{
    TopologyElementInfo_test()
    :p0(TopologyElementInfo::Point(5))
    ,p1(TopologyElementInfo::Point(6))
    ,e0(TopologyElementInfo::Edge(5))
    ,e1(TopologyElementInfo::Edge(6))
    ,t0(TopologyElementInfo::Triangle(5))
    ,t1(TopologyElementInfo::Triangle(6))
    {
    }

    TopologyElementInfo p0;
    TopologyElementInfo p1;

    TopologyElementInfo e0;
    TopologyElementInfo e1;

    TopologyElementInfo t0;
    TopologyElementInfo t1;
};

TEST_F(TopologyElementInfo_test, checkConstructor)
{
    EXPECT_TRUE(p0.type == sofa::core::topology::POINT);
    EXPECT_TRUE(p0.element.id == 5);

    EXPECT_TRUE(p1.type == sofa::core::topology::POINT);
    EXPECT_TRUE(p1.element.id == 6);

    EXPECT_TRUE(e0.type == sofa::core::topology::EDGE);
    EXPECT_TRUE(e0.element.id == 5);

    EXPECT_TRUE(e1.type == sofa::core::topology::EDGE);
    EXPECT_TRUE(e1.element.id == 6);

    EXPECT_TRUE(t0.type == sofa::core::topology::TRIANGLE);
    EXPECT_TRUE(t0.element.id == 5);

    EXPECT_TRUE(t1.type == sofa::core::topology::TRIANGLE);
    EXPECT_TRUE(t1.element.id == 6);
}

TEST_F(TopologyElementInfo_test, checkRelationalOperatorEqual)
{
    EXPECT_TRUE(p0==p0);
    EXPECT_TRUE(p1==p1);
    EXPECT_FALSE(p0==p1);

    EXPECT_TRUE(e0==e0);
    EXPECT_TRUE(e1==e1);
    EXPECT_FALSE(e0==e1);

    EXPECT_TRUE(t0==t0);
    EXPECT_TRUE(t1==t1);
    EXPECT_FALSE(t0==t1);

    EXPECT_FALSE(p0==e0);
    EXPECT_FALSE(p0==t0);
    EXPECT_FALSE(e0==t0);

}

TEST_F(TopologyElementInfo_test, checkRelationalOperatorNotEqual)
{
    EXPECT_FALSE(p0!=p0);
    EXPECT_FALSE(p1!=p1);
    EXPECT_TRUE(p0!=p1);

    EXPECT_FALSE(e0!=e0);
    EXPECT_FALSE(e1!=e1);
    EXPECT_TRUE(e0!=e1);

    EXPECT_FALSE(t0!=t0);
    EXPECT_FALSE(t1!=t1);
    EXPECT_TRUE(t0!=t1);

    EXPECT_TRUE(p0!=e0);
    EXPECT_TRUE(p0!=t0);
    EXPECT_TRUE(e0!=t0);
}

TEST_F(TopologyElementInfo_test, checkRelationalOperatorInferiorStrict)
{
    EXPECT_TRUE(p0 < p1);
    EXPECT_TRUE(p1 < e0);
    EXPECT_TRUE(e0 < e1);
    EXPECT_TRUE(e1 < t0);
    EXPECT_TRUE(t0 < t1);
}

TEST_F(TopologyElementInfo_test, checkRelationalOperatorInferiorEqual)
{
    EXPECT_TRUE(p0 <= p0);
    EXPECT_TRUE(p0 <= p1);
    EXPECT_TRUE(p1 <= p1);
    EXPECT_TRUE(p1 <= e0);
    EXPECT_TRUE(e0 <= e0);
    EXPECT_TRUE(e0 <= e1);
    EXPECT_TRUE(e1 <= e1);
    EXPECT_TRUE(e1 <= t0);
    EXPECT_TRUE(t0 <= t0);
    EXPECT_TRUE(t0 <= t1);
    EXPECT_TRUE(t1 <= t1);
}

TEST_F(TopologyElementInfo_test, checkRelationalOperatorSuperiorStrict)
{
    EXPECT_TRUE(t1 > t0);
    EXPECT_TRUE(t0 > e1);
    EXPECT_TRUE(e1 > e0);
    EXPECT_TRUE(e0 > p1);
    EXPECT_TRUE(p1 > p0);
}

TEST_F(TopologyElementInfo_test, checkRelationalOperatorSuperiorEqual)
{
    EXPECT_TRUE(t1 >= t0);
    EXPECT_TRUE(t0 >= e1);
    EXPECT_TRUE(e1 >= e0);
    EXPECT_TRUE(e0 >= p1);
    EXPECT_TRUE(p1 >= p0);

    EXPECT_TRUE(t1 >= t1);
    EXPECT_TRUE(t0 >= t0);
    EXPECT_TRUE(e1 >= e1);
    EXPECT_TRUE(e0 >= e0);
    EXPECT_TRUE(p1 >= p1);
    EXPECT_TRUE(p0 >= p0);
}

}
