/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include <gtest/gtest.h>
#include <sofa/core/collision/LayerRange.h>

namespace layerrange_test
{

using namespace sofa::core::collision;

TEST( TestLayerRangeInclude, checkLayer00IncludesLayer0)
{
    LayerRange layer00(0,0);

    EXPECT_TRUE( layer00.isLayerIncludingLayerID( 0 ) );

}

TEST( TestLayerRangeInclude, checkLayer02IncludesLayer012)
{
    LayerRange layer02(0,2);

    EXPECT_TRUE( layer02.isLayerIncludingLayerID( 0 ) );
    EXPECT_TRUE( layer02.isLayerIncludingLayerID( 1 ) );
    EXPECT_TRUE( layer02.isLayerIncludingLayerID( 2 ) );
}

TEST( TestLayerRangeInclude, checkLayer11DoesNotIncludeLayer0)
{
    LayerRange layer11(1,1);

    EXPECT_FALSE( layer11.isLayerIncludingLayerID(0) );
}

TEST( TestLayerRangeInclude, checkLayer11DoesNotIncludeLayer2)
{
    LayerRange layer11(1,1);
    EXPECT_FALSE( layer11.isLayerIncludingLayerID(2) );
}

TEST( TestLayerRangeIsOverlapping, checkLayer00OverlapsLayer00Layer01Layer02 )
{
    LayerRange layer00(0,0);
    LayerRange layer01(0,1);
    LayerRange layer02(0,2);
    EXPECT_TRUE( layer00.isOverlapping( layer00) );
    EXPECT_TRUE( layer00.isOverlapping( layer01) );
    EXPECT_TRUE( layer00.isOverlapping( layer02) );
}

TEST( TestLayerRangeIsOverlapping, checkLayer02OverLapsLayer00Layer01Layer11Layer02Layer22)
{
    LayerRange layer02(0,2);
    LayerRange layer00(0,0);
    LayerRange layer01(0,1);
    LayerRange layer11(1,1);
    LayerRange layer22(2,2);

    EXPECT_TRUE( layer02.isOverlapping( layer00) );
    EXPECT_TRUE( layer02.isOverlapping( layer01) );
    EXPECT_TRUE( layer02.isOverlapping( layer11) );
    EXPECT_TRUE( layer02.isOverlapping( layer02) );
    EXPECT_TRUE( layer02.isOverlapping( layer22) );
}

TEST( TestLayerRangeIsOverlapping, checkLayer00NotOverLapsLayer11Layer12Layer22)
{
    LayerRange layer00(0,0);
    LayerRange layer11(1,1);
    LayerRange layer12(1,2);
    LayerRange layer22(1,2);

    EXPECT_FALSE( layer00.isOverlapping( layer11 ) );
    EXPECT_FALSE( layer00.isOverlapping( layer12 ) );
    EXPECT_FALSE( layer00.isOverlapping( layer22 ) );
}

TEST( TestLayerRangeIsOverlapping, checkLayer01NotOverLapsLayer22 )
{
    LayerRange layer01(0,1);
    LayerRange layer22(2,2);
    LayerRange layer02(0,2);

    EXPECT_FALSE( layer01.isOverlapping( layer22 ) );
}




}
