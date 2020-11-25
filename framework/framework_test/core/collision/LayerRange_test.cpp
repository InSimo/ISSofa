/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

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
