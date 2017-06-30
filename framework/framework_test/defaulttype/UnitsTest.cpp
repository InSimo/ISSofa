
#include <gtest/gtest.h>
#include <sofa/defaulttype/Units.h>
#include <sofa/defaulttype/EnumTypeInfo.h>
#include <iostream>



namespace sofa
{
namespace unitsTest
{
    
TEST(UnitsTest, checkUnits)
{
    units::Mass             m0(1);
    units::Acceleration     a0(2);
    units::Time             t0(4);
    units::Length           l0(8);
    units::Stiffness        s0(16);
    units::Scalar           sc0(32);


    ASSERT_EQ(m0 * a0, units::Force(2));
    ASSERT_EQ(l0 / t0, units::Velocity(2));
    ASSERT_EQ(s0.value(), 16);

}


} // namespace unitsTest
} // namespace sofa