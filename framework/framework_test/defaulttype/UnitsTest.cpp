
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
    units::Mass<double>             m0(1);
    units::Acceleration<double>     a0(2);
    units::Time<double>             t0(4);
    units::Length<double>           l0(8);
    units::Stiffness<double>        s0(16);
    units::Scalar<double>           sc0(32);


    ASSERT_EQ(m0 * a0, units::Force<double>(2));
    ASSERT_EQ(l0 / t0, units::Velocity<double>(2));
    ASSERT_EQ(s0.value(), 16);
}


} // namespace unitsTest
} // namespace sofa