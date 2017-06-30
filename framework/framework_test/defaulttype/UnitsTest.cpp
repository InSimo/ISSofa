
#include <gtest/gtest.h>
#include <sofa/defaulttype/Units.h>
#include <iostream>
#include <sofa/core/objectmodel/Data.h>


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


    EXPECT_EQ(m0 * a0, units::Force<double>(2));
    EXPECT_EQ(l0 / t0, units::Velocity<double>(2));
    EXPECT_EQ(s0.value(), 16);
}

TEST(UnitsTest, checkDataUnits)
{
    Data<units::Length<double>> d_length("Length");

    d_length.setValue(units::Length<double>(2.0));
    EXPECT_EQ(d_length.getValue().value(), 2.0);

    d_length.beginEdit()->value() = 5.0;
    d_length.endEdit();
    EXPECT_EQ(d_length.getValue().value(), 5.0);
}

} // namespace unitsTest
} // namespace sofa
