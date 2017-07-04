
#include <gtest/gtest.h>
#include <sofa/defaulttype/Units.h>
#include <iostream>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>


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
    EXPECT_EQ(s0.unitsAsText(), "kg.s-2");

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

TEST(UnitsTest, checkDataUnitsOnCoord)
{
    typedef defaulttype::Vec3dTypes::Coord Coord;
    Coord initPosition{ 1.0, 2.0, 3.0 };
    units::Length<Coord> position(initPosition);
    EXPECT_EQ(position.value(), initPosition);
    EXPECT_EQ(position.unitsAsText(), "m");

    Data<units::Length<Coord> > d_position("Position");

    d_position.setValue(units::Length<Coord>(initPosition));
    EXPECT_EQ(d_position.getValue().value(), initPosition);

    Coord position2{ 0.0, 1.0, 2.0 };
    d_position.beginEdit()->value() = position2;
    d_position.endEdit();
    EXPECT_EQ(d_position.getValue().value(), position2);
}


TEST(UnitsTest, testOnCommonUsage)
{
    typedef defaulttype::Vec3dTypes::Coord Coord;
    typedef defaulttype::Vec3dTypes::Deriv Deriv;

    // usage for a spring F = k*dx
    units::Force<Deriv>         F;
    units::Length<Coord>        dx(Coord(2,3,4));
    units::Stiffness<double>    k(5);

    F = k * dx;

    units::Force<Deriv> F_expected(Deriv(10, 15, 20 ));
    EXPECT_EQ(F_expected, F);
    EXPECT_EQ(F.unitsAsText(), "kg.m.s-2");
}

} // namespace unitsTest
} // namespace sofa
