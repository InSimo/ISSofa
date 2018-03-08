
#include <gtest/gtest.h>
#include <sofa/defaulttype/Units.h>
#include <iostream>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>

#include <sofa/defaulttype/QuantityTypeInfo.h>

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
    EXPECT_EQ(m0 + m0, units::Mass<double>(2));
}

TEST(UnitsTest, checkDataUnits)
{
    Data<units::Length<double>> d_length("Length");

    d_length.setValue(units::Length<double>(2.0));
    EXPECT_EQ(d_length.getValue().value(), 2.0);

    d_length.beginEdit()->value() = 5.0;
    d_length.endEdit();
    EXPECT_EQ(d_length.getValue().value(), 5.0);

    //////////////////////////
    // tests on the TypeInfo
    ASSERT_EQ(std::string(defaulttype::DataTypeName<units::Length<double>>::name()), std::string("double"));

    const defaulttype::AbstractTypeInfo* typeInfo = d_length.getValueTypeInfo();
    EXPECT_TRUE(typeInfo->IsSingleValue());
    EXPECT_TRUE(typeInfo->IsMultiValue());
    EXPECT_FALSE(typeInfo->IsContainer());

    const defaulttype::AbstractMultiValueTypeInfo* typeMVInfo = typeInfo->MultiValueType();
    EXPECT_TRUE(typeMVInfo->FixedFinalSize());
    EXPECT_TRUE(typeMVInfo->getFinalValueType()->FixedFinalSize());
    EXPECT_EQ(1u, typeMVInfo->FinalSize());
    EXPECT_EQ(1u, typeMVInfo->finalSize(d_length.getValueVoidPtr()));


    //////////////////////////
    // tests on the TypeInfo::singleValueAPI
    std::string value;
    defaulttype::DataTypeInfo<units::Length<double> >::getDataValueString(d_length, value);
    EXPECT_EQ(value, "5");
    double value1;
    defaulttype::DataTypeInfo<units::Length<double> >::getDataValue(d_length.getValue(), value1);
    EXPECT_EQ(value1, 5);

    defaulttype::DataTypeInfo<units::Length<double> >::setDataValue(*d_length.beginEdit(), 9);
    d_length.endEdit();
    double value2;
    defaulttype::DataTypeInfo<units::Length<double> >::getDataValue(d_length.getValue(), value2);
    EXPECT_EQ(value2, 9);
    
    defaulttype::DataTypeInfo<units::Length<double> >::setDataValueString(*d_length.beginEdit(), "19");
    d_length.endEdit();
    double value3;
    defaulttype::DataTypeInfo<units::Length<double> >::getDataValue(d_length.getValue(), value3);
    EXPECT_EQ(value3, 19);

    defaulttype::DataTypeInfo<units::Length<double> >::resetValue(*d_length.beginEdit());
    d_length.endEdit();
    EXPECT_EQ(d_length.getValue().value(), double());


    //////////////////////////
    // tests on the TypeInfo::multiValueAPI
    double value4;
    defaulttype::DataTypeInfo<units::Length<double> >::getFinalValue(d_length.getValue(), 0, value4);
    EXPECT_EQ(value4, 0);
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

    //////////////////////////
    // tests on the TypeInfo
    ASSERT_STREQ(defaulttype::DataTypeName<units::Length<Coord>>::name(), "Vec3d");

    const defaulttype::AbstractTypeInfo* typeInfo = d_position.getValueTypeInfo();
    EXPECT_FALSE(typeInfo->IsSingleValue());
    EXPECT_TRUE(typeInfo->IsMultiValue());
    EXPECT_TRUE(typeInfo->IsContainer());

    const defaulttype::AbstractContainerTypeInfo* containerTypeInfo = typeInfo->ContainerType();
    EXPECT_EQ(3u, containerTypeInfo->containerSize(d_position.getValueVoidPtr()));
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
