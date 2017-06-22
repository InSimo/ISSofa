
#include <gtest/gtest.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/EnumTypeInfo.h>
#include <iostream>

namespace sofa
{

namespace defaulttype
{



enum class uIntEnum2 : unsigned int
{
    un = 1u,
    dix = 10u,
    cent = 100u
};

enum charEnum2 : char
{
    aa = 'a',
    bb = 'b',
    cc = 'c',
    dd = 'd'
};

enum unscopedEnum : unsigned int
{
    uns,
    sco,
    ped
};


SOFA_ENUM(uIntEnum2, un, dix, cent)

SOFA_ENUM(charEnum2, aa, bb, cc, dd)

SOFA_ENUM(unscopedEnum, uns, sco, ped)

} // namespace defaulttype

} // namespace sofa


/////////////
/// Tests ///
/////////////

namespace sofa
{

namespace enumTypeInfoTest
{

template <typename _EnumType>
struct DataEnumTypeInfoTest : public ::testing::Test
{
    using EnumType = _EnumType;
};


using EnumTypes = testing::Types<
    defaulttype::uIntEnum2,
    defaulttype::charEnum2,
    defaulttype::unscopedEnum
>;

TYPED_TEST_CASE(DataEnumTypeInfoTest, EnumTypes);



TYPED_TEST(DataEnumTypeInfoTest, checkEnumTypeInfo)
{
    using EnumType = TypeParam;
    Data<EnumType> dataTest("Enum");

    ASSERT_TRUE(defaulttype::DataTypeInfo<EnumType>::FixedFinalSize);

    //std::cout << "DEBUG L" << __LINE__ << " : " << defaulttype::DataTypeInfo<EnumType>::FixedFinalSize << std::endl;
    //std::cout << "DEBUG L" << __LINE__ << " : " << defaulttype::DataTypeInfo<EnumType>::enumSize() << std::endl;
    //std::cout << "DEBUG L" << __LINE__ << " : " << defaulttype::DataTypeInfo<EnumType>::getEnumeratorName<0>() << std::endl;
    //std::cout << "DEBUG L" << __LINE__ << " : " << defaulttype::DataTypeInfo<EnumType>::getValue<1>() << std::endl;
    //std::cout << "DEBUG L" << __LINE__ << " : " << defaulttype::DataTypeInfo<EnumType>::getEnumerator<2>() << std::endl;

}

TEST(DataEnumTypeInfoTest2, checkuIntEnum2)
{
    typedef defaulttype::uIntEnum2 uIntEnum2;
    Data<uIntEnum2> dataTest("Enum");
    dataTest.setValue(uIntEnum2::dix);
    
    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::enumSize(), 3);
    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::getEnumeratorName<0>(), "un");
    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::getEnumeratorValue<1>(), 10u);
    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::getEnumerator<2>(), uIntEnum2::cent);

    helper::WriteAccessor<Data<uIntEnum2> > dataTestW = dataTest;
    defaulttype::DataTypeInfo<uIntEnum2>::resetValue(dataTestW);
    ASSERT_EQ(dataTest, uIntEnum2::un);   // for now, the reset change the data to the first value of the enum

    std::ostringstream oVal;
    oVal << dataTest;
    ASSERT_EQ(oVal.str(), "1");


    std::istringstream iVal("dix");
    iVal >> dataTestW;
    std::string value2;
    defaulttype::DataTypeInfo<uIntEnum2>::getDataValueString(dataTest, value2);
    ASSERT_EQ(value2, "10");

}

TEST(DataEnumTypeInfoTest2, checkcharEnum2)
{
    typedef defaulttype::charEnum2 charEnum2;
    Data<charEnum2> dataTest("Enum");
    dataTest.setValue(charEnum2::bb);

    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::enumSize(), 4);
    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::getEnumeratorName<0>(), "aa");
    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::getEnumeratorValue<1>(), 'b');
    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::getEnumerator<2>(), charEnum2::cc);

    std::string value;
    defaulttype::DataTypeInfo<charEnum2>::getDataValueString(dataTest, value);
    ASSERT_EQ(value, "b");

    std::string value1("dd");
    helper::WriteAccessor<Data<charEnum2> > dataTestW = dataTest;
    defaulttype::DataTypeInfo<charEnum2>::setDataValueString(dataTestW, value1);

    std::string value2;
    defaulttype::DataTypeInfo<charEnum2>::getDataValueString(dataTest, value2);
    ASSERT_EQ(value2, "d");


    std::string value3;
    defaulttype::DataTypeInfo<charEnum2>::getDataEnumeratorString(dataTest.getValue(), value3);
    ASSERT_EQ(value3, "dd");

    charEnum2 value4(charEnum2::aa);
    helper::WriteAccessor<Data<charEnum2> > dataTestW2 = dataTest;
    defaulttype::DataTypeInfo<charEnum2>::setDataValue(dataTestW2, value4);

    charEnum2 value5;
    defaulttype::DataTypeInfo<charEnum2>::getDataValue(dataTest.getValue(), value5);
    ASSERT_EQ(value5, charEnum2::aa);

}

TEST(DataEnumTypeInfoTest2, checkunscopedEnum)
{
    typedef defaulttype::unscopedEnum unscopedEnum;
    Data<unscopedEnum> dataTest("Enum");

    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::enumSize(), 3);
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumeratorName<0>(), "uns");
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumeratorValue<1>(), 1);
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumerator<2>(), unscopedEnum::ped);

}



}// namespace enumTypeInfoTest

} // namespace sofa
