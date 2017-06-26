
#include <gtest/gtest.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/EnumTypeInfo.h>
#include <iostream>


/////////////
/// Tests ///
/////////////

namespace sofa
{

namespace enumTypeInfoTest
{


enum class uIntEnum2 : unsigned int
{
    un = 1u,
    dix = 10u,
    cent = 100u
};

SOFA_ENUM_DECL(uIntEnum2, un, dix, cent);
SOFA_ENUM_STREAM_METHODS(uIntEnum2);

enum charEnum2 : char
{
    aa = 'a',
    bb = 'b',
    cc = 'c',
    dd = 'd'
};

SOFA_ENUM_DECL(charEnum2, aa, bb, cc, dd);
SOFA_ENUM_STREAM_METHODS(charEnum2);

enum unscopedEnum : unsigned int
{
    uns,
    sco,
    ped
};

SOFA_ENUM_DECL(unscopedEnum, uns, sco, ped);
SOFA_ENUM_STREAM_METHODS(unscopedEnum);


} // namespace enumTypeInfoTest
} // namespace sofa




SOFA_ENUM_DEFINE(sofa::enumTypeInfoTest::uIntEnum2);
SOFA_ENUM_DEFINE(sofa::enumTypeInfoTest::charEnum2);
SOFA_ENUM_DEFINE(sofa::enumTypeInfoTest::unscopedEnum);





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
    uIntEnum2,
    charEnum2,
    unscopedEnum
>;

TYPED_TEST_CASE(DataEnumTypeInfoTest, EnumTypes);



TYPED_TEST(DataEnumTypeInfoTest, checkEnumTypeInfo)
{
    using EnumType = TypeParam;
    Data<EnumType> dataTest("Enum");

    ASSERT_TRUE(defaulttype::DataTypeInfo<EnumType>::FixedFinalSize);
    
}

TEST(DataEnumTypeInfoTest2, checkuIntEnum2)
{
    typedef uIntEnum2 uIntEnum2;
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
    typedef charEnum2 charEnum2;
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
    typedef unscopedEnum unscopedEnum;
    Data<unscopedEnum> dataTest("Enum");

    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::enumSize(), 3);
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumeratorName<0>(), "uns");
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumeratorValue<1>(), 1);
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumerator<2>(), unscopedEnum::ped);

}


TEST(DataEnumTypeInfoTest2, checkAbstractTypeInfoEnum)
{
    typedef uIntEnum2 uIntEnum2;
    Data<uIntEnum2> dataTest("Enum");
    dataTest.setValue(uIntEnum2::dix);

    sofa::core::objectmodel::BaseData* baseData = &dataTest;

    const defaulttype::AbstractTypeInfo* typeInfo = baseData->getValueTypeInfo();

    EXPECT_TRUE(typeInfo->SingleValueType());
    EXPECT_TRUE(typeInfo->MultiValueType());
    EXPECT_FALSE(typeInfo->ContainerType());
    EXPECT_FALSE(typeInfo->StructureType());


    std::string value;
    typeInfo->getDataValueString(dataTest.getValueVoidPtr(), value);
    ASSERT_EQ(value, "10");

    std::string value1("cent");
    typeInfo->setDataValueString(dataTest.beginEditVoidPtr(), value1);
    dataTest.endEditVoidPtr();

    std::string value2;
    typeInfo->getDataValueString(dataTest.getValueVoidPtr(), value2);
    ASSERT_EQ(value2, "100");



    const defaulttype::AbstractValueTypeInfo* enumTypeInfo = typeInfo->SingleValueType();

    std::vector<std::string > enumNames;
    enumTypeInfo->getAvailableItems(dataTest.getValueVoidPtr(), enumNames);
    ASSERT_EQ(enumNames.size(), 3);
    ASSERT_EQ(enumNames[0], "cent");

    std::string value3;
    uIntEnum2 value4(uIntEnum2::un);
    enumTypeInfo->setDataValueInteger(dataTest.beginEditVoidPtr(), static_cast<long long>(value4));
    dataTest.endEditVoidPtr();
    typeInfo->getDataValueString(dataTest.getValueVoidPtr(), value3);
    ASSERT_EQ(value3, "1");


    long long value5 = enumTypeInfo->getDataValueInteger(dataTest.getValueVoidPtr());
    ASSERT_EQ(value5, static_cast<long long>(uIntEnum2::un));

}


}// namespace enumTypeInfoTest
} // namespace sofa
