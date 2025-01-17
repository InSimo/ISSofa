
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

enum  TestFlag
{
    NONE = 0,
    ACTIVE = 1 << 0,
    LOWER    = 1 << 1,
    HIDDEN    = 1 << 2,
};
SOFA_ENUM_DECL(TestFlag, ACTIVE, LOWER, HIDDEN);
SOFA_ENUM_STREAM_METHODS_FLAG(TestFlag);
SOFA_ENUM_FLAG_OPERATORS(TestFlag);



class ContainingClass
{
public:
enum class enumInClass : int
{
    eic1 = 10,
    eic2 = 100
};

SOFA_ENUM_DECL_IN_CLASS(enumInClass, eic1, eic2);
SOFA_ENUM_STREAM_METHODS_IN_CLASS(enumInClass);

enum unscopedEnumInClass : int
{
    ueic1 = 10,
    ueic2 = 100
};

SOFA_ENUM_DECL_IN_CLASS(unscopedEnumInClass, ueic1, ueic2);
SOFA_ENUM_STREAM_METHODS_NAME_IN_CLASS(unscopedEnumInClass);

enum  TestFlagInClass : unsigned int
{
    NONE = 0,
    ACTIVE = 1 << 0,
    LOWER    = 1 << 1,
    HIDDEN    = 1 << 2,
};
SOFA_ENUM_DECL_IN_CLASS(TestFlagInClass, ACTIVE, LOWER, HIDDEN);
SOFA_ENUM_STREAM_METHODS_FLAG_IN_CLASS(TestFlagInClass);
SOFA_ENUM_FLAG_OPERATORS_IN_CLASS(TestFlagInClass);

};





} // namespace enumTypeInfoTest
} // namespace sofa

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
    unscopedEnum,
    ContainingClass::enumInClass,
    ContainingClass::unscopedEnumInClass
>;

TYPED_TEST_CASE(DataEnumTypeInfoTest, EnumTypes);



TYPED_TEST(DataEnumTypeInfoTest, checkEnumTypeInfo)
{
    using EnumType = TypeParam;
    Data<EnumType> dataTest("Enum");

    ASSERT_EQ(defaulttype::DataTypeInfo<EnumType>::byteSize(dataTest.getValue()), sizeof(typename std::underlying_type<EnumType>::type));
}

TYPED_TEST(DataEnumTypeInfoTest, checkEnumAbstractTypeInfo)
{
    using EnumType = TypeParam;
    Data<EnumType> dataTest("Enum");
    const defaulttype::AbstractTypeInfo* typeInfo = dataTest.getValueTypeInfo();

    ASSERT_TRUE(typeInfo->IsSingleValue());
    ASSERT_TRUE(typeInfo->SingleValueType()->FixedFinalSize());
}


template <typename _EnumFlagType>
struct DataEnumTypeInfoFlagTest : public ::testing::Test
{
    using EnumFlagType = _EnumFlagType;
};

using EnumFlagTypes = testing::Types<
    TestFlag,
    ContainingClass::TestFlagInClass
>;

TYPED_TEST_CASE(DataEnumTypeInfoFlagTest, EnumFlagTypes);


TYPED_TEST(DataEnumTypeInfoFlagTest, checkFlagOperatorAndStream)
{
    using EnumFlagTypes = TypeParam;
    EnumFlagTypes value1 = EnumFlagTypes::ACTIVE | EnumFlagTypes::LOWER;
    EnumFlagTypes value2 = EnumFlagTypes::HIDDEN;
    EnumFlagTypes value3 = EnumFlagTypes::LOWER | EnumFlagTypes::HIDDEN;

    ASSERT_EQ((value1 | value2), (EnumFlagTypes::ACTIVE | EnumFlagTypes::LOWER | EnumFlagTypes::HIDDEN));

    std::string valuestr;
    defaulttype::DataTypeInfo<EnumFlagTypes>::getDataValueString(value1 | value2, valuestr);
    ASSERT_EQ(valuestr, "HIDDEN LOWER ACTIVE");

    ASSERT_EQ((value1 & value2), (EnumFlagTypes::NONE));
    defaulttype::DataTypeInfo<EnumFlagTypes>::getDataValueString(value1 & value2, valuestr);
    ASSERT_EQ(valuestr, "");

    ASSERT_EQ((value2 & value3), (EnumFlagTypes::HIDDEN));
    defaulttype::DataTypeInfo<EnumFlagTypes>::getDataValueString(value2 & value3, valuestr);
    ASSERT_EQ(valuestr, "HIDDEN");

    ASSERT_EQ((value1 | value3), (EnumFlagTypes::ACTIVE | EnumFlagTypes::LOWER | EnumFlagTypes::HIDDEN));
    defaulttype::DataTypeInfo<EnumFlagTypes>::getDataValueString(value1 | value3, valuestr);
    ASSERT_EQ(valuestr, "HIDDEN LOWER ACTIVE");
}


TEST(DataEnumTypeInfoTest2, checkuIntEnum2)
{
    typedef uIntEnum2 uIntEnum2;
    Data<uIntEnum2> dataTest("Enum");
    dataTest.setValue(uIntEnum2::dix);
    
    //ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::name(), "");

    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::enumSize(), 3u);
    ASSERT_EQ(std::string(defaulttype::DataTypeInfo<uIntEnum2>::getEnumeratorName<0>()), std::string("un"));
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

    std::istringstream iVal2("100");
    iVal2 >> dataTestW;
    defaulttype::DataTypeInfo<uIntEnum2>::getDataValueString(dataTest, value2);
    ASSERT_EQ(value2, "100");

}

TEST(DataEnumTypeInfoTest2, checkcharEnum2)
{
    typedef charEnum2 charEnum2;
    Data<charEnum2> dataTest("Enum");
    dataTest.setValue(charEnum2::bb);

    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::enumSize(), 4u);
    ASSERT_EQ(std::string(defaulttype::DataTypeInfo<charEnum2>::getEnumeratorName<0>()), std::string("aa"));
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

    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::enumSize(), 3u);
    ASSERT_EQ(std::string(defaulttype::DataTypeInfo<unscopedEnum>::getEnumeratorName<0>()), std::string("uns"));
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumeratorValue<1>(), 1u);
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumerator<2>(), unscopedEnum::ped);

}


TEST(DataEnumTypeInfoTest2, checkAbstractTypeInfoEnum)
{
    typedef uIntEnum2 uIntEnum2;
    Data<uIntEnum2> dataTest("Enum");
    dataTest.setValue(uIntEnum2::dix);

    sofa::core::objectmodel::BaseData* baseData = &dataTest;

    const defaulttype::AbstractTypeInfo* typeInfo = baseData->getValueTypeInfo();

    EXPECT_TRUE(typeInfo->IsSingleValue());
    EXPECT_TRUE(typeInfo->IsMultiValue());
    EXPECT_TRUE(typeInfo->IsEnum());
    EXPECT_FALSE(typeInfo->IsContainer());
    EXPECT_FALSE(typeInfo->IsStructure());


    std::string value;
    typeInfo->getDataValueString(dataTest.getValueVoidPtr(), value);
    ASSERT_EQ(value, "10");

    std::string value1("cent");
    typeInfo->setDataValueString(dataTest.beginEditVoidPtr(), value1);
    dataTest.endEditVoidPtr();

    std::string value2;
    typeInfo->getDataValueString(dataTest.getValueVoidPtr(), value2);
    ASSERT_EQ(value2, "100");

    const defaulttype::AbstractEnumTypeInfo* einfo = typeInfo->EnumType();

    std::string name = einfo->getDataEnumeratorString(dataTest.getValueVoidPtr());
    ASSERT_EQ(name, "cent");


    einfo->setDataEnumeratorString(dataTest.beginEditVoidPtr(),"dix");
    std::string value3;
    einfo->getDataValueString(dataTest.getValueVoidPtr(), value3);
    ASSERT_EQ(value3, "10");



    std::vector<std::string > enumNames;
    einfo->getAvailableItems(dataTest.getValueVoidPtr(), enumNames);
    ASSERT_EQ(enumNames.size(), 3u);
    ASSERT_EQ(enumNames[0], "cent");

    std::string value4;
    uIntEnum2 value5(uIntEnum2::un);
    einfo->setDataValueInteger(dataTest.beginEditVoidPtr(), static_cast<long long>(value5));
    dataTest.endEditVoidPtr();
    typeInfo->getDataValueString(dataTest.getValueVoidPtr(), value4);
    ASSERT_EQ(value4, "1");


    long long value6 = einfo->getDataValueInteger(dataTest.getValueVoidPtr());
    ASSERT_EQ(value6, static_cast<long long>(uIntEnum2::un));

}


TEST(DataEnumTypeInfoTest2, checkEnumIO)
{
    typedef ContainingClass::enumInClass Enum;
    Data<Enum> dataTest("Enum");
    dataTest.setValue(Enum::eic2);
    
    //ASSERT_EQ(defaulttype::DataTypeInfo<Enum>::name(), "");

    ASSERT_EQ(defaulttype::DataTypeInfo<Enum>::enumSize(), 2u);
    ASSERT_EQ(std::string(defaulttype::DataTypeInfo<Enum>::getEnumeratorName<0>()), std::string("eic1"));
    ASSERT_EQ(defaulttype::DataTypeInfo<Enum>::getEnumeratorValue<1>(), 100u);
    ASSERT_EQ(defaulttype::DataTypeInfo<Enum>::getEnumerator<1>(), Enum::eic2);

    helper::WriteAccessor<Data<Enum> > dataTestW = dataTest;
    defaulttype::DataTypeInfo<Enum>::resetValue(dataTestW);
    EXPECT_EQ(dataTest, Enum::eic1);   // for now, the reset change the data to the first value of the enum

    Data<Enum> dataTest2("Enum2");
    dataTest2.setValue(Enum::eic2);
    helper::WriteAccessor<Data<Enum> > dataTest2W = dataTest2;
    std::string value2;

    std::ostringstream oVal;
    oVal << dataTest << " " << dataTest2;
    EXPECT_EQ(oVal.str(), "10 100");

    std::istringstream iVal("100 10");
    iVal >> dataTestW >> dataTest2W;
    defaulttype::DataTypeInfo<Enum>::getDataEnumeratorString(dataTestW, value2);
    EXPECT_EQ(value2, "eic2");
    defaulttype::DataTypeInfo<Enum>::getDataEnumeratorString(dataTest2W, value2);
    EXPECT_EQ(value2, "eic1");

    std::istringstream iVal2("eic1 eic2");
    iVal2 >> dataTestW >> dataTest2W;
    defaulttype::DataTypeInfo<Enum>::getDataValueString(dataTest, value2);
    EXPECT_EQ(value2, "10");
    defaulttype::DataTypeInfo<Enum>::getDataValueString(dataTest2, value2);
    EXPECT_EQ(value2, "100");
}

TEST(DataEnumTypeInfoTest2, checkEnumIOName)
{
    typedef ContainingClass::unscopedEnumInClass Enum;
    Data<Enum> dataTest("Enum");
    dataTest.setValue(Enum::ueic2);
    
    //ASSERT_EQ(defaulttype::DataTypeInfo<Enum>::name(), "");

    ASSERT_EQ(defaulttype::DataTypeInfo<Enum>::enumSize(), 2u);
    ASSERT_EQ(std::string(defaulttype::DataTypeInfo<Enum>::getEnumeratorName<0>()), std::string("ueic1"));
    ASSERT_EQ(defaulttype::DataTypeInfo<Enum>::getEnumeratorValue<1>(), 100u);
    ASSERT_EQ(defaulttype::DataTypeInfo<Enum>::getEnumerator<1>(), Enum::ueic2);

    helper::WriteAccessor<Data<Enum> > dataTestW = dataTest;
    defaulttype::DataTypeInfo<Enum>::resetValue(dataTestW);
    EXPECT_EQ(dataTest, Enum::ueic1);   // for now, the reset change the data to the first value of the enum

    Data<Enum> dataTest2("Enum2");
    dataTest2.setValue(Enum::ueic2);
    helper::WriteAccessor<Data<Enum> > dataTest2W = dataTest2;
    std::string value2;

    std::ostringstream oVal;
    oVal << dataTest << " " << dataTest2;
    EXPECT_EQ(oVal.str(), "ueic1 ueic2");

    std::istringstream iVal("100 10");
    iVal >> dataTestW >> dataTest2W;
    defaulttype::DataTypeInfo<Enum>::getDataEnumeratorString(dataTestW, value2);
    EXPECT_EQ(value2, "ueic2");
    defaulttype::DataTypeInfo<Enum>::getDataEnumeratorString(dataTest2W, value2);
    EXPECT_EQ(value2, "ueic1");

    std::istringstream iVal2("ueic1 ueic2");
    iVal2 >> dataTestW >> dataTest2W;
    defaulttype::DataTypeInfo<Enum>::getDataValueString(dataTest, value2);
    EXPECT_EQ(value2, "ueic1");
    defaulttype::DataTypeInfo<Enum>::getDataValueString(dataTest2, value2);
    EXPECT_EQ(value2, "ueic2");
}

}// namespace enumTypeInfoTest
} // namespace sofa
