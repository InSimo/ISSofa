
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

std::ostream& operator<< (std::ostream& stream, const uIntEnum2& myEnum)
{
    stream << static_cast<std::underlying_type<uIntEnum2>::type>(myEnum);
    return stream;
}

std::istream& operator>> (std::istream& stream, uIntEnum2& myEnum)
{
    return stream;
}

std::ostream& operator<< (std::ostream& stream, const charEnum2& myEnum)
{
    stream << static_cast<std::underlying_type<charEnum2>::type>(myEnum);
    return stream;
}

std::istream& operator >> (std::istream& stream, charEnum2& myEnum)
{
    return stream;
}

std::ostream& operator<< (std::ostream& stream, const unscopedEnum& myEnum)
{
    stream << static_cast<std::underlying_type<unscopedEnum>::type>(myEnum);
    return stream;
}

std::istream& operator >> (std::istream& stream, unscopedEnum& myEnum)
{
    return stream;
}

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
    
    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::FixedFinalSize, true);
    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::enumSize(), 3);
    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::getEnumeratorName<0>(), "un");
    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::getValue<1>(), 10u);
    ASSERT_EQ(defaulttype::DataTypeInfo<uIntEnum2>::getEnumerator<2>(), uIntEnum2::cent);
}

TEST(DataEnumTypeInfoTest2, checkcharEnum2)
{
    typedef defaulttype::charEnum2 charEnum2;
    Data<charEnum2> dataTest("Enum");

    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::FixedFinalSize, true);
    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::enumSize(), 4);
    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::getEnumeratorName<0>(), "aa");
    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::getValue<1>(), 'b');
    ASSERT_EQ(defaulttype::DataTypeInfo<charEnum2>::getEnumerator<2>(), charEnum2::cc);
}

TEST(DataEnumTypeInfoTest2, checkunscopedEnum)
{
    typedef defaulttype::unscopedEnum unscopedEnum;
    Data<unscopedEnum> dataTest("Enum");

    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::FixedFinalSize, true);
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::enumSize(), 3);
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumeratorName<0>(), "uns");
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getValue<1>(), 1);
    ASSERT_EQ(defaulttype::DataTypeInfo<unscopedEnum>::getEnumerator<2>(), unscopedEnum::ped);
}



}// namespace enumTypeInfoTest

} // namespace sofa
