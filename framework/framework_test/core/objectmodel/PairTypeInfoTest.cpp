
#include <gtest/gtest.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/PairTypeInfo.h>

#include <sofa/helper/vector.h>
#include <sofa/helper/map.h>

namespace sofa
{

template <typename _PairType>
struct DataPairTypeInfoTest : public ::testing::Test
{
    using PairType = _PairType;
};

using PairTypes = testing::Types<
    sofa::helper::pair<int, int>,
    sofa::helper::pair<int, sofa::helper::pair<double, bool>>,
    sofa::helper::pair<sofa::helper::vector<sofa::helper::pair<double, long long>>, 
    std::map<sofa::helper::pair<int, short>, sofa::helper::pair<double, sofa::helper::vector<sofa::helper::pair<double, float>>>>>
>;
TYPED_TEST_CASE(DataPairTypeInfoTest, PairTypes);

/////////////
/// Tests ///
/////////////

using namespace sofa::defaulttype;

struct PrintName
{
    template <typename MemberType>
    void operator()(MemberType&& mt) const
    {
        using DataType = typename MemberType::type;
        std::cout << DataTypeName<DataType>::name() << " " << MemberType::name() << ", ";
    }
};
struct PrintLastName
{
    template <typename MemberType>
    void operator()(MemberType&& mt) const
    {
        using DataType = typename MemberType::type;
        std::cout << DataTypeName<DataType>::name() << " " << MemberType::name();
    }
};
struct PrintValue
{
    template <typename MemberType>
    void operator()(MemberType&& mt, const typename MemberType::type& t) const
    {
        std::cout << t << ", ";
    }
};
struct PrintLastValue
{
    template <typename MemberType>
    void operator()(MemberType&& mt, const typename MemberType::type& t) const
    {
        std::cout << t;
    }
};

TYPED_TEST(DataPairTypeInfoTest, checkPairTypeInfoIsOk)
{
    using PairType = TypeParam;
    Data<PairType> data("Pair");

    std::cout << "STATIC TYPEINFO" << std::endl;
    std::cout << "pair " << DataTypeInfo<PairType>::name() << " { ";
    DataTypeInfo<PairType>::for_each(PrintName{}, PrintLastName{});
    std::cout << " };" << std::endl;

    std::cout << "DYNAMIC TYPEINFO" << std::endl;
    std::cout << "pair " << DataTypeInfo<PairType>::name() << " { ";
    DataTypeInfo<PairType>::for_each(data.getValue(), PrintValue{}, PrintLastValue{});
    std::cout << " };" << std::endl;
}

TYPED_TEST(DataPairTypeInfoTest, checkAbstractTypeInfoIsOk)
{
    using PairType = TypeParam;
    Data<PairType> data("Pair");
    sofa::core::objectmodel::BaseData* baseData = &data;

    const AbstractTypeInfo* typeInfo = baseData->getValueTypeInfo();

    ASSERT_TRUE(typeInfo->ValidInfo());
    ASSERT_FALSE(typeInfo->IsSingleValue());
    ASSERT_FALSE(typeInfo->IsContainer());
    ASSERT_TRUE(typeInfo->IsStructure());
    ASSERT_EQ(2u, typeInfo->StructureType()->structSize());
}

}
