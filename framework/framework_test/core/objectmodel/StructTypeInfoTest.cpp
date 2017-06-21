
#include <gtest/gtest.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/StructTypeInfo.h>
#include <sofa/helper/vector.h>
#include <sofa/helper/set.h>
#include <iostream>

namespace sofa
{

using namespace defaulttype;

template <typename _StructType>
struct DataStructTypeInfoTest: public ::testing::Test
{
    using StructType = _StructType;
};


//////////////////////////
/// Structures to test ///
//////////////////////////
namespace test_struct
{
struct EmptyStruct
{
    SOFA_STRUCT_DECL(EmptyStruct);
    SOFA_STRUCT_STREAM_METHODS(EmptyStruct)
};
struct SimpleStruct
{
    int myInt = 10;
    float myFloat = -0.1f;
    unsigned char myUChar = 'c';
    bool myBool = true;
    SOFA_STRUCT_DECL(SimpleStruct, myInt, myFloat, myUChar, myBool);
    SOFA_STRUCT_STREAM_METHODS(SimpleStruct)

    bool operator==(const SimpleStruct& rhs) const
    {
        return (myInt == rhs.myInt && myFloat == rhs.myFloat && myUChar == rhs.myUChar && myBool == rhs.myBool);
    }
};
struct NestedStruct
{
    SimpleStruct mySimpleStruct;
    SOFA_STRUCT_DECL(NestedStruct, mySimpleStruct);
    SOFA_STRUCT_STREAM_METHODS(NestedStruct)
};
struct ContainerStruct
{
    helper::vector<int> myIntVector = { 1,2,3 };
    helper::set<float> myFloatSet = std::set<float>({ 9,8,7 });
    SOFA_STRUCT_DECL(ContainerStruct, myIntVector, myFloatSet);
    SOFA_STRUCT_STREAM_METHODS(ContainerStruct)
};

template<typename T1, typename T2>
struct TemplatedStruct
{
    T1 myMemberT1;
    T2 myMemberT2;
    using TemplatedStruct_t = TemplatedStruct<T1, T2>;
    SOFA_STRUCT_DECL(TemplatedStruct_t, myMemberT1, myMemberT2);
    SOFA_STRUCT_STREAM_METHODS(TemplatedStruct_t)
};

/*struct PointerStruct
{
    double *myDoublePointer;
    SOFA_STRUCT_DECL(PointerStruct, myDoublePointer);
    SOFA_STRUCT_STREAM_METHODS(PointerStruct)
};*/
}

namespace defaulttype
{
    template<> struct DataTypeInfo<test_struct::EmptyStruct> : public StructTypeInfo<test_struct::EmptyStruct> {};
    template<> struct DataTypeInfo<test_struct::SimpleStruct> : public StructTypeInfo<test_struct::SimpleStruct> {};
    template<> struct DataTypeInfo<test_struct::NestedStruct> : public StructTypeInfo<test_struct::NestedStruct> {};
    template<> struct DataTypeInfo<test_struct::ContainerStruct> : public StructTypeInfo<test_struct::ContainerStruct> {};
    template<> struct DataTypeInfo<test_struct::TemplatedStruct<int, test_struct::SimpleStruct>> : public StructTypeInfo<test_struct::TemplatedStruct<int, test_struct::SimpleStruct>> {};
//    template<> struct DataTypeInfo<test_struct::PointerStruct> : public StructTypeInfo<test_struct::PointerStruct> {};
}

using StructTypes = testing::Types<
    test_struct::EmptyStruct,
    test_struct::SimpleStruct,
    test_struct::NestedStruct,
    test_struct::ContainerStruct,
    test_struct::TemplatedStruct<int, test_struct::SimpleStruct>//,
  //  test_struct::PointerStruct
>;

TYPED_TEST_CASE(DataStructTypeInfoTest, StructTypes);


/////////////
/// Tests ///
/////////////


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

TYPED_TEST(DataStructTypeInfoTest, checkStructTypeInfoIsOk)
{
    using StructType = TypeParam;
    Data<StructType> data("Struct");
    //BaseData* baseData = &data;
    
    std::cout << "STATIC TYPEINFO" << std::endl;
    std::cout << "struct " << DataTypeInfo<StructType>::name() << " { ";
    DataTypeInfo<StructType>::for_each(PrintName{}, PrintLastName{});
    std::cout << " };" << std::endl;
    
    std::cout << "DYNAMIC TYPEINFO" << std::endl;
    std::cout << "struct " << DataTypeInfo<StructType>::name() << " { ";
    DataTypeInfo<StructType>::for_each(data.getValue(), PrintValue{}, PrintLastValue{});
    std::cout << " };" << std::endl;
}

TYPED_TEST(DataStructTypeInfoTest, checkAbstractTypeInfoIsOk)
{
    using StructType = TypeParam;
    Data<StructType> data("Struct");
    sofa::core::objectmodel::BaseData* baseData = &data;
    
    const AbstractTypeInfo* typeInfo = baseData->getValueTypeInfo();
    SOFA_UNUSED(typeInfo);
    //EXPECT_TRUE(typeInfo->isStruct());
}

struct ExpectCleared
{
    template <typename MemberType>
    void operator()(MemberType&& mt, const typename MemberType::type& t) const
    {
        // TODO: need to call default constructor if available in reset
        //EXPECT_EQ(t, MemberType::type());
    }
};

TYPED_TEST(DataStructTypeInfoTest, checkStructTypeInfoResetValueIsOk)
{
    using StructType = TypeParam;
    Data<StructType> data("Struct");
    //BaseData* baseData = &data;
    DataTypeInfo<StructType>::resetValue(*data.beginEdit());
    data.endEdit();
    
    DataTypeInfo<StructType>::for_each(data.getValue(), ExpectCleared{});
    
    // DEBUG (warning : printing '\0' characters will end the output of std::cout)
    //std::cout << "RESET : struct " << DataTypeInfo<StructType>::name() << " { ";
    //DataTypeInfo<StructType>::for_each(data.getValue(), PrintValue{}, PrintLastValue{});
    //std::cout << " };" << std::endl;
}

}
