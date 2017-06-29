
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
    SOFA_STRUCT_DECL(EmptyStruct, SOFA_EMPTY);
    SOFA_STRUCT_STREAM_METHODS(EmptyStruct);

    bool operator==(const EmptyStruct& rhs) const
    {
        return true;
    }
};
struct SimpleStruct
{
    int myInt = 10;
    float myFloat = -0.1f;
    unsigned char myUChar = 'c';
    bool myBool = true;
    SOFA_STRUCT_DECL(SimpleStruct, myInt, myFloat, myUChar, myBool);
    SOFA_STRUCT_STREAM_METHODS(SimpleStruct);
    SOFA_STRUCT_COMPARE_METHOD(SimpleStruct, myInt, myFloat, myUChar, myBool);
};
} // namespace test_struct
} // namespace sofa

SOFA_STRUCT_DEFINE(sofa::test_struct::SimpleStruct);

namespace sofa
{
namespace test_struct
{
struct NestedStruct
{
    SimpleStruct mySimpleStruct;
    SOFA_STRUCT_DECL(NestedStruct, mySimpleStruct);
    SOFA_STRUCT_STREAM_METHODS(NestedStruct);
    SOFA_STRUCT_COMPARE_METHOD(NestedStruct, mySimpleStruct);
};
struct ContainerStruct
{
    helper::vector<int> myIntVector = { 1,2,3 };
    helper::set<float> myFloatSet = std::set<float>({ 9,8,7 });
    helper::vector<SimpleStruct> myStructVector;
    SOFA_STRUCT_DECL(ContainerStruct, myIntVector, myFloatSet, myStructVector);
    SOFA_STRUCT_STREAM_METHODS(ContainerStruct);
    SOFA_STRUCT_COMPARE_METHOD(ContainerStruct, myIntVector, myFloatSet, myStructVector);
};

template<typename T1, typename T2>
struct TemplatedStruct
{
    T1 myMemberT1;
    T2 myMemberT2;
    using TemplatedStruct_t = TemplatedStruct<T1, T2>;
    SOFA_STRUCT_DECL(TemplatedStruct_t, myMemberT1, myMemberT2);
    SOFA_STRUCT_STREAM_METHODS(TemplatedStruct_t);
    SOFA_STRUCT_COMPARE_METHOD(TemplatedStruct_t, myMemberT1, myMemberT2);
};

struct NoDefaultConstrStruct
{
    int myInt;

    SOFA_STRUCT_DECL(NoDefaultConstrStruct, myInt);
    SOFA_STRUCT_STREAM_METHODS(NoDefaultConstrStruct);
    SOFA_STRUCT_COMPARE_METHOD(NoDefaultConstrStruct, myInt);

    NoDefaultConstrStruct(int value) : myInt(value) {}
};

/*struct PointerStruct
{
    double *myDoublePointer;
    SOFA_STRUCT_DECL(PointerStruct, myDoublePointer);
    SOFA_STRUCT_STREAM_METHODS(PointerStruct)
};*/
} // namespace test_struct
} // namespace sofa

SOFA_STRUCT_DEFINE(sofa::test_struct::EmptyStruct);
SOFA_STRUCT_DEFINE(sofa::test_struct::NestedStruct);
SOFA_STRUCT_DEFINE(sofa::test_struct::ContainerStruct);
SOFA_STRUCT_DEFINE(sofa::test_struct::TemplatedStruct<int, test_struct::SimpleStruct>);
//SOFA_STRUCT_DEFINE(sofa::test_struct::PointerStruct);

namespace sofa
{

namespace test_struct_2
{
using StructTypes = testing::Types<
    test_struct::EmptyStruct,
    test_struct::SimpleStruct,
    test_struct::NestedStruct,
    test_struct::ContainerStruct,
    test_struct::TemplatedStruct<int, test_struct::SimpleStruct>//,
//    test_struct::PointerStruct
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
    
    ASSERT_TRUE(DataTypeInfo<StructType>::ValidInfo);
    
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
    
    ASSERT_TRUE(typeInfo->ValidInfo());
    ASSERT_FALSE(typeInfo->IsSingleValue());
    ASSERT_FALSE(typeInfo->IsContainer());
    ASSERT_TRUE(typeInfo->IsStructure());
    ASSERT_EQ(std::tuple_size<typename StructType::MembersTuple>::value, typeInfo->StructureType()->structSize());
    
    
    Data<StructType> data2("Struct");
    Data<int> dataInt("Int");
    Data<helper::vector<float>> dataVecFloat("VecFloat");
    
    ASSERT_EQ(typeInfo->typeInfoID(), data2.getValueTypeInfo()->typeInfoID());
    ASSERT_NE(typeInfo->typeInfoID(), dataInt.getValueTypeInfo()->typeInfoID());
    ASSERT_NE(typeInfo->typeInfoID(), dataVecFloat.getValueTypeInfo()->typeInfoID());
}

// Test reset for all types that have default constructor
TYPED_TEST(DataStructTypeInfoTest, checkStructTypeInfoResetValueIsOk)
{
    using StructType = TypeParam;
    Data<StructType> data("Struct");
    DataTypeInfo<StructType>::resetValue(*data.beginEdit());
    data.endEdit();
    EXPECT_EQ(data.getValue(), StructType());
    
    // DEBUG (warning : printing '\0' characters will end the output of std::cout)
    //std::cout << "RESET : struct " << DataTypeInfo<StructType>::name() << " { ";
    //DataTypeInfo<StructType>::for_each(data.getValue(), PrintValue{}, PrintLastValue{});
    //std::cout << " };" << std::endl;
}

struct ExpectCleared
{
    template <typename MemberType>
    void operator()(MemberType&& mt, const typename MemberType::type& t) const
    {
        typename MemberType::type memberCompare;
        DataTypeInfo<typename MemberType::type>::resetValue(memberCompare);
        EXPECT_EQ(t, memberCompare);
    }
};

TEST(DataStructTypeInfoTest2, checkNoDefaultConstrStruct_ResetValueIsOk)
{
    test_struct::NoDefaultConstrStruct testValue(10);
    StructTypeInfo<test_struct::NoDefaultConstrStruct>::resetValue(testValue);
    StructTypeInfo<test_struct::NoDefaultConstrStruct>::for_each(testValue, ExpectCleared{});
}

TEST(DataStructTypeInfoTest2, checkAbstractTypeInfoSimpleStruct)
{
    Data<test_struct::SimpleStruct> data("SimpleStruct");
    sofa::core::objectmodel::BaseData* baseData = &data;
    
    const AbstractTypeInfo* typeInfo = baseData->getValueTypeInfo();
    const AbstractStructureTypeInfo* structureInfo = typeInfo->StructureType();
    
    const AbstractTypeInfo* typeInfoM0 = structureInfo->getMemberTypeForIndex(0);
    EXPECT_TRUE(typeInfoM0->IsSingleValue());
    EXPECT_TRUE(typeInfoM0->SingleValueType()->Integer());
    
    const void* M0cptr = structureInfo->getMemberValue(data.getValueVoidPtr(), 0);
    EXPECT_EQ(typeInfoM0->byteSize(M0cptr), sizeof(int));
    EXPECT_EQ(typeInfoM0->getValuePtr(M0cptr), &(data.getValue().myInt));
    
    EXPECT_EQ(*(const int*)M0cptr, 10);
    EXPECT_EQ(structureInfo->getMemberName(data.getValueVoidPtr(), 0), "myInt");
    
    *(int*)structureInfo->editMemberValue(data.beginEditVoidPtr(), 0) = 42;
    data.endEditVoidPtr();
    EXPECT_EQ(*(const int*)M0cptr, 42);
}

TEST(DataStructTypeInfoTest2, checkSimpleCopy)
{
    {
        Data<test_struct::EmptyStruct> data("EmptyStruct");
        EXPECT_TRUE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 0);
    }
    {
        Data<test_struct::SimpleStruct> data("SimpleStruct");
        EXPECT_TRUE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 12);
    }
    {
        Data<test_struct::NestedStruct> data("NestedStruct");
        EXPECT_TRUE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 12);
    }
    {
        Data<test_struct::ContainerStruct> data("ContainerStruct");
        EXPECT_FALSE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 0);
    }
    {
        Data<test_struct::TemplatedStruct<int, test_struct::SimpleStruct>> data("TemplatedStruct<int, SimpleStruct>");
        EXPECT_TRUE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 16);
    }
}

TEST(DataStructTypeInfoTest2, checkFixedFinalSize)
{
    EXPECT_TRUE(DataTypeInfo<test_struct::EmptyStruct>::FixedFinalSize);
    EXPECT_TRUE(DataTypeInfo<test_struct::SimpleStruct>::FixedFinalSize);
    EXPECT_TRUE(DataTypeInfo<test_struct::NestedStruct>::FixedFinalSize);
    EXPECT_FALSE(DataTypeInfo<test_struct::ContainerStruct>::FixedFinalSize);
    EXPECT_TRUE((DataTypeInfo<test_struct::TemplatedStruct<int, test_struct::SimpleStruct>>::FixedFinalSize));
}

TEST(DataStructTypeInfoTest2, ostreamTest)
{
    test_struct::TemplatedStruct<int, test_struct::SimpleStruct> testValue{};

    std::ostringstream stringStream;
    stringStream << testValue;
    EXPECT_EQ("{ 0; { 10; -0.1; c; 1 } }", stringStream.str());
}

TEST(DataStructTypeInfoTest2, istreamTest)
{
    test_struct::TemplatedStruct<int, test_struct::SimpleStruct> testValue{};

    std::istringstream stringStream("{ 0 ; { 15 ; -0.1 ; f ; 1 } }");
    stringStream >> testValue;
    test_struct::TemplatedStruct<int, test_struct::SimpleStruct> compareValue{};
    compareValue.myMemberT2.myInt = 15;
    compareValue.myMemberT2.myUChar = 'f';
    EXPECT_EQ(testValue, compareValue);
}

} // namespace test_struct_2
} // namespace sofa
