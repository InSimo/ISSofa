
#include <gtest/gtest.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/StructTypeInfo.h>
#include <sofa/defaulttype/PairTypeInfo.h>
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

    bool operator==(const EmptyStruct&) const
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
    SOFA_STRUCT_COMPARE_METHOD(SimpleStruct);
};
} // namespace test_struct
} // namespace sofa

SOFA_STRUCT_DEFINE_TYPEINFO(sofa::test_struct::SimpleStruct);

namespace sofa
{
namespace test_struct
{
struct PairStruct
{
    sofa::helper::pair<int, int> myIntPair = {1, 2};
    SOFA_STRUCT_DECL(PairStruct, myIntPair);
    SOFA_STRUCT_STREAM_METHODS(PairStruct);
    SOFA_STRUCT_COMPARE_METHOD(PairStruct);
};
struct NestedStruct
{
    SimpleStruct mySimpleStruct;
    SOFA_STRUCT_DECL(NestedStruct, mySimpleStruct);
    SOFA_STRUCT_STREAM_METHODS(NestedStruct);
    SOFA_STRUCT_COMPARE_METHOD(NestedStruct);
};
struct InheritingStruct: public SimpleStruct
{
    int myInt_2 = 20;
    float myFloat_2 = -0.8f;
    SOFA_STRUCT_DECL_W_BASECLASS(InheritingStruct, SimpleStruct, myInt_2, myFloat_2);
    SOFA_STRUCT_STREAM_METHODS(InheritingStruct);
    SOFA_STRUCT_COMPARE_METHOD(InheritingStruct);
};
struct ContainerStruct
{
    helper::vector<int> myIntVector = { 1,2,3 };
    helper::set<float> myFloatSet = std::set<float>({ 9,8,7 });
    helper::vector<SimpleStruct> myStructVector;
    SOFA_STRUCT_DECL(ContainerStruct, myIntVector, myFloatSet, myStructVector);
    SOFA_STRUCT_STREAM_METHODS(ContainerStruct);
    SOFA_STRUCT_COMPARE_METHOD(ContainerStruct);
};

template<typename T1, typename T2>
struct TemplatedStruct
{
    T1 myMemberT1;
    T2 myMemberT2;
    using TemplatedStruct_t = TemplatedStruct<T1, T2>;
    SOFA_STRUCT_DECL(TemplatedStruct_t, myMemberT1, myMemberT2);
    SOFA_STRUCT_STREAM_METHODS(TemplatedStruct_t);
    SOFA_STRUCT_COMPARE_METHOD(TemplatedStruct_t);
};

struct NoDefaultConstrStruct
{
    int myInt;

    SOFA_STRUCT_DECL(NoDefaultConstrStruct, myInt);
    SOFA_STRUCT_STREAM_METHODS(NoDefaultConstrStruct);
    SOFA_STRUCT_COMPARE_METHOD(NoDefaultConstrStruct);

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

SOFA_STRUCT_DEFINE_TYPEINFO(sofa::test_struct::EmptyStruct);
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::test_struct::PairStruct);
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::test_struct::NestedStruct);
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::test_struct::InheritingStruct);
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::test_struct::ContainerStruct);
SOFA_STRUCT_DEFINE_TYPEINFO(sofa::test_struct::TemplatedStruct<int, test_struct::SimpleStruct>);
//SOFA_STRUCT_DEFINE(sofa::test_struct::PointerStruct);

namespace sofa
{

namespace test_struct_2
{
using StructTypes = testing::Types<
    test_struct::EmptyStruct,
    test_struct::SimpleStruct,
    test_struct::PairStruct,
    test_struct::NestedStruct,
    test_struct::InheritingStruct,
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
    void operator()(MemberType&&) const
    {
        using DataType = typename MemberType::type;
        std::cout << DataTypeName<DataType>::name() << " " << MemberType::name() << ", ";
    }
};
struct PrintLastName
{
    template <typename MemberType>
    void operator()(MemberType&&) const
    {
        using DataType = typename MemberType::type;
        std::cout << DataTypeName<DataType>::name() << " " << MemberType::name();
    }
};
struct PrintValue
{
    template <typename MemberType>
    void operator()(MemberType&&, const typename MemberType::type& t) const
    { 
        std::cout << t << ", ";
    }
};
struct PrintLastValue
{
    template <typename MemberType>
    void operator()(MemberType&&, const typename MemberType::type& t) const
    {
        std::cout << t;
    }
};


TYPED_TEST(DataStructTypeInfoTest, checkStructTypeInfoIsOk)
{
    using StructType = TypeParam;
    Data<StructType> data("Struct");
    
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
    EXPECT_TRUE(typeInfo->IsStructure());
    EXPECT_EQ(std::tuple_size<typename StructType::MembersTuple>::value, typeInfo->StructureType()->structSize());
    
    EXPECT_EQ(AbstractTypeInfo::GetType(typeInfo->typeInfoID()), typeInfo);
    std::size_t id = typeInfo->typeInfoID();
    EXPECT_EQ(AbstractTypeInfo::GetType(id)->typeInfoID(), id);
    
    Data<StructType> data2("Struct");
    Data<int> dataInt("Int");
    Data<helper::vector<float>> dataVecFloat("VecFloat");
    
    EXPECT_EQ(data2.getValueTypeInfo()->typeInfoID(), id);
    EXPECT_EQ(AbstractTypeInfo::GetType(data2.GetValueTypeInfo()->typeInfoID()), typeInfo);
    EXPECT_NE(dataInt.getValueTypeInfo()->typeInfoID(), id);
    EXPECT_NE(AbstractTypeInfo::GetType(dataInt.GetValueTypeInfo()->typeInfoID()), typeInfo);
    EXPECT_NE(dataVecFloat.getValueTypeInfo()->typeInfoID(), id);
    EXPECT_NE(AbstractTypeInfo::GetType(dataVecFloat.getValueTypeInfo()->typeInfoID()), typeInfo);
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

struct ExpectReset
{
    template <typename MemberType>
    void operator()(MemberType&&, const typename MemberType::type& t) const
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
    StructTypeInfo<test_struct::NoDefaultConstrStruct>::for_each(testValue, ExpectReset{});
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


TEST(DataStructTypeInfoTest2, checkAbstractTypeInfoInheritingStruct)
{
    Data<test_struct::InheritingStruct> data("InheritingStruct");
    sofa::core::objectmodel::BaseData* baseData = &data;

    const AbstractTypeInfo* typeInfo = baseData->getValueTypeInfo();
    const AbstractStructureTypeInfo* structureInfo = typeInfo->StructureType();

    std::cout << "STATIC TYPEINFO" << std::endl;
    std::cout << "struct " << DataTypeInfo<test_struct::InheritingStruct>::name() << " { ";
    DataTypeInfo<test_struct::InheritingStruct>::for_each(PrintName{}, PrintLastName{});
    std::cout << " };" << std::endl;

    const AbstractTypeInfo* typeInfoM0 = structureInfo->getMemberTypeForIndex(0);
    EXPECT_TRUE(typeInfoM0->IsSingleValue());
    EXPECT_TRUE(typeInfoM0->SingleValueType()->Integer());

    const void* M0cptr = structureInfo->getMemberValue(data.getValueVoidPtr(), 0);
    EXPECT_EQ(typeInfoM0->byteSize(M0cptr), sizeof(int));
    EXPECT_EQ(typeInfoM0->getValuePtr(M0cptr), &(data.getValue().myInt_2));

    EXPECT_EQ(*(const int*)M0cptr, 20);
    EXPECT_EQ(structureInfo->getMemberName(data.getValueVoidPtr(), 0), "myInt_2");

    *(int*)structureInfo->editMemberValue(data.beginEditVoidPtr(), 0) = 20;
    data.endEditVoidPtr();
    EXPECT_EQ(*(const int*)M0cptr, 20);


    const void* M2cptr = structureInfo->getMemberValue(data.getValueVoidPtr(), 2);
    EXPECT_EQ(*(const int*)M2cptr, 10);
    EXPECT_EQ(structureInfo->getMemberName(data.getValueVoidPtr(), 2), "myInt");

    *(int*)structureInfo->editMemberValue(data.beginEditVoidPtr(), 2) = 24;
    data.endEditVoidPtr();
    EXPECT_EQ(*(const int*)M2cptr, 24);
}


TEST(DataStructTypeInfoTest2, checkAbstractTypeInfoSubTypeInfoContainerStruct)
{
    Data<test_struct::ContainerStruct> data("ContainerStruct");
    test_struct::ContainerStruct* d = data.beginEdit();
    d->myIntVector.resize(10);
    d->myFloatSet.insert({1.0f, 2.0f, 3.0f});
    d->myStructVector.resize(3);
    data.endEdit();
    sofa::core::objectmodel::BaseData* baseData = &data;

    const AbstractTypeInfo* typeInfo = baseData->getValueTypeInfo();
    const void* vptr = baseData->getValueVoidPtr();

    const void* subptr;
    const AbstractTypeInfo* subTypeInfo;


    // Valid cases

    bool res = defaulttype::getSubTypeInfo(vptr, typeInfo, std::vector<const void*>{}, subptr, subTypeInfo);
    EXPECT_TRUE(res);
    EXPECT_EQ(vptr, subptr);
    EXPECT_EQ(typeInfo, subTypeInfo);


    std::size_t ks = 0; // key type is size_t for structures
    const AbstractTypeInfo* firstContainerKeyType = typeInfo->StructureType()->getMemberTypeForIndex(0)->ContainerType()->getKeyType();
    defaulttype::unique_void_ptr kv = firstContainerKeyType->createInstance();
    firstContainerKeyType->setDataValueString(kv.get(), "3"); // equivalent to *kv = 3 but we're not supposed to know the type

    res = defaulttype::getSubTypeInfo(vptr, typeInfo, {&ks, kv.get()}, subptr, subTypeInfo);
    EXPECT_TRUE(res);
    EXPECT_EQ(subptr, &data.getValue().myIntVector[3]);
    EXPECT_EQ(subTypeInfo, typeInfo->StructureType()->getMemberTypeForIndex(0)->ContainerType()->getMappedType());


    ks = 1;
    const AbstractTypeInfo* secondContainerKeyType = typeInfo->StructureType()->getMemberTypeForIndex(1)->ContainerType()->getKeyType();
    defaulttype::unique_void_ptr kset = secondContainerKeyType->createInstance();
    secondContainerKeyType->setDataValueString(kset.get(), "2.0f"); // equivalent to *kset = 2.0f but we're not supposed to know the type

    res = defaulttype::getSubTypeInfo(vptr, typeInfo, {&ks, kset.get()}, subptr, subTypeInfo);
    EXPECT_TRUE(res);
    EXPECT_EQ(subptr, &(*data.getValue().myFloatSet.find(2.0f)));
    EXPECT_EQ(subTypeInfo, typeInfo->StructureType()->getMemberTypeForIndex(1)->ContainerType()->getMappedType());


    std::string two("2");
    std::string one("1");
    res = defaulttype::getSubTypeInfo(vptr, typeInfo, {two, one}, subptr, subTypeInfo);
    EXPECT_TRUE(res);
    EXPECT_EQ(subptr, &data.getValue().myStructVector[1]);
    EXPECT_EQ(subTypeInfo, typeInfo->StructureType()->getMemberTypeForIndex(2)->ContainerType()->getMappedType());


    // Invalid cases

    std::string zero("0");
    res = defaulttype::getSubTypeInfo(vptr, typeInfo, {zero, zero, zero}, subptr, subTypeInfo);
    EXPECT_FALSE(res); // cannot use key on a Single Value


    std::string three("3");
    res = defaulttype::getSubTypeInfo(vptr, typeInfo, {three}, subptr, subTypeInfo);
    EXPECT_FALSE(res); // key is too big for the structure


    std::string eleven("11");
    res = defaulttype::getSubTypeInfo(vptr, typeInfo, {zero, eleven}, subptr, subTypeInfo);
    EXPECT_FALSE(res); // key does not exist in the container
}

TEST(DataStructTypeInfoTest2, checkSimpleCopy)
{
    {
        Data<test_struct::EmptyStruct> data("EmptyStruct");
        EXPECT_TRUE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 0u);
    }
    {
        Data<test_struct::SimpleStruct> data("SimpleStruct");
        EXPECT_TRUE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 12u);
    }
    {
        Data<test_struct::NestedStruct> data("NestedStruct");
        EXPECT_TRUE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 12u);
    }
    {
        Data<test_struct::ContainerStruct> data("ContainerStruct");
        EXPECT_FALSE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 0u);
    }
    {
        Data<test_struct::TemplatedStruct<int, test_struct::SimpleStruct>> data("TemplatedStruct<int, SimpleStruct>");
        EXPECT_TRUE(data.getValueTypeInfo()->SimpleCopy());
        EXPECT_EQ(data.getValueTypeInfo()->byteSize(data.getValueVoidPtr()), 16u);
    }
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
