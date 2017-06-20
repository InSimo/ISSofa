
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

struct EmptyStruct
{
    inline friend std::ostream& operator<<(std::ostream& os, const EmptyStruct& /*s*/) { return os; }
    inline friend std::istream& operator>>(std::istream& in, EmptyStruct& /*s*/) { return in; }
    SOFA_STRUCT_DECL(EmptyStruct);
};
struct SimpleStruct
{
    int myInt = 10;
    float myFloat = -0.1f;
    unsigned char myUChar = 'c';
    bool myBool = true;
    inline friend std::ostream& operator<<(std::ostream& os, const SimpleStruct& /*s*/) { return os; }
    inline friend std::istream& operator>>(std::istream& in, SimpleStruct& /*s*/) { return in; }
    SOFA_STRUCT_DECL(SimpleStruct, myInt, myFloat, myUChar, myBool);
};
struct NestedStruct
{
    SimpleStruct mySimpleStruct;
    inline friend std::ostream& operator<<(std::ostream& os, const NestedStruct& /*s*/) { return os; }
    inline friend std::istream& operator>>(std::istream& in, NestedStruct& /*s*/) { return in; }
};
struct ContainerStruct
{
    helper::vector<int> myIntVector = {1,2,3};
    std::set<float> myFloatSet = {9,8,7};
    inline friend std::ostream& operator<<(std::ostream& os, const ContainerStruct& /*s*/) { return os; }
    inline friend std::istream& operator>>(std::istream& in, ContainerStruct& /*s*/) { return in; }
};
struct PointerStruct
{
    double *myDoublePointer;
    inline friend std::ostream& operator<<(std::ostream& os, const PointerStruct& /*s*/) { return os; }
    inline friend std::istream& operator>>(std::istream& in, PointerStruct& /*s*/) { return in; }
};

using StructTypes = testing::Types<
    EmptyStruct,
    SimpleStruct//,
    //NestedStruct,
    //ContainerStruct//,
    //PointerStruct
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
    template <typename DataType>
    void operator()(const DataType& t) const
    { 
        std::cout << t << ", ";
    }
};
struct PrintLastValue
{
    template <typename DataType>
    void operator()(const DataType& t) const
    { 
        std::cout << t;
    }
};

TYPED_TEST(DataStructTypeInfoTest, checkStructTypeInfoIsOk)
{
    using StructType = TypeParam;
    Data<StructType> data("Struct");
    //BaseData* baseData = &data;
    StructTypeInfo<StructType> structTypeInfo;
    
    std::cout << "STATIC TYPEINFO" << std::endl;
    std::cout << "struct " << structTypeInfo.name() << " { ";
    StructTypeInfo<StructType>::for_each(PrintName{}, PrintLastName{});
    std::cout << " };" << std::endl;
    
    std::cout << "DYNAMIC TYPEINFO" << std::endl;
    std::cout << "struct " << structTypeInfo.name() << " { ";
    StructTypeInfo<StructType>::for_each(data.getValue(), PrintValue{}, PrintLastValue{});
    std::cout << " };" << std::endl;
}

TYPED_TEST(DataStructTypeInfoTest, checkAbstractTypeInfoIsOk)
{
    using StructType = TypeParam;
    //Data<StructType> data("Struct");
    //BaseData* baseData = &data;
    
    //const AbstractTypeInfo* typeInfo = data.getValueTypeInfo();
    //EXPECT_TRUE(typeInfo->isStruct());
}

struct ExpectCleared
{
    template <typename DataType>
    void operator()(const DataType& t) const
    {
        EXPECT_EQ(t, DataType());
    }
};

TYPED_TEST(DataStructTypeInfoTest, checkStructTypeInfoResetValueIsOk)
{
    using StructType = TypeParam;
    Data<StructType> data("Struct");
    //BaseData* baseData = &data;
    StructTypeInfo<StructType>::resetValue(*data.beginEdit());
    data.endEdit();
    
    StructTypeInfo<StructType>::for_each(data.getValue(), ExpectCleared{});
    
    // DEBUG (warning : printing '\0' characters will end the output of std::cout)
    //std::cout << "RESET : struct " << StructTypeInfo<StructType>::name() << " { ";
    //StructTypeInfo<StructType>::for_each(data.getValue(), PrintValue{}, PrintLastValue{});
    //std::cout << " };" << std::endl;
}

}
