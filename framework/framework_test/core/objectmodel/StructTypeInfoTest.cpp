
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
};
struct SimpleStruct
{
    int myInt;
    float myFloat;
    unsigned char myUChar;
    bool myBool;
    inline friend std::ostream& operator<<(std::ostream& os, const SimpleStruct& /*s*/) { return os; }
    inline friend std::istream& operator>>(std::istream& in, SimpleStruct& /*s*/) { return in; }
};
struct NestedStruct
{
    SimpleStruct mySimpleStruct;
    inline friend std::ostream& operator<<(std::ostream& os, const NestedStruct& /*s*/) { return os; }
    inline friend std::istream& operator>>(std::istream& in, NestedStruct& /*s*/) { return in; }
};
struct ContainerStruct
{
    std::vector<int> myIntVector;
    helper::set<float> myFloatSet;
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
    SimpleStruct,
    NestedStruct,
    ContainerStruct,
    PointerStruct
>;

TYPED_TEST_CASE(DataStructTypeInfoTest, StructTypes);


/////////////
/// Tests ///
/////////////


TYPED_TEST(DataStructTypeInfoTest, checkStructTypeInfoIsOk)
{
    using StructType = TypeParam;
    Data<StructType> data("Struct");
    StructTypeInfo<StructType> structTypeInfo;
    //const AbstractTypeInfo* typeInfo = data.getValueTypeInfo();
}

}
