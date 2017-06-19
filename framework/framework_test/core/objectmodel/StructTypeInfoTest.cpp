
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

struct EmptyStruct {};
struct SimpleStruct
{
    int myInt;
    float myFloat;
    unsigned char myUChar;
    bool myBool;
};
struct NestedStruct
{
    SimpleStruct mySimpleStruct;
};
struct ContainerStruct
{
    std::vector<int> myIntVector;
    helper::set<float> myFloatSet;
};
struct PointerStruct
{
    double *myDoublePointer;
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
