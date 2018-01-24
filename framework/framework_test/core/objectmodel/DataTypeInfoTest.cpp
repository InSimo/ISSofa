
#include <gtest/gtest.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <iostream>

namespace sofa
{

using namespace defaulttype;

TEST( DataFundamentalTypeInfoTest, checkDataTypeInfoFundamentalIsOk)
{
    Data< int > d_int("integer");
    const AbstractTypeInfo* typeInfo = d_int.getValueTypeInfo();

    EXPECT_TRUE( typeInfo->IsSingleValue() );
    EXPECT_TRUE( typeInfo->IsMultiValue() );
    EXPECT_FALSE( typeInfo->IsContainer() );
    const AbstractMultiValueTypeInfo* typeMVInfo = typeInfo->MultiValueType();
    EXPECT_TRUE( typeMVInfo->FixedFinalSize() );
    EXPECT_TRUE( typeMVInfo->getFinalValueType()->FixedFinalSize() );

    EXPECT_EQ( 1, typeMVInfo->FinalSize() );
    EXPECT_EQ( 1, typeMVInfo->finalSize( d_int.getValueVoidPtr() ) );
}

TEST( DataContainerTypeInfoTest, checkDataTypeInfoVectorSizeIsOk )
{
    Data< Vec3Types::VecCoord > d_x("position");
    sofa::helper::WriteAccessor< Data< Vec3Types::VecCoord > > x = d_x;
    x.resize( 10 );
    const AbstractTypeInfo* typeInfo = d_x.getValueTypeInfo();

    EXPECT_FALSE( typeInfo->IsSingleValue() );
    EXPECT_TRUE( typeInfo->IsMultiValue() );
    EXPECT_TRUE( typeInfo->IsContainer() );
    const AbstractMultiValueTypeInfo* typeMVInfo = typeInfo->MultiValueType();
    const AbstractContainerTypeInfo* typeCInfo = typeInfo->ContainerType();
    EXPECT_FALSE( typeMVInfo->FixedFinalSize() );
    const AbstractTypeInfo* baseInfo = typeCInfo->getMappedType();
    EXPECT_TRUE( baseInfo->IsMultiValue() );
    const AbstractMultiValueTypeInfo* baseMVInfo = baseInfo->MultiValueType();
    EXPECT_TRUE( baseMVInfo->FixedFinalSize() );

    EXPECT_EQ( baseMVInfo->FinalSize(), 3 );
    EXPECT_EQ( typeCInfo->containerSize( d_x.getValueVoidPtr() ), 10 );
    EXPECT_EQ( typeMVInfo->finalSize( d_x.getValueVoidPtr() ), baseMVInfo->FinalSize()*typeCInfo->containerSize( d_x.getValueVoidPtr() ) );
}

TEST( DataContainerTypeInfoTest, checkDataTypeInfoVectorOfVectorSizeIsOk )
{
    typedef sofa::helper::vector< sofa::helper::vector< Vec3Types::Coord > > VectorOfVectorOfCoord;
    Data< VectorOfVectorOfCoord > d_x("position");
    sofa::helper::WriteAccessor< Data< VectorOfVectorOfCoord > > x = d_x;
    x.resize( 3 );
    x[0].resize( 1 );
    x[1].resize( 2 );
    x[2].resize( 3 );
    const AbstractTypeInfo* typeInfo = d_x.getValueTypeInfo();

    EXPECT_FALSE( typeInfo->IsSingleValue() );
    EXPECT_FALSE( typeInfo->IsMultiValue() );
    EXPECT_TRUE( typeInfo->IsContainer() );
    const AbstractContainerTypeInfo* typeCInfo = typeInfo->ContainerType();
    const AbstractTypeInfo* baseInfo = typeCInfo->getMappedType();
    EXPECT_TRUE( baseInfo->IsMultiValue() );
    const AbstractMultiValueTypeInfo* baseMVInfo = baseInfo->MultiValueType();
    EXPECT_FALSE( baseMVInfo->FixedFinalSize() );
    EXPECT_EQ( baseMVInfo->FinalSize(), 3 );
}

TEST( DataTypeInfoMemcpyTest, checkDataTypeInfoVectorGetValuePtrIsOk )
{
    typedef sofa::helper::vector< Vec3Types::Coord > VectorOfCoord;
    Data< VectorOfCoord > d_vecCoord("VecCoord");
    sofa::helper::WriteAccessor< Data< VectorOfCoord > > v = d_vecCoord;
    v.resize(10);
    int i = -3;
    std::generate(v.begin(), v.end(), [&i]{ i+=3; return Vec3Types::Coord(i, i+1, i+2); });
    const AbstractTypeInfo* typeInfo = d_vecCoord.getValueTypeInfo();
    const void* vptr = d_vecCoord.getValueVoidPtr();
    
    ASSERT_TRUE(typeInfo->SimpleCopy());
    ASSERT_NE(typeInfo->getValuePtr(vptr), nullptr);
    ASSERT_EQ(typeInfo->byteSize(vptr), 10*sizeof(Vec3Types::Coord));
    
    VectorOfCoord vCopy;
    vCopy.resize(typeInfo->ContainerType()->containerSize(vptr));
    std::memcpy(vCopy.data(), typeInfo->getValuePtr(vptr), typeInfo->byteSize(vptr));
    
    ASSERT_EQ(d_vecCoord.getValue(), vCopy);
}

TEST( DataTypeInfoMemcpyTest, checkDataTypeInfoStringGetValuePtrIsOk )
{
    Data< std::string > d_str("String");
    d_str.setValue("My String");
    const AbstractTypeInfo* typeInfo = d_str.getValueTypeInfo();
    const void* vptr = d_str.getValueVoidPtr();
    
    ASSERT_TRUE(typeInfo->SimpleCopy());
    ASSERT_NE(typeInfo->getValuePtr(vptr), nullptr);
    ASSERT_EQ(typeInfo->byteSize(vptr), d_str.getValue().size()*sizeof(std::string::value_type));
    
    std::string strCopy;
    strCopy.assign((const char*)typeInfo->getValuePtr(vptr), typeInfo->byteSize(vptr));
    
    ASSERT_EQ(d_str.getValue(), strCopy);
}


struct MyType
{
    static std::string myString;
    /// Output stream
    inline friend std::ostream& operator<< ( std::ostream& out, const MyType& /*gp*/  )
    {
        return out;
    }
    /// Input stream
    inline friend std::istream& operator>> ( std::istream& in, MyType& /*gp*/ )
    {
        return in;
    }
};

std::string MyType::myString = "MyType::myString";

struct DataTypeInfoMyType_test : public ::testing::Test
{
    typedef sofa::helper::vector<MyType> VectorMyType;
    typedef sofa::Data<VectorMyType> DataVectorMyType;
    typedef sofa::Data<MyType> DataMyType;
    typedef sofa::defaulttype::DataTypeInfo<MyType>   DataTypeInfoMyType;

    static std::size_t size;

    DataTypeInfoMyType_test()
    :myData("myData")
    ,myVectorData("myVectorData")
    ,myTypeInfo(NULL)
    ,myVectorTypeInfo(NULL)
    ,myValueVoidPtr(NULL)
    ,myVectorValueVoidPtr(NULL)
    {

    }

    void SetUp()
    {
        myTypeInfo     = myData.getValueTypeInfo();
        myValueVoidPtr = myData.getValueVoidPtr();

        myVectorData.setValue( VectorMyType( DataTypeInfoMyType_test::size ) );
        myVectorTypeInfo     = myVectorData.getValueTypeInfo();
        myVectorCTypeInfo     = myVectorTypeInfo->ContainerType();
        myVectorValueVoidPtr = myVectorData.getValueVoidPtr(); 
    }
    
    DataMyType              myData;
    DataVectorMyType        myVectorData;
    const AbstractTypeInfo* myTypeInfo;
    const AbstractTypeInfo* myVectorTypeInfo;
    const AbstractContainerTypeInfo* myVectorCTypeInfo;
    const void*             myValueVoidPtr;
    const void*             myVectorValueVoidPtr;
};

std::size_t DataTypeInfoMyType_test::size = 7;

TEST_F(DataTypeInfoMyType_test, checkAbstractTypeInfoIsNotValidInfo )
{
    ASSERT_FALSE(myTypeInfo->ValidInfo());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractTypeInfoIsNotSingleValue )
{
    ASSERT_FALSE( myTypeInfo->IsSingleValue());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractTypeInfoIsNotMultiValue )
{
    ASSERT_FALSE( myTypeInfo->IsMultiValue());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractTypeInfoIsNotContainer )
{
    ASSERT_FALSE( myTypeInfo->IsContainer());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractTypeInfoIsNotZeroConstructor )
{
    ASSERT_FALSE(myTypeInfo->ZeroConstructor());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractTypeInfoIsNotSimpleCopy )
{
    ASSERT_FALSE(myTypeInfo->SimpleCopy());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractTypeInfoIsNotSimpleLayout )
{
    ASSERT_FALSE(myTypeInfo->SimpleLayout());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractTypeInfoIsNotCopyOnWrite )
{
    ASSERT_FALSE(myTypeInfo->CopyOnWrite());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractVectorTypeInfoIsNotSingleValue )
{
    ASSERT_FALSE( myVectorTypeInfo->IsSingleValue());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractVectorTypeInfoIsNotMultiValue )
{
    ASSERT_FALSE( myVectorTypeInfo->IsMultiValue());
}

TEST_F(DataTypeInfoMyType_test, checkAbstractVectorTypeInfoIsContainer )
{
    ASSERT_TRUE( myVectorTypeInfo->IsContainer());
}

TEST_F(DataTypeInfoMyType_test, checkVectorAbstractTypeInfoSizeIsOK)
{
    ASSERT_EQ( myVectorCTypeInfo->containerSize(myVectorValueVoidPtr), DataTypeInfoMyType_test::size );
}

TEST_F(DataTypeInfoMyType_test, checkVectorAbstractTypeInfoIsNotValid )
{
    ASSERT_EQ( myVectorTypeInfo->ValidInfo(),false );
}

TEST_F(DataTypeInfoMyType_test, checkVectorAbstractTypeInfoBaseTypeIsNotValid )
{
    ASSERT_EQ( myVectorCTypeInfo->getMappedType()->ValidInfo() ,false );
}

// This test does not work because the default implementation of DataTypeInfo<DataType>::getValueString 
// is an empty method.
//TEST_F(DataTypeInfoMyType_test, checkVectorAbstractTypeInfoGetValueString )
//{
//    for(std::size_t i=0;i<myVectorTypeInfo->size();++i)
//    {
//        //ASSERT_STREQ(myVectorTypeInfo->getTextValue( myVectorValueVoidPtr, i).c_str(), 
//        //             MyType::myString.c_str() );
//
//    }
//
//}

}
