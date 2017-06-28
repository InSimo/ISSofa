#include <sofa/helper/DataParserRegistry.h>
#include <sofa/defaulttype/AbstractTypeInfo.h>
#include <sofa/defaulttype/StructTypeInfo.h>
#include <sofa/core/objectmodel/Data.h>
#include <gtest/gtest.h>

namespace test_struct
{
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

SOFA_STRUCT_DEFINE(test_struct::SimpleStruct);

namespace
{

TEST(DataParserRegistryTest, intTest)
{
    auto* parser = sofa::helper::DataParserRegistry::getParser(0);

    sofa::Data<int> testValue("");
    parser->toData(std::string("22"), testValue.beginEditVoidPtr(), testValue.getValueTypeInfo());
    testValue.endEditVoidPtr();

    EXPECT_EQ(22, testValue.getValue());
    std::string output;
    parser->fromData(output, testValue.getValueVoidPtr(), testValue.getValueTypeInfo());
    EXPECT_EQ(std::string("22"), output);
}

TEST(DataParserRegistryTest, stringTest)
{
    auto* parser = sofa::helper::DataParserRegistry::getParser(0);

    sofa::Data<std::string> testValue("");
    parser->toData(std::string("Parse me"), testValue.beginEditVoidPtr(), testValue.getValueTypeInfo());
    testValue.endEditVoidPtr();

    EXPECT_EQ(std::string("Parse me"), testValue.getValue());

    std::string output;
    parser->fromData(output, testValue.getValueVoidPtr(), testValue.getValueTypeInfo());
    EXPECT_EQ(std::string("Parse me"), output);
}

TEST(DataParserRegistryTest, vecDoubleTest)
{
    auto* parser = sofa::helper::DataParserRegistry::getParser(0);

    sofa::Data<sofa::helper::vector<double>> testValue("");
    parser->toData(std::string("1.11 2.1 3.1 4.1 8.13"), testValue.beginEditVoidPtr(), testValue.getValueTypeInfo());
    testValue.endEditVoidPtr();

    {
        sofa::helper::ReadAccessor<sofa::Data<sofa::helper::vector<double>>> readTestValue = testValue;
        EXPECT_EQ(3.1, readTestValue[2]);
        EXPECT_EQ(8.13, readTestValue[4]);
    }
    std::string output;
    parser->fromData(output, testValue.getValueVoidPtr(), testValue.getValueTypeInfo());
    EXPECT_EQ(std::string("1.11 2.1 3.1 4.1 8.13"), output);
}

TEST(DataParserRegistryTest, vecBoolTest)
{
    auto* parser = sofa::helper::DataParserRegistry::getParser(0);

    sofa::Data<sofa::helper::vector<bool>> testValue("");
    parser->toData(std::string("1 1 0 1"), testValue.beginEditVoidPtr(), testValue.getValueTypeInfo());
    testValue.endEditVoidPtr();

    {
        sofa::helper::ReadAccessor<sofa::Data<sofa::helper::vector<bool>>> readTestValue = testValue;
        EXPECT_TRUE(readTestValue[1]);
        EXPECT_FALSE(readTestValue[2]);
    }
    std::string output;
    parser->fromData(output, testValue.getValueVoidPtr(), testValue.getValueTypeInfo());
    EXPECT_EQ(std::string("1 1 0 1"), output);
}

TEST(DataParserRegistryTest, structTest)
{
    auto* parser = sofa::helper::DataParserRegistry::getParser(0);

    sofa::Data<test_struct::SimpleStruct> testValue("");
    parser->toData(std::string("{ int myInt = 121 ; float myFloat = 3.14 ; unsigned char myUChar = b ; bool myBool = 0 ; }"), 
        testValue.beginEditVoidPtr(), testValue.getValueTypeInfo());
    testValue.endEditVoidPtr();

    std::string output;
    parser->fromData(output, testValue.getValueVoidPtr(), testValue.getValueTypeInfo());
    EXPECT_EQ(std::string("{ int myInt = 121 ; float myFloat = 3.14 ; unsigned char myUChar = b ; bool myBool = 0 ; }"), output);

    sofa::helper::ReadAccessor<sofa::Data<test_struct::SimpleStruct>> readTestValue = testValue;
    EXPECT_FALSE(readTestValue->myBool);
}

}
