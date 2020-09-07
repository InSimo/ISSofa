/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <sofa/core/dataparser/CppDataParser.h>
#include <gtest/gtest.h>

#include <sofa/defaulttype/EnumTypeInfo.h>
#include <sofa/defaulttype/PairTypeInfo.h>
#include <sofa/core/objectmodel/Data.h>

#include <vector>
#include <sofa/helper/map.h>

enum class uIntEnum : unsigned int
{
    first = 1u,
    second = 10u,
    third = 100u
};
SOFA_ENUM_DECL(uIntEnum, first, second, third);
SOFA_ENUM_STREAM_METHODS_NAME(uIntEnum);

struct SimpleStruct
{
    int myInt = 10;
    sofa::helper::pair<bool, uIntEnum> myPair = { false, uIntEnum::first };
    bool myBool = true;
    SOFA_STRUCT_DECL(SimpleStruct, myInt, myPair, myBool);
    SOFA_STRUCT_STREAM_METHODS(SimpleStruct);
    SOFA_STRUCT_COMPARE_METHOD(SimpleStruct);
};

template <typename _StructType>
struct CppDataParserTest : public ::testing::Test
{
    using StructType = _StructType;
};

// Single Value
struct StringTest {  using T = std::string; static T value() { return T("hello"); } static const char* text(bool) { return "\"hello\""; } };
struct EnumTest {  using T = uIntEnum; static T value() { return uIntEnum::second; } static const char* text(bool) { return "second"; } };
struct DoubleTest { using T = double; static T value() { return 1.01; } static const char* text(bool) { return "1.01"; } };
struct IntTest { using T = int; static T value() { return -1; } static const char* text(bool) { return "-1"; } };

// Multivalue
struct VecBoolTest {  using T = std::vector<bool>; static T value() { return T({true, true, false}); } static const char* text(bool) { return "{ 1, 1, 0 }"; } };

// Container
struct VecDoubleTest {  using T = sofa::helper::vector<double>; static T value() { return T({ 1.0, 8.0, 25.0 }); } static const char* text(bool) { return "{ 1.0, 8.0, 25.0 }"; } };
struct VecIntTest {  using T = sofa::helper::vector<int>; static T value() { return T({ 1, 8, 25 }); } static const char* text(bool) { return "{ 1, 8, 25 }"; } };
struct MapDoubleIntTest {  using T = std::map<double, int>; static T value() { return T({ { 5.0, 1 }, { 1.01, 5 }, { 0.0, 8 } }); } static const char* text(bool) { return "{ { 0.0, 8 }, { 1.01, 5 }, { 5.0, 1 } }"; } };
struct SetDoubleTest {  using T = sofa::helper::set<double>; static T value() { return T({ 1.1, 8.005, 25.0 }); } static const char* text(bool) { return "{ 1.1, 8.005, 25.0 }"; } };
struct FixedArrayTest {  using T = std::array<int, 3>; static T value() { return T{1, 1, 2}; } static const char* text(bool) { return "{ 1, 1, 2 }"; } };
struct EmptyVecIntTest {  using T = sofa::helper::vector<int>; static T value() { return T(); } static const char* text(bool) { return "{}"; } };
struct EmptyMapDoubleIntTest {  using T = std::map<double, int>; static T value() { return T(); } static const char* text(bool) { return "{}"; } };
struct EmptySetDoubleTest {  using T = sofa::helper::set<double>; static T value() { return T(); } static const char* text(bool) { return "{}"; } };

// Struct
struct StructTest { using T = SimpleStruct;
    static T value() { SimpleStruct s; s.myBool = false; s.myInt = 11; s.myPair = sofa::helper::pair<bool, uIntEnum>(true, uIntEnum::third); return s; }
    static const char* text(bool useNamed) { return useNamed ? "{ .myInt=11, .myPair={ .first=true, .second=third }, .myBool=false }" :  "{ 11, { true, third }, false }"; }
};

using StructTypes = testing::Types<
    StringTest, EnumTest, DoubleTest, IntTest, // Single value

    //VecBoolTest, //Multivalue
    VecDoubleTest, VecIntTest, MapDoubleIntTest, SetDoubleTest, FixedArrayTest, // Container
    EmptyVecIntTest, EmptyMapDoubleIntTest, EmptySetDoubleTest, // Container
    StructTest // Struct
>;

TYPED_TEST_CASE(CppDataParserTest, StructTypes);

namespace std
{
    //static std::ostream& operator << (std::ostream& os, const std::vector<bool>& /*vec*/) { return os; }
    //static std::istream& operator >> (std::istream& is, const std::vector<bool>& /*vec*/) { return is; }
    static std::ostream& operator << (std::ostream& os, const std::array<int, 3>& /*vec*/) { return os; }
    static std::istream& operator >> (std::istream& is, const std::array<int, 3>& /*vec*/) { return is; }
}

namespace
{
TYPED_TEST(CppDataParserTest, toCppTest)
{
    using StructType = TypeParam;

    for (bool useNamed: {false, true})
    {
        sofa::Data<typename StructType::T> data{ "test" };
        data.setValue(StructType::value());
        const char* text = StructType::text(useNamed);
        
        auto ti = data.getValueTypeInfo();
        auto ptr = data.getValueVoidPtr();

        sofa::core::dataparser::CppDataParser parser("cpp", useNamed);
        std::string buffer;
        parser.fromData(buffer, ptr, ti);
        EXPECT_STREQ(buffer.c_str(), text);
    }
}

TYPED_TEST(CppDataParserTest, toDataTest)
{
    using StructType = TypeParam;

    for (bool useNamed: {false, true})
    {
        sofa::Data<typename StructType::T> data{ "test" };
        //data.setValue(StructType::value());
        std::string text = StructType::text(useNamed);

        auto ti = data.getValueTypeInfo();
        auto ptr = data.beginEditVoidPtr();

        sofa::core::dataparser::CppDataParser parser("cpp", useNamed);
        parser.toData(text, ptr, ti);
        data.endEditVoidPtr();
        EXPECT_EQ(data.getValue(), StructType::value());
    }
}

TEST(CppDataParserTest, eraseMap)
{
    sofa::Data<std::map<double, int>> data{ "test" };

    auto ti = data.getValueTypeInfo();

    data.setValue({ { 1,1 }, { 2,3 } });
    auto ptr = data.beginEditVoidPtr();

    sofa::core::dataparser::CppDataParser parser("cpp", false);
    std::string buffer("{ {0, 5} }");
    parser.toData(buffer, ptr, ti);
    data.endEditVoidPtr();
    std::map<double, int> expectedMap { {0, 5} };
    EXPECT_EQ(expectedMap, data.getValue());
}
}
