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

#include <sofa/core/dataparser/JsonDataParser.h>
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
SOFA_ENUM_STREAM_METHODS(uIntEnum);

struct SimpleStruct
{
    int myInt = 10;
    sofa::helper::pair<bool, uIntEnum> myPair = { false, uIntEnum::third };
    bool myBool = true;
    SOFA_STRUCT_DECL(SimpleStruct, myInt, myPair, myBool);
    SOFA_STRUCT_STREAM_METHODS(SimpleStruct);
    SOFA_STRUCT_COMPARE_METHOD(SimpleStruct);
};

template <typename _StructType>
struct JsonDataParserTest : public ::testing::Test
{
    using StructType = _StructType;
};

// Single Value
class StringTest { public:  using T = std::string; static T value() { return T("hello"); } static constexpr const char* json = "\"hello\""; };
class EnumTest { public : using T = uIntEnum; static T value() { return uIntEnum::second; } static constexpr const char* json = "\"second\""; };
class DoubleTest { public: using T = double; static T value() { return 1.01; } static constexpr const char* json = "1.01"; };
class IntTest { public: using T = int; static T value() { return -1; } static constexpr const char* json = "-1"; };

// Multivalue
class VecBoolTest { public:  using T = std::vector<bool>; static T value() { return T({true, true, false}); } static constexpr const char* json = "[1,1,0]"; };

// Container
class VecDoubleTest { public:  using T = sofa::helper::vector<double>; static T value() { return T({ 1.0, 8.0, 25.0 }); } static constexpr const char* json = "[1.0,8.0,25.0]"; };
class VecIntTest { public:  using T = sofa::helper::vector<int>; static T value() { return T({ 1, 8, 25 }); } static constexpr const char* json = "[1,8,25]"; };
class MapDoubleIntTest { public:  using T = std::map<double, int>; static T value() { return T({ { 5.0, 1 }, { 1.01, 5 }, { 0.0, 8 } }); } static constexpr const char* json = "{\"0\":8,\"1.01\":5,\"5\":1}"; };
class SetDoubleTest { public:  using T = sofa::helper::set<double>; static T value() { return T({ 1.1, 8.005, 25.0 }); } static constexpr const char* json = "[1.1,8.005,25.0]"; };
class FixedArrayTest { public:  using T = std::array<int, 3>; static T value() { return T{1, 1, 2}; } static constexpr const char* json = "[1,1,2]"; };
class EmptyVecIntTest { public:  using T = sofa::helper::vector<int>; static T value() { return T(); } static constexpr const char* json = "[]"; };
class EmptyMapDoubleIntTest { public:  using T = std::map<double, int>; static T value() { return T(); } static constexpr const char* json = "{}"; };
class EmptySetDoubleTest { public:  using T = sofa::helper::set<double>; static T value() { return T(); } static constexpr const char* json = "[]"; };

// Struct
class StructTest { public:  using T = SimpleStruct; static T value() { SimpleStruct s; s.myBool = false; s.myInt = 11; s.myPair = sofa::helper::pair<bool, uIntEnum>(true, uIntEnum::third); return s; } static constexpr const char* json = "{\"myInt\":11,\"myPair\":{\"first\":1,\"second\":\"third\"},\"myBool\":0}"; };

using StructTypes = testing::Types<
    StringTest, EnumTest, DoubleTest, IntTest, // Single value

    VecBoolTest, //Multivalue
    VecDoubleTest, VecIntTest, MapDoubleIntTest, SetDoubleTest, FixedArrayTest, // Container
    EmptyVecIntTest, EmptyMapDoubleIntTest, EmptySetDoubleTest, // Container
    StructTest // Struct
>;

TYPED_TEST_CASE(JsonDataParserTest, StructTypes);

namespace std
{
    std::ostream& operator << (std::ostream& os, const std::vector<bool>& /*vec*/) { return os; }
    std::istream& operator >> (std::istream& is, const std::vector<bool>& /*vec*/) { return is; }
    std::ostream& operator << (std::ostream& os, const std::array<int, 3>& /*vec*/) { return os; }
    std::istream& operator >> (std::istream& is, const std::array<int, 3>& /*vec*/) { return is; }
}

namespace
{
    TYPED_TEST(JsonDataParserTest, toJsonTest)
    {
        using StructType = TypeParam;

        sofa::Data<typename StructType::T> data{ "test" };
        data.setValue(StructType::value());
        
        auto ti = data.getValueTypeInfo();
        auto ptr = data.getValueVoidPtr();

        sofa::core::dataparser::JsonDataParser parser("json");
        rapidjson::StringBuffer buffer;
        parser.fromData(buffer, ptr, ti);
        EXPECT_STREQ(StructType::json, buffer.GetString());
    }

    TYPED_TEST(JsonDataParserTest, toDataTest)
    {
        using StructType = TypeParam;

        sofa::Data<typename StructType::T> data{ "test" };

        auto ti = data.getValueTypeInfo();
        auto ptr = data.beginEditVoidPtr();

        sofa::core::dataparser::JsonDataParser parser("json");
        rapidjson::StringStream stream(StructType::json);
        parser.toData(stream, ptr, ti);
        data.endEditVoidPtr();
        EXPECT_EQ(StructType::value(), data.getValue());
    }

    TEST(JsonDataParserTest, eraseMap)
    {
        sofa::Data<std::map<double, int>> data{ "test" };

        auto ti = data.getValueTypeInfo();

        data.setValue({ { 1,1 }, { 2,3 } });
        auto ptr = data.beginEditVoidPtr();

        sofa::core::dataparser::JsonDataParser parser("json");
        rapidjson::StringStream stream("{\"0\":5}");
        parser.toData(stream, ptr, ti);
        data.endEditVoidPtr();
        std::map<double, int> expectedMap { {0, 5} };
        EXPECT_EQ(expectedMap, data.getValue());
    }
}
