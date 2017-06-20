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
#ifndef SOFA_DEFAULTTYPE_ENUMTYPEINFO_H
#define SOFA_DEFAULTTYPE_ENUMTYPEINFO_H

#include <sofa/defaulttype/DataTypeInfo.h>
#include <vector>
#include <type_traits>
#include <sstream>
#include <typeinfo>

#include <tuple>
#include <sofa/helper/preprocessor.h>


namespace sofa
{

    namespace defaulttype
    {

        ////////////////////////
        // EnumTypeInfo struct definition

        template<class TDataType, class TMembersTuple>
        struct EnumTypeInfo
        {
            typedef TDataType DataType;

            typedef TMembersTuple MembersTuple;

            typedef typename std::underlying_type<DataType>::type MappedType;      ///< type contained in DataType
            typedef DataTypeInfo<MappedType> MappedTypeInfo;
            typedef typename MappedTypeInfo::FinalValueType  FinalValueType;  ///< type of the final atomic values (or void if not applicable)

            static constexpr ContainerKindEnum ContainerKind = ContainerKindEnum::Single;
            //static constexpr ValueKindEnum     FinalValueKind = MappedTypeInfo::FinalValueKind;
            static constexpr ValueKindEnum     FinalValueKind = ValueKindEnum::Enum;

            static constexpr bool IsContainer = false; ///< true if this type is a container
            static constexpr bool IsSingleValue = false;  ///< true if this type is a single value
            static constexpr bool IsMultiValue = false;  ///< true if this type is equivalent to multiple values (either single value or a composition of arrays of the same type of values)

            static constexpr bool ValidInfo = MappedTypeInfo::ValidInfo;  ///< true if this type has valid infos
                                                                            /// true if this type uses integer values
            static constexpr bool Integer = MappedTypeInfo::Integer;
            /// true if this type uses scalar values
            static constexpr bool Scalar = MappedTypeInfo::Scalar;
            /// true if this type uses text values
            static constexpr bool String = MappedTypeInfo::String;
            /// true if this type is unsigned
            static constexpr bool Unsigned = MappedTypeInfo::Unsigned;

            static constexpr bool FixedFinalSize = true;  ///< true if this type has a fixed size for all level until the final values

            static constexpr bool ZeroConstructor = MappedTypeInfo::ZeroConstructor;  ///< true if the constructor is equivalent to setting memory to 0
            static constexpr bool SimpleCopy = MappedTypeInfo::SimpleCopy;  ///< true if copying the data can be done with a memcpy
            static constexpr bool SimpleLayout = MappedTypeInfo::SimpleLayout;  ///< true if the layout in memory is simply N values of the same base type
            static constexpr bool CopyOnWrite = false; ///< true if this type uses copy-on-write
            static constexpr bool StoreKeys = true;  ///< true if the item keys are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)
            static constexpr bool StoreValues = true;  ///< true if the item values are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)


            static constexpr size_t EnumSize = std::tuple_size<MembersTuple>::value;

            static std::string name()
            {
                return DataTypeName<MappedType>::name();
            }

            static size_t enumSize()
            {
                static_assert(FixedFinalSize, "Enums must have a fixed number of options");
                return EnumSize;
            }

            template<size_t index>
            static MappedType getValue()
            {
                static_assert(index < EnumSize, "Index out of enum bound");
                return std::tuple_element<index, MembersTuple>::type::readVal();
            }

            template<size_t index>
            static const char* getEnumeratorName()
            {
                static_assert(index < EnumSize, "Index out of enum bound");
                return std::tuple_element<index, MembersTuple>::type::name();
            }

            template<size_t index>
            static DataType getEnumerator()
            {
                static_assert(index < EnumSize, "Index out of enum bound");
                return std::tuple_element<index, MembersTuple>::type::getEnumerator();
            }


            static void resetValue(DataType& data, size_t /*reserve*/ = 0)
            {
                DataTypeInfo_Clear(data);
            }

            template <typename DataTypeRef, typename T>
            static void getDataValue(const DataTypeRef& data, T& value)
            {
                //getDataValueForString(data, value, std::integral_constant<bool, FinalValueKind == ValueKindEnum::String>());
            }

            template<typename DataTypeRef, typename T>
            static void setDataValue(DataTypeRef&& data, const T& value)
            {
                //setDataValueForString(std::forward<DataTypeRef>(data), value, std::integral_constant<bool, FinalValueKind == ValueKindEnum::String>());
            }

            template<typename DataTypeRef>
            static void getDataValueString(const DataTypeRef& data, std::string& value)
            {
                //DataTypeInfo_ToString(data, value);
            }

            template<typename DataTypeRef>
            static void setDataValueString(DataTypeRef&& data, const std::string& value)
            {
                //DataTypeInfo_FromString(std::forward<DataTypeRef>(data), value);
            }
        };

        // end of EnumTypeInfo struct definition
        ////////////////////////




        ////////////////////////
        // enum macro definition
        
        #define SOFA_TO_STRING_STRUCT_NAME_1(expression) MyEnumMember##expression
        #define SOFA_TO_STRING_STRUCT_NAMES(...) SOFA_FOR_EACH(SOFA_TO_STRING_STRUCT_NAME_1, (,) , __VA_ARGS__)

        #define SOFA_STRUCTURIZE_1(enumerator)                                                                                      \
            struct MyEnumMember##enumerator {                                                                                       \
                static const char* name() { return SOFA_TO_STRING_1(enumerator); }                                                  \
                static myEnumType readVal() { return static_cast<myEnumType>(myEnumT::enumerator); }                                \
                static myEnumT getEnumerator() { return myEnumT::enumerator; }                                                      \
            };

        #define SOFA_STRUCTURIZE(...) SOFA_FOR_EACH(SOFA_STRUCTURIZE_1 , SOFA_EMPTY_DELIMITER , __VA_ARGS__)

        #define SOFA_ENUM(myEnum, ...)                                                                                              \
            namespace myEnum##nspace {                                                                                              \
                typedef myEnum myEnumT;                                                                                             \
                typedef typename std::underlying_type<myEnumT>::type  myEnumType;                                                   \
                SOFA_STRUCTURIZE(__VA_ARGS__)                                                                                       \
                typedef std::tuple<SOFA_TO_STRING_STRUCT_NAMES(__VA_ARGS__)>  myEnum##myEnumTuple;                                  \
            }                                                                                                                       \
            typedef myEnum myEnum##myEnumT;                                                                                         \
            typedef typename std::underlying_type<myEnum##myEnumT>::type  myEnum##myEnumType;                                       \
                                                                                                                                    \
            template<>                                                                                                              \
            struct DataTypeInfo<myEnum##myEnumT> :                                                                                  \
                public EnumTypeInfo<myEnum##myEnumT, myEnum##nspace::myEnum##myEnumTuple>                                           \
            {    };                                                                                                                 \
            template<> struct DataTypeName<myEnum##myEnumT> { static const char* name() { return "enum"; } };

        // end of enum macro definition
        ////////////////////////



}   // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_ENUMTYPEINFO_H
