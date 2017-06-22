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

            static constexpr bool IsContainer        = false;  ///< true if this type is a container
            static constexpr bool IsSingleValue      = true;  ///< true if this type is a single value
            static constexpr bool IsMultiValue       = true;  ///< true if this type is equivalent to multiple values (either single value or a composition of arrays of the same type of values)
            static constexpr bool IsStructure        = false;  ///< true if this type is a structure

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

            template<size_t Index>
            using MemberType = typename std::tuple_element<Index, MembersTuple>::type;

            template<size_t index>
            static MappedType getEnumeratorValue()
            {
                static_assert(index < EnumSize, "Index out of enum bound");
                return MemberType<index>::enumeratorValue();
            }

            template<size_t index>
            static const char* getEnumeratorName()
            {
                static_assert(index < EnumSize, "Index out of enum bound");
                return MemberType<index>::enumeratorName();
            }

            template<size_t index>
            static DataType getEnumerator()
            {
                static_assert(index < EnumSize, "Index out of enum bound");
                return MemberType<index>::getEnumerator();
            }


            static void resetValue(DataType& data, size_t /*reserve*/ = 0)
            {
                //DataTypeInfo_Clear(data);
                data = MemberType<0>::getEnumerator(); // TODO : enforce default value to enum
            }


            ///////////
            // for_each utility with functors

            class SetDataFromString
            {
            public:
                SetDataFromString(const char* value) : m_string(value) {}

                template <typename T>
                void operator()(T&& MemberTypeI, DataType& data) const
                {
                    if (!std::strcmp(T::enumeratorName(), m_string))
                    {
                        data = T::getEnumerator();
                    }
                }
            private:
                const char* m_string;
            };


            class GetDataEnumeratorName
            {
            public:
                GetDataEnumeratorName(const char*& value) : m_string(value) {}

                template <typename T>
                void operator()(T&& MemberTypeI, DataType& data)
                {
                    if (T::getEnumerator() == data)
                    {
                        m_string = T::enumeratorName();
                    }
                }
            private:
                const char*& m_string;
            };


            class GetDataEnumeratorsNames
            {
            public:
                GetDataEnumeratorsNames(std::vector<std::string>& enumNames) : m_enumNames(enumNames) {}

                template <typename T>
                void operator()(T&& MemberTypeI, DataType& data)
                {
                    m_enumNames.push_back(T::enumeratorName());
                }
            private:
                std::vector<std::string>& m_enumNames;
            };


            class SetData
            {
            public:
                SetData(const MappedType value) : m_val(value) {
                }

                template <typename T>
                void operator()(T&& MemberTypeI, DataType& data) const
                {
                    if (T::enumeratorValue() == m_val)
                    {
                        data = T::getEnumerator();
                    }
                }
            private:
                const MappedType m_val;
            };
            

            // Call f(MemberDataType&) for each enum member
            template <typename F>
            static void for_each(DataType& data, F&& f)
            {
                TupleForEach<MembersTuple, EnumSize - 1 >::loop(data, std::forward<F>(f));
            }

            template<class Tuple, std::size_t I>
            class TupleForEach
            {
            public:
                template <typename F>
                static void loop(DataType& data, F&& f)
                {
                    f(MemberType<I>{}, data);
                    TupleForEach<Tuple, I - 1>::loop(data, std::forward<F>(f));
                }
            };

            template<class Tuple>
            class TupleForEach<Tuple, 0>
            {
            public:
                template <typename F>
                static void loop(DataType& data, F&& f)
                {
                    f(MemberType<0>{}, data);
                }
            };

            // end for_each utility
            ///////////

            template<typename DataTypeRef>
            static void setDataValueString(DataTypeRef&& data, const std::string& enumeratorName)   // set the given data according to the value of a given string
            {
                auto functor = SetDataFromString(enumeratorName.c_str());
                for_each(data, functor);
            }

            template<typename DataTypeRef>
            static void getDataEnumeratorString(const DataTypeRef& data, std::string& valueToFill)   // get the enumerator name as a string
            {
                const char* value;
                auto functor = GetDataEnumeratorName(value);
                DataTypeRef tempData = data;
                for_each(tempData, functor);
                
                std::ostringstream o; o << value; valueToFill = o.str();

            }

            template<typename DataTypeRef>
            static void getDataValueString(const DataTypeRef& data, std::string& valueToFill)   // get the enumerator value as a string
            {
                DataTypeInfo_ToString(data, valueToFill);
            }


            template<typename DataTypeRef, typename MappedType>
            static void setDataValue(DataTypeRef& data, const MappedType& value)   // set the enumerator value
            {
                auto functor = SetData(value);
                for_each(data, functor);
            }

            template <typename DataTypeRef, typename MappedType>
            static void getDataValue(const DataTypeRef& data, MappedType& value)   // get the enumerator value
            {
                value = static_cast<MappedType>(data);
            }

            template <typename DataTypeRef>
            static void getAvailableItems(const DataTypeRef& data, std::vector<std::string>& enumNames)   // get the enumerator value
            {
                auto functor = GetDataEnumeratorsNames(enumNames);
                DataTypeRef tempData = data;
                for_each(tempData, functor);
            }
            


            ///////////
            // Multi Value API

            static constexpr size_t FinalSize = 1;
            static constexpr size_t finalSize(const DataType& /*data*/) { return FinalSize; }

            static void setFinalSize(DataType& /*data*/, size_t /*size*/)
            {
            }

            template <typename DataTypeRef, typename T>
            static void getFinalValue(const DataTypeRef& data, size_t index, T& value)
            {
                if (index != 0) return;
                getDataValue(data, value);
            }

            template<typename DataTypeRef, typename T>
            static void setFinalValue(DataTypeRef&& data, size_t index, const T& value)
            {
                if (index != 0) return;
                setDataValue(std::forward<DataTypeRef>(data), value);
            }

            template<typename DataTypeRef>
            static void getFinalValueString(const DataTypeRef& data, size_t index, std::string& value)
            {
                if (index != 0) return;
                getDataValueString(data, value);
            }

            template<typename DataTypeRef>
            static void setFinalValueString(DataTypeRef&& data, size_t index, const std::string& value)
            {
                if (index != 0) return;
                setDataValueString(std::forward<DataTypeRef>(data), value);
            }

            // end of Multi Value API
            ///////////



            ///////////
            // to/from stream

            static void setDataValueStream(DataType& data, std::istream& is)
            {
                std::string enumeratorName;
                is >> enumeratorName;
                setDataValueString(data, enumeratorName);
            }

            // end of to/from stream
            ///////////


        };

        // end of EnumTypeInfo struct definition
        ////////////////////////




        ////////////////////////
        // enum macro definition
        
        #define SOFA_TO_STRING_STRUCT_NAME_1(expression) MyEnumMember##expression
        #define SOFA_TO_STRING_STRUCT_NAMES(...) SOFA_FOR_EACH(SOFA_TO_STRING_STRUCT_NAME_1, (,) , __VA_ARGS__)

        #define SOFA_STRUCTURIZE_1(enumerator)                                                                                      \
            struct MyEnumMember##enumerator {                                                                                       \
                static const char* enumeratorName() { return SOFA_TO_STRING_1(enumerator); }                                        \
                static myEnumType enumeratorValue() { return static_cast<myEnumType>(myEnumT::enumerator); }                        \
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
            template<> struct DataTypeName<myEnum##myEnumT> { static const char* name() { return "enum"; } };                       \
                                                                                                                                    \
            inline std::ostream& operator<<(std::ostream& os, const myEnum##myEnumT& s) { os << static_cast<myEnum##myEnumType>(s); return os; }        \
            inline std::istream& operator >> (std::istream& in, myEnum##myEnumT& s) { DataTypeInfo<myEnum##myEnumT>::setDataValueStream(s, in); return in; }

        // end of enum macro definition
        ////////////////////////



}   // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_ENUMTYPEINFO_H
