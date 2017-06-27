/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_DEFAULTTYPE_DATATYPEINFO_H
#define SOFA_DEFAULTTYPE_DATATYPEINFO_H

#include <vector>
#include <type_traits>
#include <sofa/helper/fixed_array.h>
#include <sofa/helper/SSOBuffer.h>
#include <sofa/helper/DecodeTypeName.h>
//#include <sofa/helper/vector.h>
#include <sofa/helper/set.h>
#include <sofa/defaulttype/DataTypeKind.h>
#include <map>
#include <sstream>
#include <typeinfo>

namespace sofa
{

namespace helper
{
template <class T, class MemoryManager >
class vector;
}

namespace defaulttype
{

#define DEFAULT_DataTypeInfo
#define DEFAULT_DataTypeName

template<class TDataType>
struct DataTypeName
#ifdef DEFAULT_DataTypeName
{
    static std::string name()
    {
        return sofa::helper::DecodeTypeName::decodeTypeName(typeid(TDataType));
    }
}
#endif
;

template<class TDataType>
extern inline void DataTypeInfo_ContainerClear(TDataType& array)
{
    array.clear();
}

template<class TDataType>
extern inline void DataTypeInfo_ContainerReserve(TDataType& array, size_t reserve)
{
    array.reserve(reserve);
}

template <class TDataType>
extern inline void DataTypeInfo_ContainerResize(TDataType& array, size_t size)
{
    array.resize(size);
}


template<class TDataType>
extern inline void DataTypeInfo_Clear(TDataType& data)
{
    data = TDataType();
}

#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable:4244) // possible loss of data warnings, unavoidable here without overloading all combinations of basic types...
#endif

template<class TDataType, class T>
extern inline void DataTypeInfo_GetValue(const TDataType& data, T& value)
{
    value = static_cast<T>(data);
}

template<class TDataType, class T>
extern inline void DataTypeInfo_SetValue(TDataType& data, const T& value)
{
    data = static_cast<TDataType>(value);
}

#if defined(_MSC_VER)
#  pragma warning(pop)
#endif

template<class TDataType>
extern inline void DataTypeInfo_GetValue(const TDataType& data, TDataType& value)
{
    value = data;
}

template<class TDataType>
extern inline void DataTypeInfo_SetValue(TDataType& data, const TDataType& value)
{
    data = value;
}

template<class TDataType>
extern inline void DataTypeInfo_GetValue(const TDataType& data, std::string& value)
{
    std::ostringstream o; o << data; value = o.str();
}

template<class TDataType>
extern inline void DataTypeInfo_SetValue(TDataType& data, const std::string& value)
{
    std::istringstream i(value); i >> data;
}

// string overloads

template<class T>
extern inline void DataTypeInfo_GetValue(const std::string& data, T& value)
{
    DataTypeInfo_SetValue(value, data);
}

template<class T>
extern inline void DataTypeInfo_SetValue(std::string& data, const T& value)
{
    DataTypeInfo_GetValue(value, data);
}

extern inline void DataTypeInfo_GetValue(const std::string& data, std::string& value)
{
    value = data;
}

extern inline void DataTypeInfo_SetValue(std::string& data, const std::string& value)
{
    data = value;
}

// bool overloads

extern inline void DataTypeInfo_GetValue(const bool& data, std::string& value)
{
    std::ostringstream o; o << data; value = o.str();
}

template<class T>
extern inline void DataTypeInfo_SetValue(bool& data, const T& value)
{
    data = (value != 0);
}

extern inline void DataTypeInfo_SetValue(bool& data, const std::string& value)
{
    std::istringstream i(value); i >> data;
}

template<class T>
extern inline void DataTypeInfo_SetValue(std::vector<bool>::reference data, const T& value)
{
    data = (value != 0);
}

extern inline void DataTypeInfo_SetValue(std::vector<bool>::reference data, const std::string& value)
{
    bool val = false;
    std::istringstream i(value); i >> val;
    data = val;
}

// to / from string

template<class TDataType>
extern inline void DataTypeInfo_ToString(const TDataType& data, std::string& value)
{
    DataTypeInfo_GetValue(data, value);
}

template<class TDataType>
extern inline void DataTypeInfo_FromString(TDataType&& data, const std::string& value)
{
    DataTypeInfo_SetValue(std::forward<TDataType>(data), value);
}


/** Type traits class for objects stored in Data

    DataTypeInfo is part of the introspection/reflection capabilities of the
    Sofa scene graph API; it is used to manipulate Data values generically
    in template code, and work with different types of containers, integers,
    scalars (float, double...), strings, etc

    For example, it can be used to work with arrays without having to handle all
    the possible array classes used in Sofa: fixed or dynamic size, CPU or GPU,
    etc.
*/

template<class TDataType>
struct DataTypeInfo;

template<class TDataType>
struct InvalidDataTypeInfo
{
    typedef TDataType DataType;
    typedef void      KeyType;         ///< type uniquely identifying items (or void if not applicable)
    typedef void      MappedType;      ///< type contained in DataType
    typedef void      FinalValueType;  ///< type of the final atomic values (or void if not applicable)

    static constexpr ContainerKindEnum ContainerKind  = ContainerKindEnum::Single;
    static constexpr ValueKindEnum     FinalValueKind = ValueKindEnum::Void;

    static constexpr bool IsContainer        = false; ///< true if this type is a container
    static constexpr bool IsSingleValue      = false;  ///< true if this type is a single value
    static constexpr bool IsMultiValue       = false;  ///< true if this type is equivalent to multiple values (either single value or a composition of arrays of the same type of values)
    static constexpr bool IsStructure        = false;  ///< true if this type is a structure

    static constexpr bool ValidInfo          = false;  ///< true if this type has valid infos
    static constexpr bool Integer            = false;  ///< true if this type uses integer values
    static constexpr bool Scalar             = false;  ///< true if this type uses scalar values
    static constexpr bool String             = false;  ///< true if this type uses text values
    static constexpr bool Unsigned           = false;  ///< true if this type is unsigned
    //static constexpr bool FixedContainerSize = true; ///< true if this type has a fixed size for this container level
    static constexpr bool FixedFinalSize     = false;  ///< true if this type has a fixed size for all level until the final values
    
    static constexpr bool ZeroConstructor    = false;  ///< true if the constructor is equivalent to setting memory to 0
    static constexpr bool SimpleCopy         = false;  ///< true if copying the data can be done with a memcpy
    static constexpr bool SimpleLayout       = false;  ///< true if the layout in memory is simply N values of the same base type
    static constexpr bool CopyOnWrite        = false; ///< true if this type uses copy-on-write
    static constexpr bool StoreKeys          = false;  ///< true if the item keys are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)
    static constexpr bool StoreValues        = false;  ///< true if the item values are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)

    //static constexpr size_t ContainerSize = 1; ///< 1, or fixed container size if FixedContainerSize is 1
    static constexpr size_t FinalSize = 1; ///< 1, or fixed final size if FixedFinalSize is 1
    static constexpr size_t ByteSize = 0; ///< if known at compile time, the size in bytes of the DataType, else 0

    static constexpr size_t finalSize(const DataType& /*data*/) { return FinalSize; }
    static constexpr size_t byteSize(const DataType& /*data*/) { return ByteSize; }

    static const void* getValuePtr(const DataType& /*data*/) { return nullptr; }

    static void resetValue(DataType& data, size_t /*reserve*/ = 0)
    {
        DataTypeInfo_Clear(data);
    }

    // Single Value API

    template <class DataTypeRef, typename T>
    static void getDataValue(const DataTypeRef& data, T& value)
    {
        DataTypeInfo_GetValue(data, value);
    }

    template<class DataTypeRef, typename T>
    static void setDataValue(DataTypeRef&& data, const T& value)
    {
        DataTypeInfo_SetValue(std::forward<DataTypeRef>(data), value);
    }

    template<class DataTypeRef>
    static void getDataValueString(const DataTypeRef& data, std::string& value)
    {
        DataTypeInfo_ToString(data, value);
    }

    template<class DataTypeRef>
    static void setDataValueString(DataTypeRef&& data, const std::string& value)
    {
        DataTypeInfo_FromString(std::forward<DataTypeRef>(data), value);
    }

    template <typename DataTypeRef>
    static void getAvailableItems(const DataTypeRef& data, std::vector<std::string>& enumNames) {}

};

#ifdef DEFAULT_DataTypeInfo
template<class TDataType>
struct DataTypeInfo : public InvalidDataTypeInfo<TDataType>
{
};
#endif

typedef sofa::helper::SSOBuffer<> TypeInfoKeyBuffer;

template<class TDataType, ValueKindEnum TValueKind, bool TUnsigned = false>
struct SingleValueTypeInfo
{
    typedef TDataType DataType;
    typedef void      KeyType;         ///< type uniquely identifying items (or void if not applicable)
    typedef DataType  MappedType;      ///< type contained in DataType
    typedef DataType  FinalValueType;  ///< type of the final atomic values (or void if not applicable)

    static constexpr ContainerKindEnum ContainerKind  = ContainerKindEnum::Single;
    static constexpr ValueKindEnum     FinalValueKind = TValueKind;

    static constexpr bool IsContainer        = false; ///< true if this type is a container
    static constexpr bool IsSingleValue      = true;  ///< true if this type is a single value
    static constexpr bool IsMultiValue       = true;  ///< true if this type is equivalent to multiple values (either single value or a composition of arrays of the same type of values)
    static constexpr bool IsStructure        = false; ///< true if this type is a structure

    static constexpr bool ValidInfo          = true;  ///< true if this type has valid infos
    /// true if this type uses integer values
    static constexpr bool Integer            = (TValueKind == ValueKindEnum::Integer ||
                                                TValueKind == ValueKindEnum::Enum  ||
                                                TValueKind == ValueKindEnum::Bool  );
    /// true if this type uses scalar values
    static constexpr bool Scalar             = (TValueKind == ValueKindEnum::Scalar);
    /// true if this type uses text values
    static constexpr bool String             = (TValueKind == ValueKindEnum::String);
    /// true if this type is unsigned
    static constexpr bool Unsigned           = TUnsigned;
    //static constexpr bool FixedContainerSize = true;  ///< true if this type has a fixed size for this container level
    static constexpr bool FixedFinalSize     = true;  ///< true if this type has a fixed size for all level until the final values
    
    static constexpr bool ZeroConstructor    = true;  ///< true if the constructor is equivalent to setting memory to 0
    static constexpr bool SimpleCopy         = true;  ///< true if copying the data can be done with a memcpy
    static constexpr bool SimpleLayout       = true;  ///< true if the layout in memory is simply N values of the same base type
    static constexpr bool CopyOnWrite        = false; ///< true if this type uses copy-on-write
    static constexpr bool StoreKeys          = true;  ///< true if the item keys are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)
    static constexpr bool StoreValues        = true;  ///< true if the item values are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)

    //static constexpr size_t ContainerSize = 1; ///< 1, or fixed container size if FixedContainerSize is 1
    static constexpr size_t FinalSize = 1; ///< 1, or fixed final size if FixedFinalSize is 1
    static constexpr size_t ByteSize = !String ? sizeof(DataType) : 0; ///< if known at compile time, the size in bytes of the DataType, else 0

    //static constexpr size_t containerSize(const DataType& /*data*/) { return ContainerSize; }
    static constexpr size_t finalSize(const DataType& /*data*/) { return FinalSize; }
    static constexpr size_t byteSize(const DataType& data)
    {
            return byteSize(data, std::integral_constant<bool, String>());
    }
protected:
    static size_t byteSize(const DataType& data, std::true_type)
    {
        return static_cast<std::string>(data).size(); //TODO: handle the string as a container<char>
    }
    static size_t byteSize(const DataType& data, std::false_type)
    {
        return ByteSize;
    }
public:

    static const void* getValuePtr(const DataType& data)
    {
        return getValuePtr(data, std::integral_constant<bool, String>());
    }
protected:
    static const void* getValuePtr(const DataType& data, std::true_type)
    {
        return static_cast<std::string>(data).data(); //TODO: handle the string as a container<char>
    }
    static const void* getValuePtr(const DataType& data, std::false_type)
    {
        return &data;
    }
public:

    static void resetValue(DataType& data, size_t /*reserve*/ = 0)
    {
        DataTypeInfo_Clear(data);
    }

    // Single Value API
protected:
    template <typename DataTypeRef, typename T>
    static void getDataValueForString(const DataTypeRef& data, T& value, std::false_type)
    {
        DataTypeInfo_GetValue(data, value);
    }
    template <typename DataTypeRef, typename T>
    static void getDataValueForString(const DataTypeRef& data, T& value, std::true_type)
    {
        std::string s;
        DataTypeInfo_ToString(data, s);
        DataTypeInfo_FromString(value, s);
    }

    template<typename DataTypeRef, typename T>
    static void setDataValueForString(DataTypeRef&& data, const T& value, std::false_type)
    {
        DataTypeInfo_SetValue(std::forward<DataTypeRef>(data), value);
    }
    template<typename DataTypeRef, typename T>
    static void setDataValueForString(DataTypeRef&& data, const T& value, std::true_type)
    {
        std::string s;
        DataTypeInfo_ToString(value, s);
        DataTypeInfo_FromString(std::forward<DataTypeRef>(data), s);
    }
public:
    template <typename DataTypeRef, typename T>
    static void getDataValue(const DataTypeRef& data, T& value)
    {
        getDataValueForString(data, value, std::integral_constant<bool, FinalValueKind == ValueKindEnum::String>());
    }

    template<typename DataTypeRef, typename T>
    static void setDataValue(DataTypeRef&& data, const T& value)
    {
        setDataValueForString(std::forward<DataTypeRef>(data), value, std::integral_constant<bool, FinalValueKind == ValueKindEnum::String>());
    }

    template<typename DataTypeRef>
    static void getDataValueString(const DataTypeRef& data, std::string& value)
    {
        DataTypeInfo_ToString(data, value);
    }

    template<typename DataTypeRef>
    static void setDataValueString(DataTypeRef&& data, const std::string& value)
    {
        DataTypeInfo_FromString(std::forward<DataTypeRef>(data), value);
    }

    template <typename DataTypeRef>
    static void getAvailableItems(const DataTypeRef& data, std::vector<std::string>& enumNames) {}

    // Multi Value API

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

#if 0
    // iterators support (trivial here)
    
    template<class DataPtr> class single_iterator
    {
        DataPtr m_data;
    public:
        single_iterator(DataPtr data) : m_data(data) {}
        void operator++() { m_data = NULL; }
        void operator++(int) { m_data = NULL; }
        DataPtr value() const { return m_data; }
        KeyType key() const { }
        bool operator==(const single_iterator<DataPtr>& i2) const { return m_data == i2.m_data; }
        bool operator!=(const single_iterator<DataPtr>& i2) const { return !(*this == i2); }
    };

    typedef single_iterator<DataType*> iterator;
    typedef single_iterator<const DataType*> const_iterator;
    static iterator begin(DataType& data)
    {
        return iterator(&data);
    }
    static iterator end(DataType& data)
    {
        return iterator(NULL);
    }
    static const_iterator cbegin(const DataType& data)
    {
        return const_iterator(&data);
    }
    static const_iterator cend(const DataType& data)
    {
        return const_iterator(NULL);
    }
#endif
};


template<> struct DataTypeInfo<bool> : public SingleValueTypeInfo<bool, ValueKindEnum::Bool> {};
template<> struct DataTypeName<bool> { static const char* name() { return "bool"; } };

template<> struct DataTypeInfo<char> : public SingleValueTypeInfo<char, ValueKindEnum::Integer> {};
template<> struct DataTypeName<char> { static const char* name() { return "char"; } };

template<> struct DataTypeInfo<unsigned char> : public SingleValueTypeInfo<unsigned char, ValueKindEnum::Integer, true> {};
template<> struct DataTypeName<unsigned char> { static const char* name() { return "unsigned char"; } };

template<> struct DataTypeInfo<short> : public SingleValueTypeInfo<short, ValueKindEnum::Integer> {};
template<> struct DataTypeName<short> { static const char* name() { return "short"; } };

template<> struct DataTypeInfo<unsigned short> : public SingleValueTypeInfo<unsigned short, ValueKindEnum::Integer, true> {};
template<> struct DataTypeName<unsigned short> { static const char* name() { return "unsigned short"; } };

template<> struct DataTypeInfo<int> : public SingleValueTypeInfo<int, ValueKindEnum::Integer> {};
template<> struct DataTypeName<int> { static const char* name() { return "int"; } };

template<> struct DataTypeInfo<unsigned int> : public SingleValueTypeInfo<unsigned int, ValueKindEnum::Integer, true> {};
template<> struct DataTypeName<unsigned int> { static const char* name() { return "unsigned int"; } };

template<> struct DataTypeInfo<long> : public SingleValueTypeInfo<long, ValueKindEnum::Integer> {};
template<> struct DataTypeName<long> { static const char* name() { return "long"; } };

template<> struct DataTypeInfo<unsigned long> : public SingleValueTypeInfo<unsigned long, ValueKindEnum::Integer, true> {};
template<> struct DataTypeName<unsigned long> { static const char* name() { return "unsigned long"; } };

template<> struct DataTypeInfo<long long> : public SingleValueTypeInfo<long long, ValueKindEnum::Integer> {};
template<> struct DataTypeName<long long> { static const char* name() { return "long long"; } };

template<> struct DataTypeInfo<unsigned long long> : public SingleValueTypeInfo<unsigned long long, ValueKindEnum::Integer, true> {};
template<> struct DataTypeName<unsigned long long> { static const char* name() { return "unsigned long long"; } };

template<> struct DataTypeInfo<float> : public SingleValueTypeInfo<float, ValueKindEnum::Scalar> {};
template<> struct DataTypeName<float> { static const char* name() { return "float"; } };

template<> struct DataTypeInfo<double> : public SingleValueTypeInfo<double, ValueKindEnum::Scalar> {};
template<> struct DataTypeName<double> { static const char* name() { return "double"; } };

template<> struct DataTypeInfo<std::string> : public SingleValueTypeInfo<std::string, ValueKindEnum::String> {};
template<> struct DataTypeName<std::string> { static const char* name() { return "string"; } };


template<class TDataType, class TMappedType, int TFixedSize>
struct MultiValueTypeInfo
{
    typedef TDataType DataType;
    typedef void      KeyType;         ///< type uniquely identifying items (or void if not applicable)
    typedef TMappedType MappedType;      ///< type contained in DataType
    typedef DataTypeInfo<MappedType> MappedTypeInfo;
    typedef typename MappedTypeInfo::FinalValueType  FinalValueType;  ///< type of the final atomic values (or void if not applicable)

    static constexpr ContainerKindEnum ContainerKind = ContainerKindEnum::Single;
    static constexpr ValueKindEnum     FinalValueKind = MappedTypeInfo::FinalValueKind;

    static constexpr bool IsContainer   = false;  ///< true if this type is a container
    static constexpr bool IsSingleValue = false;  ///< true if this type is a single value
    static constexpr bool IsMultiValue  = true;   ///< true if this type is equivalent to multiple values (either single value or a composition of arrays of the same type of values)
    static constexpr bool IsStructure   = false;  ///< true if this type is a structure

    static constexpr bool ValidInfo = MappedTypeInfo::ValidInfo;  ///< true if this type has valid infos
                                             /// true if this type uses integer values
    static constexpr bool Integer = MappedTypeInfo::Integer;
    /// true if this type uses scalar values
    static constexpr bool Scalar = MappedTypeInfo::Scalar;
    /// true if this type uses text values
    static constexpr bool String = MappedTypeInfo::String;
    /// true if this type is unsigned
    static constexpr bool Unsigned = MappedTypeInfo::Unsigned;

    static constexpr bool FixedFinalSize = MappedTypeInfo::FixedFinalSize && TFixedSize>0;  ///< true if this type has a fixed size for all level until the final values

    static constexpr bool ZeroConstructor = MappedTypeInfo::ZeroConstructor;  ///< true if the constructor is equivalent to setting memory to 0
    static constexpr bool SimpleCopy = TFixedSize>0 && MappedTypeInfo::SimpleCopy;  ///< true if copying the data can be done with a memcpy
    static constexpr bool SimpleLayout = MappedTypeInfo::SimpleLayout;  ///< true if the layout in memory is simply N values of the same base type
    static constexpr bool CopyOnWrite = TFixedSize == 0; ///< true if this type uses copy-on-write
    static constexpr bool StoreKeys = false;  ///< true if the item keys are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)
    static constexpr bool StoreValues = false;  ///< true if the item values are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)

    static constexpr size_t FinalSize = (TFixedSize == 0 ? 1 : TFixedSize)*MappedTypeInfo::FinalSize; ///< 1, or fixed final size if FixedFinalSize is 1
    static constexpr size_t ByteSize = FixedFinalSize ? TFixedSize * MappedTypeInfo::ByteSize : 0; ///< if known at compile time, the size in bytes of the DataType, else 0

    static size_t finalSize(const DataType& data)
    {
        if (FixedFinalSize)
        {
            return FinalSize;
        }
        else
        {
            return data.size() * FinalSize;
        }
    }
    static constexpr size_t byteSize(const DataType& /*data*/)
    {
        // No general formula exists to compute the byteSize (finalSize(data) * MappedTypeInfo::ByteSize is wrong when DataType is vector<bool> because it's a bitset)
        // Use Single Value API or Container API if available
        return ByteSize;
    }

    static const void* getValuePtr(const DataType& data)
    {
        return nullptr; // TODO
    }

    static void resetValue(DataType& data, size_t /*reserve*/ = 0)
    {
        DataTypeInfo_Clear(data);
    }

    // Single Value API
protected:
    template <typename DataTypeRef, typename T>
    static void getDataValueForString(const DataTypeRef& data, T& value, std::false_type)
    {
        DataTypeInfo_GetValue(data, value);
    }
    template <typename DataTypeRef, typename T>
    static void getDataValueForString(const DataTypeRef& data, T& value, std::true_type)
    {
        std::string s;
        DataTypeInfo_ToString(data, s);
        DataTypeInfo_FromString(value, s);
    }

    template<typename DataTypeRef, typename T>
    static void setDataValueForString(DataTypeRef&& data, const T& value, std::false_type)
    {
        DataTypeInfo_SetValue(std::forward<DataTypeRef>(data), value);
    }
    template<typename DataTypeRef, typename T>
    static void setDataValueForString(DataTypeRef&& data, const T& value, std::true_type)
    {
        std::string s;
        DataTypeInfo_ToString(value, s);
        DataTypeInfo_FromString(std::forward<DataTypeRef>(data), s);
    }
public:
    template <typename DataTypeRef, typename T>
    static void getDataValue(const DataTypeRef& data, T& value)
    {
        getDataValueForString(data, value, std::integral_constant<bool, FinalValueKind == ValueKindEnum::String>());
    }

    template <typename DataTypeRef, typename T>
    static void setDataValue(DataTypeRef&& data, const T& value)
    {
        setDataValueForString(std::forward<DataTypeRef>(data), value, std::integral_constant<bool, FinalValueKind == ValueKindEnum::String>());
    }

    template<typename DataTypeRef>
    static void getDataValueString(const DataTypeRef& data, std::string& value)
    {
        DataTypeInfo_ToString(data, value);
    }

    template<typename DataTypeRef>
    static void setDataValueString(DataTypeRef&& data, const std::string& value)
    {
        DataTypeInfo_FromString(std::forward<DataTypeRef>(data), value);
    }

    template <typename DataTypeRef>
    static void getAvailableItems(const DataTypeRef& data, std::vector<std::string>& enumNames) {}

    // Multi Value API

    template<typename DataTypeRef>
    static void setFinalSize(DataTypeRef&& data, size_t size)
    {
        if (!FixedFinalSize)
        {
            DataTypeInfo_ContainerResize(std::forward<DataTypeRef>(data), size / FinalSize);
        }
    }

    template <typename DataTypeRef, typename T>
    static void getFinalValue(const DataTypeRef& data, size_t index, T& value)
    {
        MappedTypeInfo::getFinalValue(data[index / FinalSize], index % FinalSize, value);
    }

    template<typename DataTypeRef, typename T>
    static void setFinalValue(DataTypeRef&& data, size_t index, const T& value)
    {
        MappedTypeInfo::setFinalValue(data[index / FinalSize], index % FinalSize, value);
    }

    template<typename DataTypeRef>
    static void getFinalValueString(const DataTypeRef& data, size_t index, std::string& value)
    {
        MappedTypeInfo::getFinalValueString(data[index / FinalSize], index % FinalSize, value);
    }

    template<typename DataTypeRef>
    static void setFinalValueString(DataTypeRef&& data, size_t index, const std::string& value)
    {
        MappedTypeInfo::setFinalValueString(data[index / FinalSize], index % FinalSize, value);
    }
};






template<class TDataType, ContainerKindEnum TContainerKind, size_t TFixedSize> struct DataTypeInfo_ContainerTypes;

template<class TDataType, size_t TFixedSize> struct DataTypeInfo_ContainerTypes<TDataType, ContainerKindEnum::Array, TFixedSize>
{
    typedef typename TDataType::size_type      KeyType;
    typedef typename TDataType::value_type     MappedType;
    typedef typename TDataType::const_iterator const_iterator;
    typedef typename TDataType::iterator       iterator;
    static auto value(const TDataType&, const const_iterator& it) -> decltype(*it) { return *it; }
    static auto value(      TDataType&, const       iterator& it) -> decltype(*it) { return *it; }
    static KeyType key(const TDataType& data, const const_iterator& it) { return (it - data.begin()); }
    static const KeyType& key(const TDataType& data, const const_iterator& it, TypeInfoKeyBuffer& keyBuffer)
    {
        KeyType& key = *keyBuffer.getOrCreate<KeyType>();
        key = (it - data.begin());
        return key;
    }
    static auto valueAtIndex(const TDataType& data, size_t index) -> decltype(data[index]) { return data[index]; }
    static auto valueAtIndex(      TDataType& data, size_t index) -> decltype(data[index]) { return data[index]; }
    static KeyType keyAtIndex(const TDataType& data, size_t index) { return index; }
    static const KeyType& keyAtIndex(const TDataType& data, size_t index, TypeInfoKeyBuffer& keyBuffer)
    {
        KeyType& key = *keyBuffer.getOrCreate<KeyType>();
        key = index;
        return key;
    }
    static const MappedType* find(const TDataType& data, const KeyType& k)
    {
        if (k < static_cast<KeyType>(data.size()))
        {
            return &(data[k]);
        }
        else
        {
            return NULL;
        }
    }
    static MappedType* findEdit(TDataType& data, const KeyType& k)
    {
        if (k < static_cast<KeyType>(data.size()))
        {
            return &(data[k]);
        }
        else
        {
            return NULL;
        }
    }
    static bool erase(TDataType& data, KeyType k)
    {
        return false;
    }
    static MappedType* insert(TDataType& data, const KeyType& k)
    {
        if (k >= static_cast<KeyType>(data.size()))
        {
            if (TFixedSize==0)
            {
                resizeIfNotFixed(data, k+1, std::integral_constant<bool, TFixedSize!=0>());
            }
            else
            {
                return NULL; // fixed size, resize is not available
            }
        }
        return &(data[k]);
    }
    static void clear(TDataType& data)
    {
        clearBasedOnFixed(data, std::integral_constant<bool, TFixedSize!=0>());
    }
    static void reserve(TDataType& data, size_t reserve)
    {
        reserveIfNotFixed(data, reserve, std::integral_constant<bool, TFixedSize!=0>());
    }
    static void resize(TDataType& data, size_t size)
    {
        resizeIfNotFixed(data, size, std::integral_constant<bool, TFixedSize!=0>());
    }

protected:
    template<class T>
    static void clearBasedOnFixed(T& data, std::false_type)
    {
        DataTypeInfo_ContainerClear(data);
    }
    template<class T>
    static void reserveIfNotFixed(T& data, size_t reserve, std::false_type)
    {
        DataTypeInfo_ContainerReserve(data, reserve);
    }
    template<class T>
    static void resizeIfNotFixed(T& data, size_t size, std::false_type)
    {
        DataTypeInfo_ContainerResize(data, size);
    }
    template<class T>
    static void clearBasedOnFixed(T& data, std::true_type)
    {
        DataTypeInfo_Clear(data);
    }
    template<class T>
    static void reserveIfNotFixed(T& /*data*/, size_t /*reserve*/, std::true_type)
    {
    }
    template<class T>
    static void resizeIfNotFixed(T& /*data*/, size_t /*size*/, std::true_type)
    {
    }
};

template<class TDataType> struct DataTypeInfo_ContainerTypes<TDataType, ContainerKindEnum::Set, 0>
{
    typedef typename TDataType::key_type KeyType;
    typedef typename TDataType::value_type MappedType;
    typedef typename TDataType::const_iterator const_iterator;
    typedef typename TDataType::iterator       iterator;
    typedef const KeyType& KeyTypeReturn;
    static const MappedType& value(const TDataType&, const const_iterator& it) { return *it; }
    static const KeyType&    key  (const TDataType&, const const_iterator& it) { return *it; }
    static const KeyType&    key  (const TDataType&, const const_iterator& it, TypeInfoKeyBuffer& keyBuffer) { return *it; }
    static const MappedType& valueAtIndex(const TDataType& data, size_t index)
    {
        const_iterator it = data.cbegin();
        std::advance(it, index);
        return *it;
    }
    static const KeyType& keyAtIndex(const TDataType& data, size_t index)
    {
        const_iterator it = data.cbegin();
        std::advance(it, index);
        return *it;
    }
    static const KeyType& keyAtIndex(const TDataType& data, size_t index, TypeInfoKeyBuffer& keyBuffer)
    {
        return keyAtIndex(data, index);
    }
    static const MappedType* find(const TDataType& data, const KeyType& k)
    {
        const_iterator it = data.find(k);
        return (it == data.end()) ? NULL : &(*it);
    }
    static MappedType* findEdit(TDataType& /*data*/, const KeyType& /*k*/)
    {
        return NULL;
    }
    static MappedType* insert(TDataType& data, const KeyType& k)
    {
        data.insert(k);
        return NULL;
    }
    static bool erase(TDataType& data, KeyType k)
    {
        return data.erase(k) != 0;
    }
    static void clear(TDataType& data)
    {
        DataTypeInfo_ContainerClear(data);
    }
    static void reserve(TDataType& /*data*/, size_t /*reserve*/)
    {
    }
    static void resize(TDataType& /*data*/, size_t /*size*/)
    {
    }
};

template<class TDataType> struct DataTypeInfo_ContainerTypes<TDataType, ContainerKindEnum::Map, 0>
{
    typedef typename TDataType::key_type KeyType;
    typedef typename TDataType::mapped_type MappedType;
    typedef typename TDataType::const_iterator const_iterator;
    typedef typename TDataType::iterator       iterator;
    static const MappedType& value(const TDataType&, const const_iterator& it) { return it->second; }
    static       MappedType& value(      TDataType&, const       iterator& it) { return it->second; }
    static const KeyType& key(const TDataType& data, const const_iterator& it) { return it->first; }
    static const KeyType& key(const TDataType& data, const const_iterator& it, TypeInfoKeyBuffer&) { return it->first; }
    static const MappedType& valueAtIndex(const TDataType& data, size_t index)
    {
        const_iterator it = data.cbegin();
        std::advance(it, index);
        return it->second;
    }
    static       MappedType& valueAtIndex(      TDataType& data, size_t index)
    {
        iterator it = data.begin();
        std::advance(it, index);
        return it->second;
    }
    static const KeyType& keyAtIndex(const TDataType& data, size_t index)
    {
        const_iterator it = data.cbegin();
        std::advance(it, index);
        return it->first;
    }
    static const KeyType& keyAtIndex(const TDataType& data, size_t index, TypeInfoKeyBuffer&)
    {
        const_iterator it = data.cbegin();
        std::advance(it, index);
        return it->first;
    }
    static const MappedType* find(const TDataType& data, const KeyType& k)
    {
        const_iterator it = data.find(k);
        return (it == data.end()) ? NULL : &(it->second);
    }
    static MappedType* findEdit(TDataType& data, const KeyType& k)
    {
        iterator it = data.find(k);
        return (it == data.end()) ? NULL : &(it->second);
    }
    static MappedType* insert(TDataType& data, const KeyType& k)
    {
        return &(data[k]);
    }
    static bool erase(TDataType& data, KeyType k)
    {
        return data.erase(k)!=0;
    }
    static void clear(TDataType& data)
    {
        DataTypeInfo_ContainerClear(data);
    }
    static void reserve(TDataType& /*data*/, size_t /*reserve*/)
    {
    }
    static void resize(TDataType& /*data*/, size_t /*size*/)
    {
    }
};

template<class TDataType, ContainerKindEnum TContainerKind, size_t TFixedSize = 0>
struct IsContainerMultiValue
{
    typedef DataTypeInfo_ContainerTypes<TDataType, TContainerKind, TFixedSize> ContainerTypes;
    typedef typename ContainerTypes::MappedType MappedType;      ///< type contained in DataType
    typedef DataTypeInfo<MappedType> MappedTypeInfo;

    static constexpr bool value =
        (TContainerKind==ContainerKindEnum::Array ||
         (TContainerKind==ContainerKindEnum::Set && MappedTypeInfo::FinalSize == 1)) &&
        MappedTypeInfo::IsMultiValue && MappedTypeInfo::FixedFinalSize;
};

template<class TDataType, ContainerKindEnum TContainerKind, size_t TFixedSize, bool TMultiValue>
struct ContainerMultiValueTypeInfo;

template<class TDataType, ContainerKindEnum TContainerKind, size_t TFixedSize>
struct ContainerMultiValueTypeInfo<TDataType, TContainerKind, TFixedSize, false>
{
    typedef TDataType DataType;

    /// true if this type is equivalent to multiple values (either single value or a composition of arrays of the same type of values)
    static constexpr bool IsMultiValue = false;
    /// true if this type has a fixed size for all level until the final values
    static constexpr bool FixedFinalSize = false;
    typedef void FinalValueType;  ///< type of the final atomic values (or void if not applicable)
    static constexpr ValueKindEnum     FinalValueKind = ValueKindEnum::Void;
    static constexpr size_t FinalSize = 1; ///< 0, or fixed final size if FixedFinalSize is 1
    static size_t finalSize(const DataType& data)
    {
        return 0;
    }
};

template<class TDataType, ContainerKindEnum TContainerKind, size_t TFixedSize>
struct ContainerMultiValueTypeInfo<TDataType, TContainerKind, TFixedSize, true>
{
    typedef TDataType DataType;
    typedef DataTypeInfo_ContainerTypes<TDataType, TContainerKind, TFixedSize> ContainerTypes;
    typedef typename ContainerTypes::KeyType    KeyType;         ///< type uniquely identifying items (or void if not applicable)
    typedef typename ContainerTypes::MappedType MappedType;      ///< type contained in DataType

    typedef DataTypeInfo<KeyType> KeyTypeInfo;
    typedef DataTypeInfo<MappedType> MappedTypeInfo;

    /// true if this type is equivalent to multiple values (either single value or a composition of arrays of the same type of values)
    static constexpr bool IsMultiValue = true;
    /// true if this type has a fixed size for all level until the final values
    static constexpr bool FixedFinalSize     = TFixedSize>0 && KeyTypeInfo::FixedFinalSize && MappedTypeInfo::FixedFinalSize;

    typedef typename MappedTypeInfo::FinalValueType  FinalValueType;  ///< type of the final atomic values (or void if not applicable)
    static constexpr ValueKindEnum     FinalValueKind = MappedTypeInfo::FinalValueKind;


    static constexpr size_t FinalSize = (TFixedSize>0 ? TFixedSize : 1)*MappedTypeInfo::FinalSize; ///< 0, or fixed final size if FixedFinalSize is 1
    static size_t finalSize(const DataType& data)
    {
        if (FixedFinalSize)
        {
            return FinalSize;
        }
        else
        {
            return data.size() * MappedTypeInfo::FinalSize;
        }
    }

    static void setFinalSize(DataType& data, size_t size)
    {
        if (!TFixedSize && TContainerKind == ContainerKindEnum::Array)
        {
            ContainerTypes::resize(data, size / MappedTypeInfo::FinalSize);
        }
    }

    template <typename T>
    static void getFinalValue(const DataType& data, size_t index, T& value)
    {
        MappedTypeInfo::getFinalValue(ContainerTypes::valueAtIndex(data, index / MappedTypeInfo::FinalSize),
                                      index % MappedTypeInfo::FinalSize, value);
    }

    static void getFinalValueString(const DataType& data, size_t index, std::string& value)
    {
        MappedTypeInfo::getFinalValueString(ContainerTypes::valueAtIndex(data, index / MappedTypeInfo::FinalSize),
                                            index % MappedTypeInfo::FinalSize, value);
    }

protected:

    template<typename T>
    static void setFinalValueIfSet(DataType& data, size_t index, const T& value, std::false_type)
    {
        MappedTypeInfo::setFinalValue(ContainerTypes::valueAtIndex(data, index / MappedTypeInfo::FinalSize),
                                      index % MappedTypeInfo::FinalSize, value);
    }

    template<typename T>
    static void setFinalValueIfSet(DataType& data, size_t index, const T& value, std::true_type)
    {
        KeyType t;
        KeyTypeInfo::setDataValue(t, value);
        ContainerTypes::insert(data, t);
    }

    template<typename T>
    static void setFinalValueStringIfSet(DataType& data, size_t index, const T& value, std::false_type)
    {
        MappedTypeInfo::setFinalValueString(ContainerTypes::valueAtIndex(data, index / MappedTypeInfo::FinalSize),
                                      index % MappedTypeInfo::FinalSize, value);
    }

    template<typename T>
    static void setFinalValueStringIfSet(DataType& data, size_t index, const T& value, std::true_type)
    {
        KeyType t;
        KeyTypeInfo::setDataValueString(t, value);
        ContainerTypes::insert(data, t);
    }

public:
    template<typename T>
    static void setFinalValue(DataType& data, size_t index, const T& value)
    {
        setFinalValueIfSet(data, index, value, std::integral_constant<bool, TContainerKind == ContainerKindEnum::Set>());
    }

    static void setFinalValueString(DataType& data, size_t index, const std::string& value)
    {
        setFinalValueStringIfSet(data, index, value, std::integral_constant<bool, TContainerKind == ContainerKindEnum::Set>());
    }
};

template<class TDataType, ContainerKindEnum TContainerKind, size_t TFixedSize = 0>
struct ContainerTypeInfo : public ContainerMultiValueTypeInfo<TDataType, TContainerKind, TFixedSize,
    IsContainerMultiValue<TDataType, TContainerKind, TFixedSize>::value>
{
    typedef TDataType DataType;
    typedef DataTypeInfo_ContainerTypes<TDataType, TContainerKind, TFixedSize> ContainerTypes;
    typedef typename ContainerTypes::KeyType    KeyType;         ///< type uniquely identifying items (or void if not applicable)
    typedef typename ContainerTypes::MappedType MappedType;      ///< type contained in DataType

    typedef DataTypeInfo<KeyType> KeyTypeInfo;
    typedef DataTypeInfo<MappedType> MappedTypeInfo;

    static constexpr ContainerKindEnum ContainerKind  = TContainerKind;

    static constexpr bool IsContainer        = true;   ///< true if this type is a container
    static constexpr bool IsSingleValue      = false;  ///< true if this type is a single value
    //static constexpr bool IsMultiValue       =
    //    (TContainerKind==ContainerKindEnum::Array ||
    //     (TContainerKind==ContainerKindEnum::Set && MappedTypeInfo::FinalSize == 1)) &&
    //    MappedTypeInfo::IsMultiValue && MappedTypeInfo::FixedFinalSize;
    static constexpr bool IsStructure        = false;  ///< true if this type is a structure

    static constexpr bool ValidInfo          = MappedTypeInfo::ValidInfo; ///< true if this type has valid infos
    static constexpr bool Integer            = MappedTypeInfo::Integer;   ///< true if this type uses integer values
    static constexpr bool Scalar             = MappedTypeInfo::Scalar;    ///< true if this type uses scalar values
    static constexpr bool String             = MappedTypeInfo::String;    ///< true if this type uses text values
    static constexpr bool Unsigned           = MappedTypeInfo::Unsigned;  ///< true if this type is unsigned
    static constexpr bool FixedContainerSize = TFixedSize>0;  ///< true if this type has a fixed size for this container level

    /// true if the constructor is equivalent to setting memory to 0
    static constexpr bool ZeroConstructor    = (TContainerKind==ContainerKindEnum::Array && TFixedSize>0 && MappedTypeInfo::ZeroConstructor);
    /// true if copying the data can be done with a memcpy
    static constexpr bool SimpleCopy         = (TContainerKind==ContainerKindEnum::Array && MappedTypeInfo::SimpleCopy);
    /// true if the layout in memory is simply N values of the same base type
    static constexpr bool SimpleLayout       = (TContainerKind==ContainerKindEnum::Array && MappedTypeInfo::SimpleLayout);
    /// true if this type uses copy-on-write
    static constexpr bool CopyOnWrite        = !(TContainerKind==ContainerKindEnum::Array && TFixedSize>0 && TFixedSize < 10);
    /// true if the item keys are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)
    static constexpr bool StoreKeys          = (TContainerKind==ContainerKindEnum::Set || TContainerKind==ContainerKindEnum::Map);
    /// true if the item values are stored within the data structure
    static constexpr bool StoreValues        = (TContainerKind!=ContainerKindEnum::Set);

    static constexpr size_t ContainerSize = TFixedSize>0 ? TFixedSize : 1; ///< 0, or fixed container size if FixedContainerSize is 1
    static constexpr size_t ByteSize = SimpleCopy && FixedContainerSize ? TFixedSize * MappedTypeInfo::ByteSize : 0; ///< if known at compile time, the size in bytes of the DataType, else 0

    static std::string name()
    {
        return DataTypeName<TDataType>::name();
    }

    static size_t containerSize(const DataType& data)
    {
        if (FixedContainerSize)
        {
            return ContainerSize;
        }
        else
        {
            return data.size();
        }
    }
    static size_t byteSize(const DataType& data)
    {
        return SimpleCopy ? containerSize(data) * MappedTypeInfo::ByteSize : 0;
    }

    static const void* getValuePtr(const DataType& data)
    {
        return getValuePtr(data, std::integral_constant<bool, SimpleCopy>());
    }
protected:
    static const void* getValuePtr(const DataType& data, std::true_type)
    {
        //return containerSize(data) > 0 ? data.data() : nullptr; // data() member method is missing on some types, so we need to use iterators
        return containerSize(data) > 0 ? std::addressof(*data.cbegin()) : nullptr;
    }
    static const void* getValuePtr(const DataType& /*data*/, std::false_type)
    {
        return nullptr;
    }
    
public:
    static void resetValue(DataType& data, size_t reserve = 0)
    {
        ContainerTypes::clear(data);
        if (reserve > 0)
        {
            ContainerTypes::reserve(data, reserve);
        }
    }

    static void getDataValueString(const DataType& data, std::string& value)
    {
        DataTypeInfo_ToString(data, value);
    }

    static void setDataValueString(DataType& data, const std::string& value)
    {
        DataTypeInfo_FromString(data, value);
    }

    static void setContainerSize(DataType& data, size_t size)
    {
        ContainerTypes::resize(data, size);
    }

    static const MappedType& getItemValue(const DataType& data, size_t index)
    {
        return ContainerTypes::valueAtIndex(data, index);
    }

    static auto getItemKey(const DataType& data, size_t index) -> decltype(ContainerTypes::keyAtIndex(data, index))
    {
        return ContainerTypes::keyAtIndex(data, index);
    }

    static const KeyType& getItemKey(const DataType& data, size_t index, TypeInfoKeyBuffer& keyBuffer)
    {
        return ContainerTypes::keyAtIndex(data, index, keyBuffer);
    }

    static MappedType* editItemValue(DataType& data, size_t index)
    {
        return editItemValueIf(data, index, std::integral_constant<bool, StoreValues>());
    }
protected:
    template<class T>
    static MappedType* editItemValueIf(T& data, size_t index, std::true_type)
    {
        return &ContainerTypes::valueAtIndex(data, index);
    }
    template<class T>
    static MappedType* editItemValueIf(T& data, size_t index, std::false_type)
    {
        return NULL;
    }
public:

    static void getItemValueString(const DataType& data, size_t index, std::string& value)
    {
        MappedTypeInfo::getDataValueString(getItemValue(data, index), value);
    }

    static void setItemValueString(DataType& data, size_t index, const std::string& value)
    {
        if (StoreValues)
        {
            MappedTypeInfo::setDataValueString(*editItemValue(data, index), value);
        }
        else // std::set can only write keys
        {
            KeyType key;
            KeyTypeInfo::getDataValueString(key, value);
            insertItem(data, key);
        }
    }

    template<typename T>
    static void insertItemValueString(DataType& data, const KeyType& key, const std::string& value)
    {
        MappedType* mappedValue = insertItem(data, key);
        if (StoreValues)
        {
            MappedTypeInfo::setDataValueString(mappedValue, value);
        }
    }

    static const MappedType* findItem(const DataType& data, const KeyType& key)
    {
        return ContainerTypes::find(data, key);
    }

    static MappedType* findEditItem(DataType& data, const KeyType& key)
    {
        return ContainerTypes::findEdit(data, key);
    }

    static MappedType* insertItem(DataType& data, const KeyType& key)
    {
        return ContainerTypes::insert(data, key);
    }

    static bool eraseItem(DataType& data, const KeyType& key)
    {
        return ContainerTypes::erase(data, key);
    }

    
    typedef typename ContainerTypes::iterator iterator;
    typedef typename ContainerTypes::const_iterator const_iterator;
    static iterator begin(DataType& data)
    {
        return data.begin();
    }
    static iterator end(DataType& data)
    {
        return data.end();
    }
    static const_iterator cbegin(const DataType& data)
    {
        return data.cbegin();
    }
    static const_iterator cend(const DataType& data)
    {
        return data.cend();
    }

    static auto value(const DataType& data, const const_iterator& it) -> decltype(ContainerTypes::value(data, it))
    {
        return ContainerTypes::value(data, it);
    }

    static auto key(const DataType& data, const const_iterator& it) -> decltype(ContainerTypes::key(data, it))
    {
        return ContainerTypes::key(data, it);
    }

    static const KeyType& key(const DataType& data, const const_iterator& it, TypeInfoKeyBuffer& keyBuffer)
    {
        return ContainerTypes::key(data, it, keyBuffer);
    }

    static auto value(DataType& data, const iterator& it) -> decltype(ContainerTypes::value(data, it))
    {
        return ContainerTypes::value(data, it);
    }

    static auto key(DataType& data, const iterator& it) -> decltype(ContainerTypes::key(data, it))
    {
        return ContainerTypes::key(data, it);
    }

    static const KeyType& key(DataType& data, const iterator& it, TypeInfoKeyBuffer& keyBuffer)
    {
        return ContainerTypes::key(data, it, keyBuffer);
    }
};


template<class T, std::size_t N>
struct DataTypeInfo< std::array<T,N> > : public ContainerTypeInfo<std::array<T,N>, ContainerKindEnum::Array, N> {};
template<class T, std::size_t N>
struct DataTypeName< std::array<T,N> > { static std::string name() { std::ostringstream o; o << "std::array<" << DataTypeName<T>::name() << "," << N << ">"; return o.str(); } };

template<class T, std::size_t N>
struct DataTypeInfo< sofa::helper::fixed_array<T,N> > : public ContainerTypeInfo<sofa::helper::fixed_array<T,N>, ContainerKindEnum::Array, N> {};
template<class T, std::size_t N>
struct DataTypeName< sofa::helper::fixed_array<T,N> > { static std::string name() { std::ostringstream o; o << "fixed_array<" << DataTypeName<T>::name() << "," << N << ">"; return o.str(); } };

template<class T, class Alloc>
struct DataTypeInfo< std::vector<T,Alloc> > : public ContainerTypeInfo<std::vector<T,Alloc>, ContainerKindEnum::Array, 0> {};
template<class T, class Alloc>
struct DataTypeName< std::vector<T,Alloc> > { static std::string name() { std::ostringstream o; o << "std::vector<" << DataTypeName<T>::name() << ">"; return o.str(); } };

template<class T, class Alloc>
struct DataTypeInfo< sofa::helper::vector<T,Alloc> > : public ContainerTypeInfo<sofa::helper::vector<T,Alloc>, ContainerKindEnum::Array, 0> {};
template<class T, class Alloc>
struct DataTypeName< sofa::helper::vector<T,Alloc> > { static std::string name() { std::ostringstream o; o << "vector<" << DataTypeName<T>::name() << ">"; return o.str(); } };

// specialization for vector<bool>, container API not supported due to inability to get pointer to contained values, only multivalue API
//template<class Alloc>
//struct DataTypeInfo< std::vector<bool, Alloc> > : public InvalidDataTypeInfo< std::vector<bool, Alloc> > {};
//template<class Alloc>
//struct DataTypeInfo< sofa::helper::vector<bool, Alloc> > : public InvalidDataTypeInfo< sofa::helper::vector<bool, Alloc> > {};
template<class Alloc>
struct DataTypeInfo< sofa::helper::vector<bool, Alloc> > : public MultiValueTypeInfo<sofa::helper::vector<bool, Alloc>, bool, 0> {};
template<class Alloc>
struct DataTypeInfo< std::vector<bool, Alloc> > : public MultiValueTypeInfo < std::vector<bool, Alloc>, bool, 0 > {};

template<class T, class Compare, class Alloc>
struct DataTypeInfo< std::set<T,Compare,Alloc> > : public ContainerTypeInfo<std::set<T,Compare,Alloc>, ContainerKindEnum::Set, 0> {};
template<class T, class Compare, class Alloc>
struct DataTypeName< std::set<T,Compare,Alloc> > { static std::string name() { std::ostringstream o; o << "std::set<" << DataTypeName<T>::name() << ">"; return o.str(); } };

template<class T, class Compare, class Alloc>
struct DataTypeInfo< sofa::helper::set<T,Compare,Alloc> > : public ContainerTypeInfo<sofa::helper::set<T,Compare,Alloc>, ContainerKindEnum::Set, 0> {};
template<class T, class Compare, class Alloc>
struct DataTypeName< sofa::helper::set<T,Compare,Alloc> > { static std::string name() { std::ostringstream o; o << "set<" << DataTypeName<T>::name() << ">"; return o.str(); } };

template<class K, class T, class Compare, class Alloc>
struct DataTypeInfo< std::map<K,T,Compare,Alloc> > : public ContainerTypeInfo<std::map<K,T,Compare,Alloc>, ContainerKindEnum::Map, 0> {};
template<class K, class T, class Compare, class Alloc>
struct DataTypeName< std::map<K,T,Compare,Alloc> > { static std::string name() { std::ostringstream o; o << "std::map<" << DataTypeName<K>::name() << "," << DataTypeName<T>::name() << ">"; return o.str(); } };


} // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_DATATYPEINFO_H
