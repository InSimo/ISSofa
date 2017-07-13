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
#ifndef SOFA_DEFAULTTYPE_QUANTITYTYPEINFO_H
#define SOFA_DEFAULTTYPE_QUANTITYTYPEINFO_H


#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/defaulttype/Units.h>
#include <array>

namespace sofa
{

namespace defaulttype
{

template <typename T, int kg, int m, int s, int A, int K, int mol, int cd, typename Enable = void>
class QuantityTypeInfo : public DataTypeInfo<T> {};


template <typename TDataType, int kg, int m, int s, int A, int K, int mol, int cd>
class QuantityTypeInfo<TDataType, kg, m, s, A, K, mol, cd, typename std::enable_if<DataTypeInfo<TDataType>::IsSingleValue>::type> : public DataTypeInfo<TDataType>
{
public:
    typedef units::Quantity<TDataType, kg, m, s, A, K, mol, cd> DataType;


    static constexpr size_t finalSize(const DataType& /*data*/) { return DataTypeInfo<TDataType>::FinalSize; }
    static constexpr size_t byteSize(const DataType& /*data*/)
    {
        return byteSize(DataType());
    }

    static const void* getValuePtr(const DataType& data)
    {
        return getValuePtr(data);
    }
protected:
    static const void* getValuePtr(const DataType& data, std::true_type)
    {
        return nullptr;
    }
    static const void* getValuePtr(const DataType& data, std::false_type)
    {
        return &data.value();
    }
public:

    static void resetValue(DataType& data, size_t /*reserve*/ = 0)
    {
        data = DataType();
    }

    // Single Value API

    template <typename DataTypeRef, typename T>
    static void getDataValue(const DataTypeRef& data, T& value)
    {
        value = data.value();
    }

    template<typename DataTypeRef, typename T>
    static void setDataValue(DataTypeRef&& data, const T& value)
    {
        data = value;
    }
    

    // Multi Value API
    static void setFinalSize(DataType& /*data*/, size_t /*size*/)
    {
    }

    template <typename DataTypeRef, typename T>
    static void getFinalValue(const DataTypeRef& data, size_t index, T& value)
    {
        value = data.value();
    }

    template<typename DataTypeRef, typename T>
    static void setFinalValue(DataTypeRef&& data, size_t index, const T& value)
    {
        data = value;
    }
};



template <typename TDataType, int kg, int m, int s, int A, int K, int mol, int cd>
class QuantityTypeInfo<TDataType, kg, m, s, A, K, mol, cd, typename std::enable_if<DataTypeInfo<TDataType>::IsContainer>::type> : public DataTypeInfo<TDataType>
{
public:

    using QuantityDataType = units::Quantity<TDataType, kg, m, s, A, K, mol, cd>;
    using ValueDataType = TDataType;
    using ValueInfo = DataTypeInfo<TDataType>;

    static void resetValue(QuantityDataType& data, size_t reserve = 0)
    {
        ValueInfo::resetValue(data.value(), reserve);
    }

    static void getDataValueString(const QuantityDataType& data, std::string& value)
    {
        ValueInfo::getDataValueString(data.value(), value);
    }

    static void setDataValueString(QuantityDataType& data, const std::string& value)
    {
        ValueInfo::setDataValueString(data.value(), value);
    }

    static constexpr size_t byteSize(const QuantityDataType& data)
    {
        return ValueInfo::byteSize(data.value());
    }

    static size_t containerSize(const QuantityDataType& data)
    {
        return ValueInfo::containerSize(data.value());
    }

    static void setContainerSize(QuantityDataType& data, size_t size)
    {
        ValueInfo::setContainerSize(data.value(), size);
    }

    static auto getItemKey(const QuantityDataType& data, size_t index) -> decltype(ValueInfo::getItemKey(data.value(), index))
    {
        return ValueInfo::getItemKey(data.value(), index);
    }

    static const typename ValueInfo::KeyType& getItemKey(const QuantityDataType& data, size_t index, TypeInfoKeyBuffer& keyBuffer)
    {
        return ValueInfo::getItemKey(data.value(), index,keyBuffer);
    }

    static const typename ValueInfo::MappedType& getItemValue(const QuantityDataType& data, size_t index)
    {
        return ValueInfo::getItemValue(data.value(), index);
    }

    static typename ValueInfo::MappedType* editItemValue(QuantityDataType& data, size_t index)
    {
        return ValueInfo::editItemValue(data.value(), index);
    }

    static const void* getValuePtr(const QuantityDataType& data)
    {
        return ValueInfo::getValuePtr(data.value());
    }

    static const typename ValueInfo::MappedType* findItem(const QuantityDataType& data, const typename ValueInfo::KeyType& key)
    {
        return ValueInfo::findItem(data.value(), key);
    }

    static typename ValueInfo::MappedType* findEditItem(QuantityDataType& data, const typename ValueInfo::KeyType& key)
    {
        return ValueInfo::findEditItem(data.value(), key);
    }

    static typename ValueInfo::MappedType* insertItem(QuantityDataType& data, const typename ValueInfo::KeyType& key)
    {
        return ValueInfo::insertItem(data.value(), key);
    }

    static bool eraseItem(QuantityDataType& data, const typename ValueInfo::KeyType& key)
    {
        return ValueInfo::eraseItem(data.value(), key);
    }

    static const typename ValueInfo::KeyType& key(const QuantityDataType& data, const typename ValueInfo::const_iterator& it, TypeInfoKeyBuffer& keyBuffer)
    {
        return ValueInfo::key(data.value(), it, keyBuffer);
    }

    static auto value(const QuantityDataType& data, const typename ValueInfo::const_iterator& it) -> decltype(ValueInfo::value(data.value(), it))
    {
        return ValueInfo::value(data.value(), it);
    }

    static auto value(QuantityDataType& data, const typename ValueInfo::iterator& it) -> decltype(ValueInfo::value(data.value(), it))
    {
        return ValueInfo::value(data.value(), it);
    }

    static size_t finalSize(const QuantityDataType& data)
    {
        return ValueInfo::finalSize(data.value());
    }

    static void setFinalSize(QuantityDataType& data, size_t size)
    {
        ValueInfo::setFinalSize(data.value(), size);
    }

    template <typename T>
    static void getFinalValue(const QuantityDataType& data, size_t index, T& value)
    {
        ValueInfo::getFinalValue(data.value(), index, value);
    }

    template<typename T>
    static void setFinalValue(QuantityDataType& data, size_t index, const T& value)
    {
        ValueInfo::setFinalValue(data.value(), index, value);
    }

    static void getFinalValueString(const QuantityDataType& data, size_t index, std::string& value)
    {
        ValueInfo::getFinalValueString(data.value(), index, value);
    }

    static void setFinalValueString(QuantityDataType& data, size_t index, const std::string& value)
    {
        ValueInfo::setFinalValueString(data.value(), index, value);
    }

};



template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
struct DataTypeInfo<units::Quantity<T, kg, m, s, A, K, mol, cd> > : public QuantityTypeInfo<T, kg, m, s, A, K, mol, cd>
{
    typedef std::tuple<meta::Units, meta::ReadOnly> MetaTuple;
    static constexpr MetaTuple Metadata = MetaTuple{ meta::Units(std::array<int, 7>{kg, m, s, A, K, mol, cd} ), meta::ReadOnly() };

    class AddMetaI
    {
    public:
        AddMetaI(std::map<int, defaulttype::AbstractMetadata*>& value) : m_map(value) {}

        template <typename T1, typename T2>
        void operator()(T1&& MemberTypeI, T2&& MemberTypeI2) const
        {
            auto abstractMetadata = new defaulttype::VirtualMetadata<T1>(MemberTypeI2);
            int id = abstractMetadata->getId();
            m_map[id] = abstractMetadata;
        }
    private:
        std::map<int, defaulttype::AbstractMetadata*>& m_map;
    };

    template<size_t Index>
    using MemberType = typename std::tuple_element<Index, MetaTuple>::type;

    template<class Tuple, std::size_t I>
    class TupleForEach
    {
    public:
        template <typename F>
        static void loop(F&& f)
        {
            f(MemberType<I>{}, std::get<I> (DataTypeInfo<units::Quantity<T, kg, m, s, A, K, mol, cd>>::Metadata));
            TupleForEach<Tuple, I - 1>::loop(std::forward<F>(f));
        }

    };

    template<class Tuple>
    class TupleForEach<Tuple, 0>
    {
    public:
        template <typename F>
        static void loop(F&& f)
        {
            f(MemberType<0>{}, std::get<0>(DataTypeInfo<units::Quantity<T, kg, m, s, A, K, mol, cd>>::Metadata));
        }
    };

    static std::map<int, defaulttype::AbstractMetadata*> getMetadata()
    {
        std::map<int, defaulttype::AbstractMetadata*> metadataMap;

        // TODO Circular Dependencies between VirtualTypeInfo and QuantityTypeInfo, need to find another design

        /*AddMetaI functor = AddMetaI(metadataMap);
        TupleForEach<MetaTuple, std::tuple_size<MetaTuple>::value -1>::loop(std::forward<AddMetaI>(functor));*/
        return metadataMap;
    }

};


template<class T, int kg, int m, int s, int A, int K, int mol, int cd> 
struct DataTypeName<units::Quantity<T, kg, m, s, A, K, mol, cd> > { static const char* name() { return DataTypeName<T>::name(); } };


} // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_QUANTITYTYPEINFO_H
