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
    static const void* getValuePtr(const TDataType& data)
    {
        return DataTypeInfo<TDataType>::getValuePtr(data.value());
    }

//    typedef units::Quantity<TDataType, kg, m, s, A, K, mol, cd> DataType;    
//    typedef typename DataTypeInfo<TDataType>::ContainerTypes DContainerTypes;
//
//    typedef typename TDataType::size_type      KeyType;
//    typedef typename TDataType::const_iterator const_iterator;
//    typedef typename TDataType::iterator       iterator;
//    typedef sofa::helper::SSOBuffer<> TypeInfoKeyBuffer;
//
//    static auto value(const DataType&, const const_iterator& it) -> decltype(*it) { return *it; }
//    static auto value(DataType&, const       iterator& it) -> decltype(*it) { return *it; }
//    static KeyType key(const DataType& data, const const_iterator& it) { return (it - data.begin()); }
//    static const KeyType& key(const DataType& data, const const_iterator& it, TypeInfoKeyBuffer& keyBuffer)
//    {
//        KeyType& key = *keyBuffer.getOrCreate<KeyType>();
//        key = (it - data.begin());
//        return key;
//    }

};



template<class T, int kg, int m, int s, int A, int K, int mol, int cd>
struct DataTypeInfo<units::Quantity<T, kg, m, s, A, K, mol, cd> > : public QuantityTypeInfo<T, kg, m, s, A, K, mol, cd>
{
    typedef std::tuple<meta::Units, meta::ReadOnly> MetaTuple;
    static constexpr MetaTuple Metadata = { meta::Units(std::array<int, 7>{kg, m, s, A, K, mol, cd} ), meta::ReadOnly() };

    class AddMetaI
    {
    public:
        class AddMetaI(std::map<int, defaulttype::AbstractMetadata*>& value) : m_map(value) {}

        template <typename T, typename T2>
        void operator()(T&& MemberTypeI, T2&& MemberTypeI2) const
        {
            auto abstractMetadata = new defaulttype::VirtualMetadata<T>(MemberTypeI2);
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
            f(MemberType<I>{}, std::get<I> (Metadata));
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
            f(MemberType<0>{}, std::get<0>(Metadata));
        }
    };

    static std::map<int, defaulttype::AbstractMetadata*> getMetadata()
    {
        std::map<int, defaulttype::AbstractMetadata*> metadataMap;
        AddMetaI functor = AddMetaI(metadataMap);
        TupleForEach<MetaTuple, std::tuple_size<MetaTuple>::value -1 >::loop(std::forward<AddMetaI>(functor));
        return metadataMap;
    }

};


template<class T, int kg, int m, int s, int A, int K, int mol, int cd> 
struct DataTypeName<units::Quantity<T, kg, m, s, A, K, mol, cd> > { static const char* name() { return DataTypeName<T>::name(); } };


} // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_QUANTITYTYPEINFO_H
