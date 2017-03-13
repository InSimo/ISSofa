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
#ifndef SOFA_DEFAULTTYPE_VIRTUALTYPEINFO_H
#define SOFA_DEFAULTTYPE_VIRTUALTYPEINFO_H

#include "AbstractTypeInfo.h"
#include "DataTypeInfo.h"
#include <typeinfo>

namespace sofa
{

namespace defaulttype
{

template<class TDataType, bool TSingleValue, bool TMultiValue, bool TContainer>
class VirtualTypeInfoImpl;

template<class TDataType>
class VirtualTypeInfo : public VirtualTypeInfoImpl<TDataType,
    DataTypeInfo<TDataType>::IsSingleValue,
    DataTypeInfo<TDataType>::IsMultiValue,
    DataTypeInfo<TDataType>::IsContainer>
{
public:
    typedef TDataType DataType;
    typedef DataTypeInfo<DataType> Info;

    static VirtualTypeInfo<DataType>* get() { static VirtualTypeInfo<DataType> t; return &t; }

protected:
    VirtualTypeInfo() = default;
    ~VirtualTypeInfo() = default;
};

template<class TDataType, class TBaseClass>
class AbstractTypeInfoImpl : public TBaseClass
{
public:
    typedef TDataType DataType;
    typedef DataTypeInfo<TDataType> Info;
    
    virtual bool ValidInfo() const override { return Info::ValidInfo; }

    virtual std::string name() const override                { return DataTypeName<DataType>::name(); }
    virtual ContainerKindEnum ContainerKind() const override { return Info::ContainerKind;  }
    virtual ValueKindEnum FinalValueKind() const override    { return Info::FinalValueKind; }
    
    virtual bool IsSingleValue() const override { return Info::IsSingleValue; }
    virtual bool IsContainer() const override   { return Info::IsContainer;   }
    virtual bool IsMultiValue() const override  { return Info::IsMultiValue;  }

    virtual bool ZeroConstructor() const override { return Info::ZeroConstructor; }
    virtual bool SimpleCopy() const override      { return Info::SimpleCopy;      }
    virtual bool SimpleLayout() const override    { return Info::SimpleLayout;    }
    virtual bool CopyOnWrite() const override     { return Info::CopyOnWrite;     }

    virtual void resetValue(void* data, size_t reserve = 0) const override
    {
        Info::resetValue(*(DataType*)data, reserve);
    }
    virtual void getDataValueString(const void* data, std::string& value) const override
    {
        Info::getDataValueString(*(const DataType*)data, value);
    }
    virtual void setDataValueString(void* data, const std::string& value) const override
    {
        Info::setDataValueString(*(DataType*)data, value);
    }

    virtual bool getAvailableItems(std::vector<std::string>& /*result*/, const void* /*data*/) const override
    {
        // @TODO support for enums
        return false;
    }
    virtual const std::type_info* type_info() const override { return &typeid(DataType); }
};

template<class TDataType>
class VirtualTypeInfoImpl<TDataType, false, false, false> : public AbstractTypeInfoImpl<TDataType, AbstractTypeInfo>
{
public:
    typedef TDataType DataType;
    typedef DataTypeInfo<DataType> Info;

    // Base API

    virtual const AbstractValueTypeInfo*      SingleValueType() const override { return NULL; }
    virtual const AbstractMultiValueTypeInfo* MultiValueType()  const override { return NULL; }
    virtual const AbstractContainerTypeInfo*  ContainerType()   const override { return NULL; }
    
};


template<class TDataType, class TBaseClass>
class AbstractMultiValueTypeInfoImpl : public TBaseClass
{
public:
    typedef TDataType DataType;
    typedef DataTypeInfo<DataType> Info;
    typedef typename Info::FinalValueType FinalValueType;

    // MultiValue API

    virtual AbstractValueTypeInfo* getFinalValueType() const override { return VirtualTypeInfo<FinalValueType>::get(); }
    virtual bool Integer()  const override { return Info::Integer;  }
    virtual bool Scalar()   const override { return Info::Scalar;   }
    virtual bool String()   const override { return Info::String;   }
    virtual bool Unsigned() const override { return Info::Unsigned; }

    virtual bool FixedFinalSize()              const override { return Info::FixedFinalSize;                    }
    virtual size_t FinalSize()                 const override { return Info::FinalSize;                         }
    virtual size_t finalSize(const void* data) const override { return Info::finalSize(*(const DataType*)data); }

    virtual void setFinalSize(void* data, size_t size) const override
    {
        Info::setFinalSize(*(DataType*)data, size);
    }

    virtual std::string getFinalValueString (const void* data, size_t index) const override
    {
        std::string v;
        Info::getFinalValueString(*(const DataType*)data, index, v);
        return v;
    }
    virtual long long   getFinalValueInteger(const void* data, size_t index) const override
    {
        long long v = 0;
        Info::getFinalValue(*(const DataType*)data, index, v);
        return v;
    }
    virtual double      getFinalValueScalar (const void* data, size_t index) const override
    {
        double v = 0;
        Info::getFinalValue(*(const DataType*)data, index, v);
        return v;
    }

    virtual void setFinalValueString (void* data, size_t index, const std::string& value) const override
    {
        Info::setFinalValueString(*(DataType*)data, index, value);
    }
    virtual void setFinalValueInteger(void* data, size_t index, long long value) const override
    {
        Info::setFinalValue(*(DataType*)data, index, value);
    }
    virtual void setFinalValueScalar (void* data, size_t index, double value) const override
    {
        Info::setFinalValue(*(DataType*)data, index, value);
    }
    
};

template<class TDataType>
class VirtualTypeInfoImpl<TDataType, true, true, false> : public AbstractMultiValueTypeInfoImpl< TDataType, AbstractTypeInfoImpl<TDataType, AbstractValueTypeInfo> >
{
public:
    typedef TDataType DataType;
    typedef DataTypeInfo<DataType> Info;

    // Base API
    
    virtual const AbstractValueTypeInfo*      SingleValueType() const override { return this; }
    virtual const AbstractMultiValueTypeInfo* MultiValueType()  const override { return this; }
    virtual const AbstractContainerTypeInfo*  ContainerType()   const override { return NULL; }

    // Value API

    virtual bool Unsigned() const override { return Info::Unsigned; }

    virtual long long   getDataValueInteger(const void* data) const override
    {
        long long v = 0;
        Info::getDataValue(*(const DataType*)data, v);
        return v;
    }
    virtual double      getDataValueScalar (const void* data) const override
    {
        double v = 0;
        Info::getDataValue(*(const DataType*)data, v);
        return v;
    }

    virtual void setDataValueInteger(void* data, long long value) const override
    {
        Info::setDataValue(*(DataType*)data, value);
    }
    virtual void setDataValueScalar (void* data, double value) const override
    {
        Info::setDataValue(*(DataType*)data, value);
    }
};

template<class TDataType>
class VirtualTypeInfoImpl<TDataType, false, true, false> : public AbstractMultiValueTypeInfoImpl< TDataType, AbstractTypeInfoImpl<TDataType, AbstractMultiValueTypeInfo> >
{
public:
    typedef TDataType DataType;
    typedef DataTypeInfo<DataType> Info;

    // Base API

    virtual const AbstractValueTypeInfo*      SingleValueType() const override { return NULL; }
    virtual const AbstractMultiValueTypeInfo* MultiValueType()  const override { return this; }
    virtual const AbstractContainerTypeInfo*  ContainerType()   const override { return NULL; }
};


template<class T, class K>
AbstractTypeInfo* VirtualTypeInfo_GetValueType(const T& /*data*/, const K& /*key*/)
{
    // TODO: support for struct with different types per key
    return VirtualTypeInfo< typename DataTypeInfo<T>::MappedType >::get();
}

template<class TDataType, class TBaseClass>
class AbstractContainerTypeInfoImpl : public TBaseClass
{
public:
    typedef TDataType DataType;
    typedef DataTypeInfo<DataType> Info;
    typedef typename Info::KeyType KeyType;
    typedef typename Info::MappedType MappedType;

    // Container API


    virtual AbstractTypeInfo* getKeyType() const override { return VirtualTypeInfo<KeyType>::get(); }
    virtual AbstractTypeInfo* getMappedType() const override { return VirtualTypeInfo<MappedType>::get(); }
    virtual AbstractTypeInfo* getValueTypeForIndex(const void* data, size_t index) const override
    {
        return VirtualTypeInfo_GetValueType(*(const DataType*)data, index);
    }
    virtual AbstractTypeInfo* getValueTypeForKey(const void* data, const void* key) const
    {
        return VirtualTypeInfo_GetValueType(*(const DataType*)data, *(const KeyType*)key);
    }
    virtual bool FixedContainerSize() const override { return Info::FixedContainerSize; }

    virtual size_t containerSize(const void* data) const override
    {
        return Info::containerSize(*(const DataType*)data);
    }
    virtual void setContainerSize(void* data, size_t size) const override
    {
        return Info::setContainerSize(*(DataType*)data, size);
    }

    virtual const void* getItemKey(const void* data, size_t index, helper::SSOBuffer<>& keyBuffer) const override
    {
        return &Info::getItemKey(*(const DataType*)data, index, keyBuffer);
    }
    
    virtual const void* getItemValue(const void* data, size_t index) const override
    {
        return &Info::getItemValue(*(const DataType*)data, index);
    }
    
    virtual void* editItemValue(void* data, size_t index) const override
    {
        if (Info::StoreValues)
        {
            return Info::editItemValue(*(DataType*)data, index);
        }
        else
        {
            return NULL;
        }
    }

    // key-based access
    
    virtual const void* findItem(const void* data, const void* key) const override
    {
        return Info::findItem(*(const DataType*)data, *(const KeyType*)key);
    }
    
    virtual void* findEditItem(void* data, const void* key) const override
    {
        return Info::findEditItem(*(DataType*)data, *(const KeyType*)key);
    }
    
    virtual void* insertItem(void* data, const void* key) const override
    {
        return Info::insertItem(*(DataType*)data, *(const KeyType*)key);
    }
    
    virtual bool  eraseItem(void* data, const void* key) const override
    {
        return Info::eraseItem(*(DataType*)data, *(const KeyType*)key);
    }

    // temporary key creation to parse a key and then use it to access/insert an item
    virtual void* newKey   (const void* /*data*/, helper::SSOBuffer<>& keyBuffer) const override
    {
        return keyBuffer.getOrCreate<KeyType>();
    }

    // iterators

    typedef typename TBaseClass::const_iterator const_iterator;
    typedef typename TBaseClass::      iterator       iterator;

    typedef typename Info::const_iterator const_iterator_impl;
    typedef typename Info::iterator iterator_impl;

    virtual void newCBegin (const void* data, const_iterator& iter) const override
    {
        iter.setContext(this, data);
        const DataType& dataImpl = *(const DataType*)data;
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        const_iterator_impl& iterImpl = *buffer.getOrCreate<const_iterator_impl>();
        iterImpl = dataImpl.cbegin();
    }
    virtual void newCEnd   (const void* data, const_iterator& iter) const override
    {
        iter.setContext(this, data);
        const DataType& dataImpl = *(const DataType*)data;
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        const_iterator_impl& iterImpl = *buffer.getOrCreate<const_iterator_impl>();
        iterImpl = dataImpl.cend();
    }
    virtual void next      (const_iterator& iter) const override
    {
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        const_iterator_impl& iterImpl = *buffer.getOrCreate<const_iterator_impl>();
        ++iterImpl;
    }
    virtual bool isEqual   (const const_iterator& iter1, const const_iterator& iter2) const override
    {
        const helper::SSOBuffer<>& buffer1 = iter1.getIterBuffer();
        const helper::SSOBuffer<>& buffer2 = iter2.getIterBuffer();
        const const_iterator_impl& iter1Impl = *buffer1.get<const_iterator_impl>();
        const const_iterator_impl& iter2Impl = *buffer2.get<const_iterator_impl>();
        return iter1Impl == iter2Impl;
    }
    virtual void newBegin  (void* data, iterator& iter) const override
    {
        iter.setContext(this, data);
        DataType& dataImpl = *(DataType*)data;
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        iterator_impl& iterImpl = *buffer.getOrCreate<iterator_impl>();
        iterImpl = dataImpl.begin();
    }
    virtual void newEnd    (void* data, iterator& iter) const override
    {
        iter.setContext(this, data);
        DataType& dataImpl = *(DataType*)data;
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        iterator_impl& iterImpl = *buffer.getOrCreate<iterator_impl>();
        iterImpl = dataImpl.end();
    }
    virtual void next      (iterator& iter) const override
    {
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        const_iterator_impl& iterImpl = *buffer.getOrCreate<const_iterator_impl>();
        ++iterImpl;
    }
    virtual bool isEqual   (const iterator& iter1, const iterator& iter2) const override
    {
        const helper::SSOBuffer<>& buffer1 = iter1.getIterBuffer();
        const helper::SSOBuffer<>& buffer2 = iter2.getIterBuffer();
        const iterator_impl& iter1Impl = *buffer1.get<iterator_impl>();
        const iterator_impl& iter2Impl = *buffer2.get<iterator_impl>();
        return iter1Impl == iter2Impl;
    }
    
    virtual const void* getItemKey(const_iterator& iter) const override
    {
        const DataType& dataImpl = *(const DataType*)iter.getData();
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        const const_iterator_impl& iterImpl = *buffer.get<const_iterator_impl>();
        return &Info::key(dataImpl, iterImpl, iter.getKeyBuffer());
    }
    
    virtual const void* getItemValue(const_iterator& iter) const override
    {
        const DataType& dataImpl = *(const DataType*)iter.getData();
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        const const_iterator_impl& iterImpl = *buffer.get<const_iterator_impl>();
        return &Info::value(dataImpl, iterImpl);
    }
    
    virtual const void* getItemKey(iterator& iter) const override
    {
        const DataType& dataImpl = *(const DataType*)iter.getData();
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        const iterator_impl& iterImpl = *buffer.get<iterator_impl>();
        return &Info::key(dataImpl, iterImpl, iter.getKeyBuffer());
    }
    
    virtual void* getItemValue(iterator& iter) const override
    {
        return getItemValueIf(iter, std::integral_constant<bool,Info::StoreValues>());
    }

protected:
    template<class Iterator>
    void* getItemValueIf(Iterator& iter, std::true_type) const
    {
        DataType& dataImpl = *(DataType*)iter.getData();
        helper::SSOBuffer<>& buffer = iter.getIterBuffer();
        const iterator_impl& iterImpl = *buffer.get<iterator_impl>();
        return &Info::value(dataImpl, iterImpl);
    }
    template<class Iterator>
    void* getItemValueIf(Iterator& /*iter*/, std::false_type) const
    {
        return NULL;
    }
};

template<class TDataType>
class VirtualTypeInfoImpl<TDataType, false, true, true> : public AbstractContainerTypeInfoImpl< TDataType, AbstractMultiValueTypeInfoImpl< TDataType, AbstractTypeInfoImpl<TDataType, AbstractContainerMultiValueTypeInfo> > >
{
public:
    typedef TDataType DataType;
    typedef DataTypeInfo<DataType> Info;

    // Base API
    
    virtual const AbstractValueTypeInfo*      SingleValueType() const override { return NULL; }
    virtual const AbstractMultiValueTypeInfo* MultiValueType()  const override { return this; }
    virtual const AbstractContainerTypeInfo*  ContainerType()   const override { return this; }

};

template<class TDataType>
class VirtualTypeInfoImpl<TDataType, false, false, true> : public AbstractContainerTypeInfoImpl< TDataType, AbstractTypeInfoImpl<TDataType, AbstractContainerTypeInfo> >
{
public:
    typedef TDataType DataType;
    typedef DataTypeInfo<DataType> Info;

    // Base API
    
    virtual const AbstractValueTypeInfo*      SingleValueType() const override { return NULL; }
    virtual const AbstractMultiValueTypeInfo* MultiValueType()  const override { return NULL; }
    virtual const AbstractContainerTypeInfo*  ContainerType()   const override { return this; }

};

} // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_VIRTUALTYPEINFO_H
