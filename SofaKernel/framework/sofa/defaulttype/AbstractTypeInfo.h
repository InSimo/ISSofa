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
#ifndef SOFA_DEFAULTTYPE_ABSTRACTTYPEINFO_H
#define SOFA_DEFAULTTYPE_ABSTRACTTYPEINFO_H

#include <sofa/config.h>
#include <sofa/defaulttype/DataTypeKind.h>
#include <sofa/helper/SSOBuffer.h>
#include <typeinfo>
#include <string>
#include <vector>

namespace sofa
{

namespace defaulttype
{

class AbstractValueTypeInfo;
class AbstractMultiValueTypeInfo;
class AbstractContainerTypeInfo;

class SOFA_DEFAULTTYPE_API AbstractTypeInfo
{
public:
    virtual bool ValidInfo() const = 0;

    virtual std::string name() const = 0;
    virtual ContainerKindEnum ContainerKind() const = 0;
    virtual ValueKindEnum FinalValueKind() const = 0;
    
    virtual bool IsSingleValue() const = 0;
    virtual bool IsContainer() const = 0;
    virtual bool IsMultiValue() const = 0;

    virtual const AbstractValueTypeInfo* SingleValueType() const = 0;
    virtual const AbstractMultiValueTypeInfo* MultiValueType() const = 0;
    virtual const AbstractContainerTypeInfo* ContainerType() const = 0;

    virtual bool ZeroConstructor() const = 0;
    virtual bool SimpleCopy() const = 0;
    virtual bool SimpleLayout() const = 0;
    virtual bool CopyOnWrite() const = 0;

    virtual void resetValue(void* data, size_t reserve = 0) const = 0;
    virtual void getDataValueString(const void* data, std::string& value) const = 0;
    virtual void setDataValueString(void* data, const std::string& value) const = 0;

    virtual bool getAvailableItems(std::vector<std::string>& result, const void* data) const = 0;

    virtual const std::type_info* type_info() const = 0;

    AbstractTypeInfo(const AbstractTypeInfo&) = delete;
    AbstractTypeInfo(AbstractTypeInfo&&) = delete;
    void operator=(const AbstractTypeInfo&) = delete;
    void operator=(AbstractTypeInfo&&) = delete;
protected:
    AbstractTypeInfo();
    virtual ~AbstractTypeInfo();
};

                         
/// for Kind() == DTK_VALUE or multi value containers
class SOFA_DEFAULTTYPE_API AbstractMultiValueTypeInfo : public virtual AbstractTypeInfo
{
public:
    virtual AbstractValueTypeInfo* getFinalValueType() const = 0; // NULL for DTK_STRUCT, unless all items are the same type
    virtual bool Integer()  const = 0;
    virtual bool Scalar()   const = 0;
    virtual bool String()   const = 0;
    virtual bool Unsigned() const = 0;

    virtual bool FixedFinalSize() const = 0;
    virtual size_t FinalSize() const = 0;
    virtual size_t finalSize(const void* data) const = 0;

    virtual void setFinalSize(void* data, size_t size) const = 0;
    virtual std::string getFinalValueString (const void* data, size_t index) const = 0;
    virtual long long   getFinalValueInteger(const void* data, size_t index) const = 0;
    virtual double      getFinalValueScalar (const void* data, size_t index) const = 0;

    virtual void setFinalValueString (void* data, size_t index, const std::string& value) const = 0;
    virtual void setFinalValueInteger(void* data, size_t index, long long value) const = 0;
    virtual void setFinalValueScalar (void* data, size_t index, double value) const = 0;

    // void has no value
    // enum can be accessed with either IntegerValue (0..n-1) or TextValue (one of getAvailableItems)
    // bool can be accessed with either IntegerValue (0/1) or TextValue (false/true)
    // pointer can be accessed with either IntegerValue (but not valid for storage) or TextValue

protected:
    AbstractMultiValueTypeInfo() = default;
    virtual ~AbstractMultiValueTypeInfo() = default;
};

/// for Kind() == DTK_VALUE
class SOFA_DEFAULTTYPE_API AbstractValueTypeInfo : public AbstractMultiValueTypeInfo
{
public:

    virtual long long   getDataValueInteger(const void* data) const = 0;
    virtual double      getDataValueScalar (const void* data) const = 0;

    virtual void setDataValueInteger(void* data, long long value) const = 0;
    virtual void setDataValueScalar (void* data, double value) const = 0;

    // void has no value
    // enum can be accessed with either IntegerValue (0..n-1) or TextValue (one of getAvailableItems)
    // bool can be accessed with either IntegerValue (0/1) or TextValue (false/true)
    // pointer can be accessed with either IntegerValue (but not valid for storage) or TextValue

protected:
    AbstractValueTypeInfo() = default;
    virtual ~AbstractValueTypeInfo() = default;
};


template<class TypeInfo, typename DataPtr>
class TypeInfoItemIterator
{
public:
    typedef TypeInfoItemIterator<TypeInfo,DataPtr> type;

    DataPtr value()
    {
        return m_typeinfo->getItemValue(*this);
    }

    const void* key()
    {
        return m_typeinfo->getItemKey(*this);
    }

    AbstractTypeInfo* valueType()
    {
        return m_typeinfo->getValueTypeForKey(m_data, key());
    }

    AbstractTypeInfo* keyType()
    {
        return m_typeinfo->getKeyType();
    }

    void operator++()
    {
        m_typeinfo->next(*this);
    }

    void operator++(int)
    {
        m_typeinfo->next(*this);
    }

    bool operator==(const type& i2) const
    {
        return m_data == i2.m_data && m_typeinfo->isEqual(*this, i2);
    }

    bool operator!=(const type& i2) const
    {
        return !(*this == i2);
    }

//    friend TypeInfo;
//protected:
    
    void setContext(const TypeInfo* typeinfo, DataPtr data)
    {
        m_typeinfo = typeinfo;
        m_data = data;
    }
    helper::SSOBuffer<>& getIterBuffer()
    {
        return m_iterBuffer;
    }
    helper::SSOBuffer<>& getKeyBuffer()
    {
        return m_keyBuffer;
    }
    const helper::SSOBuffer<>& getIterBuffer() const
    {
        return m_iterBuffer;
    }
    const helper::SSOBuffer<>& getKeyBuffer() const
    {
        return m_keyBuffer;
    }
    DataPtr getData() const
    {
        return m_data;
    }
    
private:
    const TypeInfo* m_typeinfo = NULL;
    DataPtr m_data = NULL;
    helper::SSOBuffer<> m_iterBuffer;
    helper::SSOBuffer<> m_keyBuffer;
};

/// for Kind() == DTK_ARRAY or DTK_SET or DTK_MAP or DTK_STRUCT
class SOFA_DEFAULTTYPE_API AbstractContainerTypeInfo : public virtual AbstractTypeInfo
{
public:
    virtual AbstractTypeInfo* getKeyType() const = 0; // DTK_ARRAY: must be integer type, DTK_STRUCT: must be enum type, DTK_MAP/DTK_SET: arbitrary
    virtual AbstractTypeInfo* getMappedType() const = 0; // DTK_ARRAY/DTK_MAP/DTK_SET: arbitrary, DTK_STRUCT: void unless all values are the same
    virtual AbstractTypeInfo* getValueTypeForIndex(const void* data, size_t index) const = 0;
    virtual AbstractTypeInfo* getValueTypeForKey(const void* data, const void* key) const = 0;
    virtual bool FixedContainerSize() const = 0; // true for DTK_STRUCT and for fixed-size DTK_ARRAY

    virtual size_t containerSize(const void* data) const = 0;
    virtual void setContainerSize(void* data, size_t size) const = 0; // only for non-fixed-size DTK_ARRAY

    // index-based access, inefficient for DTK_MAP/DTK_SET, use iterators if possible
    
    virtual const void* getItemKey(const void* data, size_t index, helper::SSOBuffer<>& keyBuffer) const = 0;
    virtual const void* getItemValue(const void* data, size_t index) const = 0;
    virtual void* editItemValue(void* data, size_t index) const = 0;

    // key-based access
    
    virtual const void* findItem(const void* data, const void* key) const = 0;
    virtual void* findEditItem(void* data, const void* key) const = 0;
    virtual void* insertItem(void* data, const void* key) const = 0;
    virtual bool  eraseItem(void* data, const void* key) const = 0;

    // temporary key creation to parse a key and then use it to access/insert an item
    virtual void* newKey   (const void* data, helper::SSOBuffer<>& keyBuffer) const = 0;

    // iterators

    typedef TypeInfoItemIterator<AbstractContainerTypeInfo, const void*> const_iterator;
    typedef TypeInfoItemIterator<AbstractContainerTypeInfo,       void*>       iterator;

    virtual void newCBegin (const void* data, const_iterator& iter) const = 0;
    virtual void newCEnd   (const void* data, const_iterator& iter) const = 0;
    virtual void next      (const_iterator& iter) const = 0;
    virtual bool isEqual   (const const_iterator& iter1, const const_iterator& iter2) const = 0;
    virtual void newBegin  (void* data, iterator& iter) const = 0;
    virtual void newEnd    (void* data, iterator& iter) const = 0;
    virtual void next      (iterator& iter) const = 0;
    virtual bool isEqual   (const iterator& iter1, const iterator& iter2) const = 0;
    virtual const void* getItemKey(const_iterator& iter) const = 0;
    virtual const void* getItemValue(const_iterator& iter) const = 0;
    virtual const void* getItemKey(iterator& iter) const = 0;
    virtual void* getItemValue(iterator& iter) const = 0;

    // iterators creation with c++11 move semantics
    
    const_iterator cbegin(const void* data) const
    {
        const_iterator result;
        newCBegin(data, result);
        result.setContext(this, data);
        return result;
    }
    const_iterator cend(const void* data) const
    {
        const_iterator result;
        newCEnd(data, result);
        result.setContext(this, data);
        return result;
    }
    iterator begin(void* data) const
    {
        iterator result;
        newBegin(data, result);
        result.setContext(this, data);
        return result;
    }
    iterator end(void* data) const
    {
        iterator result;
        newEnd(data, result);
        result.setContext(this, data);
        return result;
    }
};


class SOFA_DEFAULTTYPE_API AbstractContainerMultiValueTypeInfo : public AbstractContainerTypeInfo, public AbstractMultiValueTypeInfo
{
};

} // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_ABSTRACTTYPEINFO_H
