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
#ifndef SOFA_HELPER_INTEGER_ID_H
#define SOFA_HELPER_INTEGER_ID_H

#include <sofa/SofaFramework.h>
#include <sofa/helper/vector.h>
#include <sofa/helper/accessor.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <limits>

namespace sofa
{

namespace helper
{

typedef const char* (*integer_id_name)();

template < integer_id_name Name, typename Index = unsigned int, Index DefaultId = (Index)-1 >
class integer_id
{
public:
    typedef integer_id<Name, Index, DefaultId> Id;
    typedef Index index_type;
    typedef int sindex_type;
protected:
    index_type index;
public:
    static const char* getName() { return (*Name)(); }
    static index_type getDefaultId() { return DefaultId; }

    integer_id() : index(DefaultId) {}
    explicit integer_id(index_type i) : index(i) {}
    integer_id(const Id& i) : index(i.index) {}

    Id& operator=(const Id& i)
    {
        index = i.index;
        return *this;
    }

    index_type getId() const { return index; }
    void setId(index_type i) { index = i; }

    bool isValid() const
    {
        return index != DefaultId;
    }
    bool isValid(index_type size) const
    {
        return (unsigned)index < (unsigned)size;
    }

    bool operator==(const Id& a) const
    {
        return index == a.index;
    }

    bool operator==(const index_type& i) const
    {
        return index == i;
    }

    bool operator!=(const Id& a) const
    {
        return index != a.index;
    }

    bool operator!=(const index_type& i) const
    {
        return index != i;
    }

    bool operator<(const Id& a) const
    {
        return index < a.index;
    }

    bool operator<(const index_type& i) const
    {
        return index < i;
    }

    bool operator<=(const Id& a) const
    {
        return index <= a.index;
    }

    bool operator<=(const index_type& i) const
    {
        return index <= i;
    }

    bool operator>(const Id& a) const
    {
        return index > a.index;
    }

    bool operator>(const index_type& i) const
    {
        return index > i;
    }

    bool operator>=(const Id& a) const
    {
        return index >= a.index;
    }

    bool operator>=(const index_type& i) const
    {
        return index >= i;
    }

    template<typename int_type>
    Id operator+(int_type i) const
    {
        return Id(index + i);
    }

    template<typename int_type>
    Id& operator+=(int_type i)
    {
        index += i;
        return *this;
    }

    template<typename int_type>
    Id operator-(int_type i) const
    {
        return Id(index - i);
    }

    sindex_type operator-(const Id& i) const
    {
        return (sindex_type)(index - i.index);
    }
    
    template<typename int_type>
    Id& operator-=(int_type i)
    {
        index -= i;
        return *this;
    }
    
    Id& operator++()
    {
        ++index;
        return *this;
    }

    Id operator++(int)
    {
        Id old = *this;
        index++;
        return old;
    }
    
    Id& operator--()
    {
        --index;
        return *this;
    }
    
    Id operator--(int)
    {
        Id old = *this;
        index--;
        return old;
    }

    /// Output stream
    inline friend std::ostream& operator<< ( std::ostream& os, const Id& i )
    {
        return os << i.getId();
    }

    /// Input stream
    inline friend std::istream& operator>> ( std::istream& in, Id& i )
    {
        index_type v = i.getDefaultId();
        if (in >> v)
            i.setId(v);
        return in;
    }

};

void SOFA_HELPER_API vector_access_failure(const void* vec, unsigned size, unsigned i, const std::type_info& type, const char* tindex);

template <class T, class TIndex, bool CheckIndices =
#if !defined(NDEBUG) || defined(SOFA_CHECK_CONTAINER_ACCESS)
    true
#else
    false
#endif
    , class MemoryManager = CPUMemoryManager<T> >
class vector_id : public vector<T, MemoryManager>
{
public:
    typedef vector<T, MemoryManager> Inherit;
    typedef T value_type;
    typedef TIndex index_type;
    typedef index_type ID;
    typedef typename Inherit::size_type size_type;
    typedef typename Inherit::reference reference;
    typedef typename Inherit::const_reference const_reference;
    typedef typename Inherit::iterator iterator;
    typedef typename Inherit::const_iterator const_iterator;

    template<class T2> struct rebind
    {
        typedef typename MemoryManager::template rebind<T2>::other MM2;
        typedef vector_id< T2, TIndex, CheckIndices, MM2 > other;
    };

    /// Basic constructor
    vector_id() : Inherit() {}
    /// Constructor
    vector_id(size_type n, const T& value): Inherit(n,value) {}
    /// Constructor
    vector_id(int n, const T& value): Inherit(n,value) {}
    /// Constructor
    vector_id(long n, const T& value): Inherit(n,value) {}
    /// Constructor
    explicit vector_id(size_type n): Inherit(n) {}
    /// Constructor
    vector_id(const std::vector<T>& x): Inherit(x) {}

#ifdef __STL_MEMBER_TEMPLATES
    /// Constructor
    template <class InputIterator>
    vector_id(InputIterator first, InputIterator last): Inherit(first,last) {}
#else /* __STL_MEMBER_TEMPLATES */
    /// Constructor
    vector_id(const_iterator first, const_iterator last): Inherit(first,last) {}
#endif /* __STL_MEMBER_TEMPLATES */

    /// Read/write random access, with explicit index_type
    reference at(index_type n)
    {
        if (CheckIndices)
        {
            if (!n.isValid(this->size()))
                vector_access_failure(this, this->size(), n.getId(), typeid(T), n.getName());
        }
        return *(this->begin() + n.getId());
    }

    /// Read-only random access, with explicit index_type
    const_reference at(index_type n) const
    {
        if (CheckIndices)
        {
            if (!n.isValid(this->size()))
                vector_access_failure(this, this->size(), n.getId(), typeid(T), n.getName());
        }
        return *(this->begin() + n.getId());
    }

    /// Read/write random access, with explicit index_type
    reference operator()(index_type n)
    {
        return at(n);
    }

    /// Read-only random access, with explicit index_type
    const_reference operator()(index_type n) const
    {
        return at(n);
    }

    /// Read/write random access, with explicit index_type
    reference operator[](index_type n)
    {
        return at(n);
    }

    /// Read-only random access
    const_reference operator[](index_type n) const
    {
        return at(n);
    }

    index_type push_back(const_reference v)
    {
        index_type i(this->size());
        Inherit::push_back(v);
        return i;
    }
protected:
    
    /// Read/write random access with regular index type, protected to force use of explicit index_type
    reference operator[](size_type n)
    {
        return at(index_type(n));
    }
    
    /// Read-only random access with regular index type, protected to force use of explicit index_type
    const_reference operator[](size_type n) const
    {
        return at(index_type(n));
    }

};


/// ReadAccessor implementation class for vector_id types
template<class T>
class ReadAccessorVectorId
{
public:
    typedef T container_type;
    typedef typename container_type::index_type index_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::value_type value_type;
    typedef typename container_type::reference reference;
    typedef typename container_type::const_reference const_reference;
    typedef typename container_type::iterator iterator;
    typedef typename container_type::const_iterator const_iterator;

protected:
    const container_type& vref;

public:
    ReadAccessorVectorId(const container_type& container) : vref(container) {}
    ~ReadAccessorVectorId() {}

    const container_type& ref() const { return vref; }

    bool empty() const { return vref.empty(); }
    size_type size() const { return vref.size(); }
    const_reference operator[](index_type i) const { return vref[i]; }
    const_reference operator()(index_type i) const { return vref(i); }

    const_iterator begin() const { return vref.begin(); }
    const_iterator end() const { return vref.end(); }

    inline friend std::ostream& operator<< ( std::ostream& os, const ReadAccessorVectorId<T>& vec )
    {
        return os << vec.vref;
    }

};

/// WriteAccessor implementation class for vector_id types
template<class T>
class WriteAccessorVectorId
{
public:
    typedef T container_type;
    typedef typename container_type::index_type index_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::value_type value_type;
    typedef typename container_type::reference reference;
    typedef typename container_type::const_reference const_reference;
    typedef typename container_type::iterator iterator;
    typedef typename container_type::const_iterator const_iterator;

protected:
    container_type& vref;

public:
    WriteAccessorVectorId(container_type& container) : vref(container) {}
    ~WriteAccessorVectorId() {}

    const container_type& ref() const { return vref; }
    container_type& wref() { return vref; }

    bool empty() const { return vref.empty(); }
    size_type size() const { return vref.size(); }

    const_reference operator[](index_type i) const { return vref[i]; }
    const_reference operator()(index_type i) const { return vref(i); }
    reference operator[](index_type i) { return vref[i]; }
    reference operator()(index_type i) { return vref(i); }

    const_iterator begin() const { return vref.begin(); }
    iterator begin() { return vref.begin(); }
    const_iterator end() const { return vref.end(); }
    iterator end() { return vref.end(); }

    void clear() { vref.clear(); }
    void resize(size_type s, bool /*init*/ = true) { vref.resize(s); }
    void reserve(size_type s) { vref.reserve(s); }
    index_type push_back(const_reference v) { return vref.push_back(v); }

    inline friend std::ostream& operator<< ( std::ostream& os, const WriteAccessorVectorId<T>& vec )
    {
        return os << vec.vref;
    }

    inline friend std::istream& operator>> ( std::istream& in, WriteAccessorVectorId<T>& vec )
    {
        return in >> vec.vref;
    }

};

// Support for vector_id

template <class T, class TIndex, bool CheckIndices, class MemoryManager>
class ReadAccessor< vector_id<T, TIndex, CheckIndices, MemoryManager> > : public ReadAccessorVectorId< vector_id<T, TIndex, CheckIndices, MemoryManager> >
{
public:
    typedef ReadAccessorVectorId< vector_id<T, TIndex, CheckIndices, MemoryManager> > Inherit;
    typedef typename Inherit::container_type container_type;
    ReadAccessor(const container_type& c) : Inherit(c) {}
};

template <class T, class TIndex, bool CheckIndices, class MemoryManager>
class WriteAccessor< vector_id<T, TIndex, CheckIndices, MemoryManager> > : public WriteAccessorVectorId< vector_id<T, TIndex, CheckIndices, MemoryManager> >
{
public:
    typedef WriteAccessorVectorId< vector_id<T, TIndex, CheckIndices, MemoryManager> > Inherit;
    typedef typename Inherit::container_type container_type;
    WriteAccessor(container_type& c) : Inherit(c) {}
};

template<sofa::helper::integer_id_name Name, typename Index, Index DefaultId, class T>
extern inline void DataTypeInfo_GetValue(const sofa::helper::integer_id<Name, Index, DefaultId>& data, T& value)
{
    value = static_cast<T>(data.getId());
}

template<sofa::helper::integer_id_name Name, typename Index, Index DefaultId>
extern inline void DataTypeInfo_GetValue(const sofa::helper::integer_id<Name, Index, DefaultId>& data, std::string& value)
{
    std::ostringstream o; o << data; value = o.str();
}

} // namespace helper

namespace defaulttype
{

template<class TValue, class TIndex, bool CheckIndices, class MemoryManager, size_t TFixedSize>
struct DataTypeInfo_ContainerTypes<helper::vector_id<TValue,TIndex,CheckIndices,MemoryManager>, ContainerKindEnum::Array, TFixedSize>
{
    typedef helper::vector_id<TValue,TIndex,CheckIndices,MemoryManager> TDataType;
    typedef typename TDataType::index_type      KeyType;
    typedef typename TDataType::value_type     MappedType;
    typedef typename TDataType::const_iterator const_iterator;
    typedef typename TDataType::iterator       iterator;
    static auto value(const TDataType&, const const_iterator& it) -> decltype(*it) { return *it; }
    static auto value(      TDataType&, const       iterator& it) -> decltype(*it) { return *it; }
    static KeyType key(const TDataType& data, const const_iterator& it) { return KeyType(it - data.begin()); }
    static const KeyType& key(const TDataType& data, const const_iterator& it, TypeInfoKeyBuffer& keyBuffer)
    {
        KeyType& key = *keyBuffer.getOrCreate<KeyType>();
        key = KeyType(it - data.begin());
        return key;
    }
    static auto valueAtIndex(const TDataType& data, size_t index) -> decltype(data[KeyType(index)]) { return data[KeyType(index)]; }
    static auto valueAtIndex(      TDataType& data, size_t index) -> decltype(data[KeyType(index)]) { return data[KeyType(index)]; }
    static KeyType keyAtIndex(const TDataType& data, size_t index) { return KeyType(index); }
    static const KeyType& keyAtIndex(const TDataType& data, size_t index, TypeInfoKeyBuffer& keyBuffer)
    {
        KeyType& key = *keyBuffer.getOrCreate<KeyType>();
        key = KeyType(index);
        return key;
    }
    static const MappedType* find(const TDataType& data, const KeyType& k)
    {
        if (k < data.size())
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
        if (k < data.size())
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
        if (k >= data.size())
        {
            if (TFixedSize==0)
            {
                resizeIfNotFixed(data, k.getId()+1, std::integral_constant<bool, TFixedSize!=0>());
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

template <sofa::helper::integer_id_name Name, typename Index, Index DefaultId>
struct DataTypeInfo< sofa::helper::integer_id<Name, Index, DefaultId> > : public SingleValueTypeInfo<sofa::helper::integer_id<Name, Index, DefaultId>, ValueKindEnum::Integer > {};
template <sofa::helper::integer_id_name Name, typename Index, Index DefaultId>
struct DataTypeName< sofa::helper::integer_id<Name, Index, DefaultId> >
{ static std::string name() { std::ostringstream o; o << "integer_id<" << Name() << "," << DataTypeName<Index>::name() << "," << DefaultId << ">"; return o.str(); } };

template <class T, class TIndex, bool CheckIndices, class MemoryManager>
struct DataTypeInfo< sofa::helper::vector_id<T, TIndex, CheckIndices, MemoryManager> > : public ContainerTypeInfo< sofa::helper::vector_id<T, TIndex, CheckIndices, MemoryManager>, ContainerKindEnum::Array, 0 > {};
template <class T, class TIndex, bool CheckIndices, class MemoryManager>
struct DataTypeName< sofa::helper::vector_id<T, TIndex, CheckIndices, MemoryManager> >
{ static std::string name() { std::ostringstream o; o << "vector_id<" << DataTypeName<T>::name() << "," << DataTypeName<TIndex>::name() << ">"; return o.str(); } };

} // namespace defaulttype

} // namespace sofa


namespace std
{

template <sofa::helper::integer_id_name Name, typename Index, Index DefaultId>
struct hash<sofa::helper::integer_id<Name, Index, DefaultId> >
{
    std::size_t operator()(const sofa::helper::integer_id<Name, Index, DefaultId>& iid) const
    {
        return std::hash<Index>()(iid.getId());
    }
};

} // namespace std

#endif
