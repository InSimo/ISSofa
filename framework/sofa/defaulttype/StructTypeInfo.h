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
#ifndef SOFA_DEFAULTTYPE_STRUCTTYPEINFO_H
#define SOFA_DEFAULTTYPE_STRUCTTYPEINFO_H

#include <sofa/defaulttype/DataTypeInfo.h>
#include <type_traits>
#include <string>
#include <tuple>
#include <sofa/helper/preprocessor.h>

namespace sofa
{

namespace defaulttype
{

template<class TDataType, class TMembersTuple = typename TDataType::MembersTuple>
struct StructTypeInfo
{
    typedef TDataType   DataType;
    typedef std::string KeyType;                ///< type uniquely identifying items (or void if not applicable)
    typedef void        MappedType;             ///< type contained in DataType (TODO: check if all member types are identical)
    
    typedef TMembersTuple MembersTuple;
    
    template<size_t Index>
    using MemberType = typename std::tuple_element<Index,MembersTuple>::type;
    template<size_t Index>
    using MemberDataType = typename MemberType<Index>::type;

    static constexpr ContainerKindEnum ContainerKind = ContainerKindEnum::Single;
    static constexpr ValueKindEnum     FinalValueKind = ValueKindEnum::Void; // TODO

    // Fold expression emulation in C++11
    // Example result for structs with 3 members : op(op(f(M1), f(M2)), f(M3))
    template <class Tuple, std::size_t N = std::tuple_size<Tuple>::value>
    struct ApplyOnMembers
    {
        template<typename F, typename Op>
        static constexpr bool apply(F&& f, Op&& op) { return op(f(MemberType<N-1>{}), ApplyOnMembers<Tuple, N-1>::apply(std::forward<F>(f), std::forward<Op>(op))); }
    };
    template <class Tuple>
    struct ApplyOnMembers<Tuple, 0>
    {
        template<typename F, typename Op>
        static constexpr bool apply(F&&, Op&&) { return true; }
    };
    // Functions that can be used as Op
    static constexpr bool And(bool b1, bool b2) { return b1 && b2; }
    static constexpr bool Or(bool b1, bool b2) { return b1 || b2; }
    // Functions to apply on members
    struct IsMemberFixedFinalSize { template <typename MemberType> constexpr bool operator()(MemberType&&) const { return DataTypeInfo<typename MemberType::type>::FixedFinalSize; } };
    struct IsMemberValidInfo  { template <typename MemberType> constexpr bool operator()(MemberType&&) const { return DataTypeInfo<typename MemberType::type>::ValidInfo;  }};
    struct IsMemberSimpleCopy { template <typename MemberType> constexpr bool operator()(MemberType&&) const { return DataTypeInfo<typename MemberType::type>::SimpleCopy; }};
    
    ///< true if this type is a container
    static constexpr bool IsContainer        = false;
    ///< true if this type is a single value
    static constexpr bool IsSingleValue      = false;
    ///< true if this type is equivalent to multiple values (either single value or a composition of arrays of the same type of values)
    static constexpr bool IsMultiValue       = false; // TODO: check if all member types are identical
    ///< true if this type is a structure
    static constexpr bool IsStructure        = true;
    ///< true if this type is a Enum
    static constexpr bool IsEnum             = false;

    ///< true if this type has valid infos
    static constexpr bool ValidInfo          = ApplyOnMembers<MembersTuple>::apply(IsMemberValidInfo{}, And);
    ///< true if this type uses integer values
    static constexpr bool Integer            = false;
    ///< true if this type uses scalar values
    static constexpr bool Scalar             = false;
    ///< true if this type uses text values
    static constexpr bool String             = false;
    ///< true if this type is unsigned
    static constexpr bool Unsigned           = false;

    /// true if the constructor is equivalent to setting memory to 0
    static constexpr bool ZeroConstructor    = false; // TODO (for data links optimization)
    /// true if copying the data can be done with a memcpy
    static constexpr bool SimpleCopy         = ApplyOnMembers<MembersTuple>::apply(IsMemberSimpleCopy{}, And);
    /// true if the layout in memory is simply N values of the same base type
    static constexpr bool SimpleLayout       = false; // TODO (for data links optimization)
    /// true if this type uses copy-on-write
    static constexpr bool CopyOnWrite        = (sizeof(DataType) > 64); // TODO
    /// true if the item keys are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)
    static constexpr bool StoreKeys          = false;
    /// true if the item values are stored within the data structure
    static constexpr bool StoreValues        = true;

    static constexpr bool FixedFinalSize = ApplyOnMembers<MembersTuple>::apply(IsMemberFixedFinalSize{}, And);  ///< true if this type has a fixed size for all level until the final values
    static constexpr size_t FinalSize = 1; ///< 1, or fixed final size if FixedFinalSize is 1

    ///< size of the structure 
    static constexpr size_t StructSize = std::tuple_size<MembersTuple>::value;
    ///< if known at compile time, the size in bytes of the DataType, else 0
    static constexpr size_t ByteSize = SimpleCopy && StructSize > 0 ? sizeof(DataType) : 0; // Force 0 for empty struct as the standard mandates that any object must be at least of size 1 so different objects have different addresses
    
    ///< name of the structure
    static constexpr std::string name()
    {
        return DataTypeName<TDataType>::name();
    }

    static constexpr size_t structSize()
    {
        return StructSize;
    }
    static constexpr size_t byteSize(const DataType& /*data*/)
    {
        return ByteSize;
    }
    
    static const void* getValuePtr(const DataType& data)
    {
        return SimpleCopy ? &data : nullptr;
    }
    
    static void resetValue(DataType& data, size_t /*reserve*/ = 0)
    {
        ResetValue<DataType>::resetValue(data);
    }

    template<size_t Index>
    static auto getMemberValue(const DataType& data) -> const MemberDataType<Index>&
    {
        return MemberType<Index>::readRef(data);
    }

    template<size_t Index>
    static KeyType getMemberName(const DataType& data)
    {
        return MemberType<Index>::name();
    }

    template<size_t Index>
    static auto editMemberValue(DataType& data) -> MemberDataType<Index>&
    {
        return MemberType<Index>::writeRef(data);
    }

    static bool areEqual(const DataType& lhs, const DataType& rhs)
    {
        bool equal = true;
        for_each(lhs, rhs, StructEqual(equal));
        return equal;
    }

    static void getDataValueStream(const DataType& data, std::ostream& os)
    {
        os << "{ ";
        for_each(data, StructToStream(os), StructToStreamLast(os));
        os << "}";
    }

    static void setDataValueStream(DataType& data, std::istream& is)
    {
        is.ignore(2, '{');
        for_each(data, StreamToStruct(is), StreamToStructLast(is));
        is.ignore(2, '}');
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

    ///< Call f<MemberType>() for each struct member
    template <typename F>
    static void for_each(F&& f)
    {
        TupleForEach<MembersTuple>::visit(std::forward<F>(f), std::forward<F>(f));
    }
    ///< Call f(const MemberDataType&) for each struct member
    template <typename F>
    static void for_each(const DataType& data, F&& f)
    {
        TupleForEach<MembersTuple>::visit(data, std::forward<F>(f), std::forward<F>(f));
    }   
    ///< Call f(MemberDataType&) for each struct member
    template <typename F>
    static void for_each(DataType& data, F&& f)
    {
        TupleForEach<MembersTuple>::visit(data, std::forward<F>(f), std::forward<F>(f));
    }
    ///< Call f<MemberType>() for each struct member except lf<MemberType>() for the last member (useful for serializers)
    template <typename F, typename LastF>
    static void for_each(F&& f, LastF&& lf)
    {
        TupleForEach<MembersTuple>::visit(std::forward<F>(f), std::forward<LastF>(lf));
    }
    ///< Call f(const MemberDataType&) for each struct member except lf(const MemberDataType&) for the last member (useful for serializers)
    template <typename F, typename LastF>
    static void for_each(const DataType& data, F&& f, LastF&& lf)
    {
        TupleForEach<MembersTuple>::visit(data, std::forward<F>(f), std::forward<LastF>(lf));
    }   
    ///< Call f(MemberDataType&) for each struct member except except lf(MemberDataType&) for the last member (useful for serializers)
    template <typename F, typename LastF>
    static void for_each(DataType& data, F&& f, LastF&& lf)
    {
        TupleForEach<MembersTuple>::visit(data, std::forward<F>(f), std::forward<LastF>(lf));
    }
    ///< Call f(MemberDataType&) for each member of 2 structs
    template <typename F>
    static void for_each(const DataType& data, const DataType& data2, F&& f)
    {
        TupleForEach<MembersTuple>::visit(data, data2, std::forward<F>(f), std::forward<F>(f));
    }
    

    struct GetMemberValue
    {
        template <typename MemberType>
        void operator()(MemberType&&, const DataType&& data) { m_value = &MemberType::readRef(data); }
        const void* m_value;   
    };
    struct GetMemberName
    {
        template <typename MemberType>
        void operator()(MemberType&&, const DataType&&) { m_name = MemberType::name(); }
        const char* m_name;   
    };
    struct EditMemberValue
    {
        template <typename MemberType>
        void operator()(MemberType&&, DataType&& data) { m_value = &MemberType::writeRef(data); }
        void* m_value;   
    };
    static const void* getMemberValue(const DataType& data, size_t index)
    {
        assert(index < StructSize);
        GetMemberValue gmv;
        TupleForElem<MembersTuple>::visit(std::forward<const DataType>(data), index, std::forward<GetMemberValue>(gmv));
        return gmv.m_value;
    }
    static KeyType getMemberName(const DataType& data, size_t index)
    {
        assert(index < StructSize);
        GetMemberName gmn;
        TupleForElem<MembersTuple>::visit(std::forward<const DataType>(data), index,  std::forward<GetMemberName>(gmn));
        return gmn.m_name;
    }
    static void* editMemberValue(DataType& data, size_t index)
    {
        assert(index < StructSize);
        EditMemberValue emv;
        TupleForElem<MembersTuple>::visit(std::forward<DataType>(data), index,  std::forward<EditMemberValue>(emv));
        return emv.m_value;
    }
    
    
protected:
    
    ////////////////////////
    /// Tuple visitation ///
    ////////////////////////
    
    // Visit all struct members using (tail) recursion and call a method on each of them
    template <class Tuple, std::size_t I = 0, std::size_t N = std::tuple_size<Tuple>::value-1>
    class TupleForEach
    {
    public:
        template <typename F, typename LastF>
        static void visit(F&& f, LastF&& lf)
        {
            //f.template operator()<MemberType<I>>(); // Does not work with VS2015, need to instantiate
            f(MemberType<I>{});
            TupleForEach<Tuple,I+1>::visit(std::forward<F>(f), std::forward<LastF>(lf));
        }
        template <typename F, typename LastF>
        static void visit(const DataType& data, F&& f, LastF&& lf)
        {
            f(MemberType<I>{}, getMemberValue<I>(data));
            TupleForEach<Tuple,I+1>::visit(data, std::forward<F>(f), std::forward<LastF>(lf));
        }
        template <typename F, typename LastF>
        static void visit(const DataType& data, const DataType& data2, F&& f, LastF&& lf)
        {
            f(MemberType<I>{}, getMemberValue<I>(data), getMemberValue<I>(data2));
            TupleForEach<Tuple,I+1>::visit(data, data2, std::forward<F>(f), std::forward<LastF>(lf));
        }
        template <typename F, typename LastF>
        static void visit(DataType& data, F&& f, LastF&& lf)
        {
            f(MemberType<I>{}, editMemberValue<I>(data));
            TupleForEach<Tuple,I+1>::visit(data, std::forward<F>(f), std::forward<LastF>(lf));
        }
    };
     
    // Last member : end of recursion
    template <class Tuple, std::size_t N>
    class TupleForEach<Tuple, N, N>
    {
    public:
        template <typename F, typename LastF>
        static void visit(F&&, LastF&& lf)
        {
            //lf.template operator()<MemberType<N>>(); // Does not work with VS2015, need to instantiate
            lf(MemberType<N>{});
        }
        template <typename F, typename LastF>
        static void visit(const DataType& data, F&&, LastF&& lf)
        {
            lf(MemberType<N>{}, getMemberValue<N>(data));
        }
        template <typename F, typename LastF>
        static void visit(const DataType& data, const DataType& data2, F&&, LastF&& lf)
        {
            lf(MemberType<N>{}, getMemberValue<N>(data), getMemberValue<N>(data2));
        }
        template <typename F, typename LastF>
        static void visit(DataType& data, F&&, LastF&& lf)
        {
            lf(MemberType<N>{}, editMemberValue<N>(data));
        }
    };

    // Required for empty structs
    template <class Tuple>
    class TupleForEach<Tuple, 0, -1>
    {
    public:
        template <typename F, typename LastF>
        static void visit(F&&, LastF&&) {}
        template <typename F, typename LastF>
        static void visit(const DataType&, F&&, LastF&&) {}
        template <typename F, typename LastF>
        static void visit(DataType&, F&&, LastF&&) {}
    };

    
    // Visit all struct members using (tail) recursion and call a method on a specific indexed one
    template <class Tuple, size_t I = std::tuple_size<Tuple>::value>
    struct TupleForElem
    {
        template <typename D, typename F>
        static void visit(D&& data, size_t index, F&& f)
        {
            if (index == I-1) f(MemberType<I-1>{}, std::forward<D>(data));
            else TupleForElem<Tuple, I-1>::visit( std::forward<D>(data), index, std::forward<F>(f));
        }
    };

    // End of recursion : should not happen
    template <class Tuple>
    struct TupleForElem<Tuple, 0>
    {
        template <typename D, typename F>
        static void visit(D&&, size_t, F&&) { assert(false); }
    };
    
    
    ////////////////
    /// Functors ///
    ////////////////

    class StructEqual
    {
    public:
        StructEqual(bool& areEqual) : m_areEqual(areEqual) {}
        template <typename MemberType>
        void operator()(MemberType&&, const typename MemberType::type& lhs, const typename MemberType::type& rhs)
        {
            using DataType = typename MemberType::type;
            m_areEqual &= (lhs == rhs);
        }
    private:
        bool& m_areEqual;
    };

    class StructToStream
    {
    public:
        StructToStream(std::ostream& os) : m_stream(os) {}
        template <typename MemberType>
        void operator()(MemberType&&, const typename MemberType::type& data)
        {
            using DataType = typename MemberType::type;
            m_stream << data << "; ";
        }
    private:
        std::ostream& m_stream;
    };
    class StructToStreamLast
    {
    public:
        StructToStreamLast(std::ostream& os) : m_stream(os) {}
        template <typename MemberType>
        void operator()(MemberType&&, const typename MemberType::type& data)
        {
            using DataType = typename MemberType::type;
            m_stream << data << " ";
        }
    private:
        std::ostream& m_stream;
    };

    class StreamToStruct
    {
    public:
        StreamToStruct(std::istream& is) : m_stream(is) {}
        template <typename MemberType>
        void operator()(MemberType&&, typename MemberType::type& data)
        {
            m_stream >> data; // Value
            m_stream.ignore(2, ';');
        }
    private:
        std::istream& m_stream;
    };
    class StreamToStructLast
    {
    public:
        StreamToStructLast(std::istream& is) : m_stream(is) {}
        template <typename MemberType>
        void operator()(MemberType&&, typename MemberType::type& data)
        {
            m_stream >> data; // Value
        }
    private:
        std::istream& m_stream;
    };

    // This resetValue will be used if T does not have a default constructor
    template <typename T, typename Enable = void>
    struct ResetValue
    {
        // TODO: replace with generic lambda in C++14
        struct ResetValueFunctor
        {
            template <typename MemberType>
            void operator()(MemberType&&, typename MemberType::type& t) const { DataTypeInfo<typename MemberType::type>::resetValue(t); }
        };

        static void resetValue(DataType& data)
        {
            for_each(data, ResetValueFunctor{});
        }
    };

    // This resetValue will be used if T has a default constructor
    template <typename T>
    struct ResetValue<T, typename std::enable_if<std::is_default_constructible<T>::value>::type>
    {
        static void resetValue(DataType& data)
        {
            data = DataType();
        }
    };
};

#define SOFA_STRUCT_MEMBER(MemberName)                                       \
    struct MemberInfo_##MemberName{                                          \
    typedef decltype(MemberName) type;                                       \
    static const char* name() { return SOFA_TO_STRING_1(MemberName); }       \
    static const type& readRef(const StructType& s) { return s.MemberName; } \
    static type& writeRef(StructType& s) { return s.MemberName; }            \
    };

#define SOFA_MEMBERINFO_TYPE_NAME(MemberName)  MemberInfo_##MemberName

#define SOFA_STRUCT_DECL(TStruct, ...)                                  \
 using StructType = TStruct;                                            \
 SOFA_FOR_EACH(SOFA_STRUCT_MEMBER, SOFA_EMPTY_DELIMITER, __VA_ARGS__)   \
 using MembersTuple = std::tuple<SOFA_FOR_EACH(SOFA_MEMBERINFO_TYPE_NAME, (,), __VA_ARGS__)>

#define SOFA_STRUCT_STREAM_METHODS(TStruct) \
  inline friend std::ostream& operator<<(std::ostream& os, const TStruct& s) { sofa::defaulttype::StructTypeInfo<TStruct>::getDataValueStream(s, os); return os; } \
  inline friend std::istream& operator >> (std::istream& in, TStruct& s) { sofa::defaulttype::StructTypeInfo<TStruct>::setDataValueStream(s, in); return in; } SOFA_REQUIRE_SEMICOLON

#define SOFA_STRUCT_COMPARE_METHOD(TStruct)                                                                                    \
inline bool operator==(const TStruct& rhs) const { return sofa::defaulttype::StructTypeInfo<TStruct>::areEqual(*this, rhs); } \
 SOFA_REQUIRE_SEMICOLON

// Variadic macro to handle templated arguments like T<int, int> (semicolon is a separator for macro)
#define SOFA_STRUCT_DEFINE_TYPEINFO(...)                                          \
namespace sofa { namespace defaulttype {                                     \
template<> struct DataTypeInfo<__VA_ARGS__> : public StructTypeInfo<__VA_ARGS__> {}; \
}} SOFA_REQUIRE_SEMICOLON


} // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_STRUCTTYPEINFO_H
