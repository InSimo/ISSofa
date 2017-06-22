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
// ? IsPOD
// ? FixedSize
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

    static constexpr bool IsContainer        = false; ///< true if this type is a container
    static constexpr bool IsSingleValue      = false; ///< true if this type is a single value
    static constexpr bool IsMultiValue       = false; /// TODO: check if all member types are identical
    static constexpr bool IsStructure        = true;  ///< true if this type is a structure

    static constexpr bool ValidInfo          = true;  ///< TODO: check if all member types are valid
    static constexpr bool Integer            = false; ///< true if this type uses integer values
    static constexpr bool Scalar             = false; ///< true if this type uses scalar values
    static constexpr bool String             = false; ///< true if this type uses text values
    static constexpr bool Unsigned           = false; ///< true if this type is unsigned

    /// true if the constructor is equivalent to setting memory to 0
    static constexpr bool ZeroConstructor    = false; // TODO (for data links optimization)
    /// true if copying the data can be done with a memcpy
    static constexpr bool SimpleCopy         = false; // TODO (for data links optimization)
    /// true if the layout in memory is simply N values of the same base type
    static constexpr bool SimpleLayout       = false; // TODO (for data links optimization)
    /// true if this type uses copy-on-write
    static constexpr bool CopyOnWrite        = (sizeof(DataType) > 64); // TODO
    /// true if the item keys are stored within the data structure (in which case getContainerKey() returns a const reference instead of a temporary value)
    static constexpr bool StoreKeys          = false;
    /// true if the item values are stored within the data structure
    static constexpr bool StoreValues        = true;

    static constexpr size_t StructSize = std::tuple_size<MembersTuple>::value; ///< size of the structure 

    static constexpr std::string name()
    {
        return DataTypeName<TDataType>::name();
    }

    static constexpr size_t structSize()
    {
        return StructSize;
    }
    
    // TODO: replace with generic lambda in C++14
    struct ResetValue
    {
        template <typename T>
        void operator()(T&& t) const { DataTypeInfo<T>::resetValue(t); }
    };
    static void resetValue(DataType& data, size_t /*reserve*/ = 0)
    {
        for_each(data, ResetValue{});
    }
    
    template<size_t Index>
    static auto getMemberValue(const DataType& data) -> const MemberDataType<Index>&
    {
        return MemberType<Index>::readRef(data);
    }

    template<size_t Index>
    static const KeyType& getMemberName(const DataType& data)
    {
        return MemberType<Index>::name();
    }

    template<size_t Index>
    static auto editMemberValue(DataType& data) -> MemberDataType<Index>&
    {
        return MemberType<Index>::writeRef(data);
    }
    

    template<typename DataTypeRef>
    static void getDataValueString(const DataTypeRef& data, std::string& value)
    {
        // TODO
    }

    template<typename DataTypeRef>
    static void setDataValueString(DataTypeRef&& data, const std::string& value)
    {
        // TODO
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

    
    // TODO: replace with generic lambdas in C++14
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
    static const std::string getMemberName(const DataType& data, size_t index)
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
            f(getMemberValue<I>(data));
            TupleForEach<Tuple,I+1>::visit(data, std::forward<F>(f), std::forward<LastF>(lf));
        }
        template <typename F, typename LastF>
        static void visit(DataType& data, F&& f, LastF&& lf)
        {
            f(editMemberValue<I>(data));
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
            lf(getMemberValue<N>(data));
        }
        template <typename F, typename LastF>
        static void visit(DataType& data, F&&, LastF&& lf)
        {
            lf(editMemberValue<N>(data));
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

} // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_STRUCTTYPEINFO_H
