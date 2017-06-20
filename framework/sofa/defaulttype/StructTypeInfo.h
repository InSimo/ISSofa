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
    typedef TDataType DataType;
    typedef const char* KeyType;         ///< type uniquely identifying items (or void if not applicable)
    typedef TMembersTuple MembersTuple;

    static constexpr bool IsContainer        = false; ///< true if this type is a container
    static constexpr bool IsSingleValue      = false;  ///< true if this type is a single value
    static constexpr bool IsMultiValue       = false; /// TODO: check if all member types are identical

    static constexpr bool ValidInfo          = true; ///< TODO: check if all member types are valid
    static constexpr bool Integer            = false;   ///< true if this type uses integer values
    static constexpr bool Scalar             = false;    ///< true if this type uses scalar values
    static constexpr bool String             = false;    ///< true if this type uses text values
    static constexpr bool Unsigned           = false;  ///< true if this type is unsigned
    static constexpr bool FixedStructSize     = true;  ///< true if this type has a fixed size for this container level

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

    static constexpr size_t StructSize = std::tuple_size<MembersTuple>::value; ///< to be filled by the MACRO

    static std::string name()
    {
        return DataTypeName<TDataType>::name();
    }

    static size_t structSize(const DataType& /*data*/)
    {
        static_assert(FixedStructSize,"Structs must have a fixed number of members");
        return StructSize;
    }
    
    template<size_t Index>
    using MemberType = typename std::tuple_element<Index,MembersTuple>::type;
    template<size_t Index>
    using MemberDataType = typename MemberType<Index>::type;

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
    
    ///< Call f<MemberType>() for each struct member
    template <typename F>
    static void for_each(F&& f)
    {
        TupleForEach<MembersTuple>::loop(std::forward<F>(f), std::forward<F>(f));
    }
    ///< Call f(const MemberDataType&) for each struct member
    template <typename F>
    static void for_each(const DataType& data, F&& f)
    {
        TupleForEach<MembersTuple>::loop(data, std::forward<F>(f), std::forward<F>(f));
    }   
    ///< Call f(MemberDataType&) for each struct member
    template <typename F>
    static void for_each(DataType& data, F&& f)
    {
        TupleForEach<MembersTuple>::loop(data, std::forward<F>(f), std::forward<F>(f));
    }
    ///< Call f<MemberType>() for each struct member except lf<MemberType>() for the last member (useful for serializers)
    template <typename F, typename LastF>
    static void for_each(F&& f, LastF&& lf)
    {
        TupleForEach<MembersTuple>::loop(std::forward<F>(f), std::forward<LastF>(lf));
    }
    ///< Call f(const MemberDataType&) for each struct member except lf(const MemberDataType&) for the last member (useful for serializers)
    template <typename F, typename LastF>
    static void for_each(const DataType& data, F&& f, LastF&& lf)
    {
        TupleForEach<MembersTuple>::loop(data, std::forward<F>(f), std::forward<LastF>(lf));
    }   
    ///< Call f(MemberDataType&) for each struct member except except lf(MemberDataType&) for the last member (useful for serializers)
    template <typename F, typename LastF>
    static void for_each(DataType& data, F&& f, LastF&& lf)
    {
        TupleForEach<MembersTuple>::loop(data, std::forward<F>(f), std::forward<LastF>(lf));
    }

private:
    // visit all struct members using (tail) recursion
    template <class Tuple, std::size_t I = 0, std::size_t N = std::tuple_size<Tuple>::value-1>
    class TupleForEach
    {
    public:
        template <typename F, typename LastF>
        static void loop(F&& f, LastF&& lf)
        {
            //f.template operator()<MemberType<I>>(); // Does not work with VS2015, need to instanciate
            f(MemberType<I>{});
            TupleForEach<Tuple,I+1>::loop(std::forward<F>(f), std::forward<LastF>(lf));
        }
        template <typename F, typename LastF>
        static void loop(const DataType& data, F&& f, LastF&& lf)
        {
            f(getMemberValue<I>(data));
            TupleForEach<Tuple,I+1>::loop(data, std::forward<F>(f), std::forward<LastF>(lf));
        }
        template <typename F, typename LastF>
        static void loop(DataType& data, F&& f, LastF&& lf)
        {
            f(editMemberValue<I>(data));
            TupleForEach<Tuple,I+1>::loop(data, std::forward<F>(f), std::forward<LastF>(lf));
        }
    };
     
    // Last member : end of recursion
    template <class Tuple, std::size_t N>
    class TupleForEach<Tuple, N, N>
    {
    public:
        template <typename F, typename LastF>
        static void loop(F&& f, LastF&& lf)
        {
            SOFA_UNUSED(f);
            //lf.template operator()<MemberType<N>>(); // Does not work with VS2015, need to instanciate
            lf(MemberType<N>{});
        }
        template <typename F, typename LastF>
        static void loop(const DataType& data, F&& f, LastF&& lf)
        {
            SOFA_UNUSED(f);
            lf(getMemberValue<N>(data));
        }
        template <typename F, typename LastF>
        static void loop(DataType& data, F&& f, LastF&& lf)
        {
            SOFA_UNUSED(f);
            lf(editMemberValue<N>(data));
        }
    };

    // Required for empty structs
    template <class Tuple>
    class TupleForEach<Tuple, 0, -1>
    {
    public:
        template <typename F, typename LastF>
        static void loop(F&& f, LastF&& lf)
        {
            SOFA_UNUSED(f);
            SOFA_UNUSED(lf);
        }
        template <typename F, typename LastF>
        static void loop(const DataType& data, F&& f, LastF&& lf)
        {
            SOFA_UNUSED(data);
            SOFA_UNUSED(f);
            SOFA_UNUSED(lf);
        }
        template <typename F, typename LastF>
        static void loop(DataType& data, F&& f, LastF&& lf)
        {
            SOFA_UNUSED(data);
            SOFA_UNUSED(f);
            SOFA_UNUSED(lf);
        }
    };
    
// TODO: decide if we want to support accessing a member with a runtime index
/* 
private:
    template <size_t I>
    struct visit_impl
    {
        template <typename T, typename F>
        static void visit(T& tup, size_t idx, F fun)
        {
            if (idx == I - 1) fun(std::get<I - 1>(tup));
            else visit_impl<I - 1>::visit(tup, idx, fun);
        }
    };

    template <>
    struct visit_impl<0>
    {
        template <typename T, typename F>
        static void visit(T& tup, size_t idx, F fun) { assert(false); }
    };

public:
    template <typename F, typename... Ts>
    void visit_at(std::tuple<Ts...> const& tup, size_t idx, F fun)
    {
        visit_impl<sizeof...(Ts)>::visit(tup, idx, fun);
    }

    template <typename F, typename... Ts>
    void visit_at(std::tuple<Ts...>& tup, size_t idx, F fun)
    {
        visit_impl<sizeof...(Ts)>::visit(tup, idx, fun);
    }
};*/

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
