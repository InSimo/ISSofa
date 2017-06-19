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
#include <vector>
#include <type_traits>
#include <map>
#include <sstream>
#include <typeinfo>

namespace sofa
{

namespace defaulttype
{

template<class TDataType, class TMembersTuple = TDataType::MyMembers>
// ? IsPOD
// ? FixedSize
struct StructTypeInfo
{
    typedef TDataType DataType;

//    typedef ?? StructMemberNameEnum;

    typedef std::string    KeyType;         ///< type uniquely identifying items (or void if not applicable)
// OR    typedef StructMemberNameEnum KeyType;

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

    static void resetValue(DataType& data, size_t reserve = 0)
    {
        /*
          foreach(Member m) {
            DataTypeInfo<m::type>::resetValue(m.ref(data));
          }
        */
    }

    template<size_t index>
    static auto getMemberValue(const DataType& data) -> decltype(const typename std::get<index>(MembersTuple())::type &)
    {
        return std::get<index>(MembersTuple())::readRef(data);
    }

    template<size_t index>
    static const KeyType& getMemberName(const DataType& data)
    {
        return std::get<index>(MembersTuple())::name();
    }

    template<size_t index>
    static auto editMemberValue(DataType& data) -> decltype(typename std::get<index>(MemberTypes())::type &)
    {
        return std::get<index>(MembersTuple())::writeRef(data);
    }
};

template <class TStruct, const char* Name, class TMember, TMember* TStruct::MemberPtr>
struct MyStructMember {
    typedef decltype(((TStruct*)nullptr)->MemberName) type;
    static const char* name() { return SOFA_TOSTRING(MemberName); }
    static const type& readRef(const TStruct& s) { return s.MemberName; }
    static type& writeRef(TStruct& s) { return s.MemberName; }
}


typedef std::tuple<
    struct M1 { ...},
    struct M2 { ...} > T;



struct MyMember1 { ...};
struct MyMember2 { ...};
typedef std::tuple<M1, M2 > MyMembers;

#define SOFA_STRUCT_MEMBER_I(TStruct, MemberName)
struct {
    typedef decltype(((TStruct*)nullptr)->MemberName) type;
    static const char* name() { return SOFA_TOSTRING(MemberName); }
    static const type& readRef(const TStruct& s) { return s.MemberName; }
    static type& writeRef(TStruct& s) { return s.MemberName; }
}

#define SOFA_STRUCT_MEMBERS(TStruct, TMembers) \
    std::tuple<
        SOFA_CALL_FOREACH(SOFA_STRUCT_MEMBER_I, TMembers)
    >

} // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_STRUCTTYPEINFO_H
