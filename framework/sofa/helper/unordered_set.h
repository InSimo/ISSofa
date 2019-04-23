/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_HELPER_UNORDERED_SET_H
#define SOFA_HELPER_UNORDERED_SET_H

#include <sofa/SofaFramework.h>
#include <sofa/defaulttype/DataTypeInfo.h>

#include <unordered_set>
#include <iostream>
#include <string>
#include <algorithm>

namespace sofa
{

namespace helper
{

template<
    class Key,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = std::allocator<Key>>
class unordered_set : public std::unordered_set<Key, Hash, KeyEqual, Allocator>
{
public:
    // inherit constructors from std::unordered_set
    using std::unordered_set<Key, Hash, KeyEqual, Allocator>::unordered_set;

    /// Output stream
    friend std::ostream& operator<< ( std::ostream& o, const unordered_set<Key, Hash, KeyEqual, Allocator>& s )
    {
        //std::ostream_joiner is not standard yet
        //std::copy(s.begin(), s.end(), std::experimental::make_ostream_joiner(o, " "));
        if (!s.empty())
        {
            auto it = s.begin();
            o << *it;
            ++it;
            std::copy(it, s.end(), std::ostream_iterator<Key>(o, " "));
        }
        return o;
    }

    /// Input stream
    friend std::istream& operator>> ( std::istream& i, unordered_set<Key, Hash, KeyEqual, Allocator>& s )
    {
        s.clear();
        std::copy(std::istream_iterator<Key>(i), std::istream_iterator<Key>(), std::inserter(s, s.begin()));
        return i;
    }
};

} // namespace helper

namespace defaulttype
{

template<class Key, class Hash, class KeyEqual, class Allocator>
struct DataTypeInfo< sofa::helper::unordered_set<Key, Hash, KeyEqual, Allocator> > :
    public ContainerTypeInfo<sofa::helper::unordered_set<Key, Hash, KeyEqual, Allocator>, ContainerKindEnum::Set, 0> {};

template<class Key, class Hash, class KeyEqual, class Allocator>
struct DataTypeName< sofa::helper::unordered_set<Key, Hash, KeyEqual, Allocator> >
{
    static std::string name() { std::ostringstream o; o << "unordered_set<" << DataTypeName<Key>::name() << ">"; return o.str(); }
};

} //defaulttype

} // namespace sofa

#endif // SOFA_HELPER_UNORDERED_SET_H
