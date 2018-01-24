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
#ifndef SOFA_DEFAULTTYPE_PAIRTYPEINFO_H
#define SOFA_DEFAULTTYPE_PAIRTYPEINFO_H

#include <sofa/defaulttype/StructTypeInfo.h>
#include <sofa/helper/pair.h>

namespace sofa
{

namespace defaulttype
{

template<typename TPair>
struct PairTypeInfo_First
{
    using type = typename TPair::first_type;
    static const char* name() { return "first"; }
    static const type& readRef(const TPair& s) { return s.first; }
    static type& writeRef(TPair& s) { return s.first; }
};

template<typename TPair>
struct PairTypeInfo_Second
{
    using type = typename TPair::second_type;
    static const char* name() { return "second"; }
    static const type& readRef(const TPair& s) { return s.second; }
    static type& writeRef(TPair& s) { return s.second; }
};

template<typename TPair>
using PairTypeInfo = StructTypeInfo<TPair, std::tuple<PairTypeInfo_First<TPair>, PairTypeInfo_Second<TPair> > >;

template <typename TFirst, typename TSecond> 
struct DataTypeInfo<std::pair<TFirst, TSecond>> : public PairTypeInfo<std::pair<TFirst, TSecond>> {};
template <typename TFirst, typename TSecond>
struct DataTypeName<std::pair<TFirst, TSecond>> { static const char* name() { return "pair"; } };

template <typename TFirst, typename TSecond>
struct DataTypeInfo<sofa::helper::pair<TFirst, TSecond>> : public PairTypeInfo<sofa::helper::pair<TFirst, TSecond>> {};
template <typename TFirst, typename TSecond>
struct DataTypeName<sofa::helper::pair<TFirst, TSecond>> { static const char* name() { return "pair"; } };

}

}    

#endif  // SOFA_DEFAULTTYPE_PAIRTYPEINFO_H
