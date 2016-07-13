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
#ifndef SOFA_HELPER_PAIR_H
#define SOFA_HELPER_PAIR_H

#include <sofa/SofaFramework.h>

#include <utility>
#include <iostream>
#include <limits>

namespace sofa
{

namespace helper
{

template<class T1, class T2>
class pair : std::pair<T1, T2>
{
public:

    pair() : std::pair<T1, T2>() {}

    pair(const T1& v1, const T2& v2) : std::pair<T1, T2>(v1,v2) {}

    pair(const pair& p) : std::pair(p) {}
    
    pair(const std::pair<T1, T2>& p) : std::pair<T1, T2>(p) {}
    
    pair<T1, T2>& operator=(const std::pair<T1, T2>& p)
    {
        this->first = p.first;
        this->second = p.second;
        return (*this);
    }

    std::ostream& write(std::ostream& os) const
    {
        os << this->first << "," << this->second;
        return os;
    }

    std::istream& read(std::istream& in)
    {
        in >> this->first;
        in.ignore(std::numeric_limits<std::streamsize>::max(), ',');
        in >> this->second;

        if (in.rdstate() & std::ios_base::eofbit) { in.clear(); }
        return in;
    }

    /// Output stream
    inline friend std::ostream& operator<<(std::ostream& os, const pair<T1, T2>& p)
    {
        return p.write(os);
    }

    /// Input stream
    inline friend std::istream& operator>>(std::istream& in, pair<T1, T2>& p)
    {
        return p.read(in);
    }
};

} // namespace helper

} // namespace sofa

#endif
