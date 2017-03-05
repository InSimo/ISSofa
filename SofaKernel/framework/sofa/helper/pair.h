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

#include <sofa/config.h>

#include <utility>
#include <iostream>
#include <limits>

namespace sofa
{

namespace helper
{

template<class T1, class T2>
class pair : public std::pair<T1, T2>
{
public:
    
    // inherit constructors from std::pair
    using std::pair<T1,T2>::pair;

    //**************************
    //   Serialization format :
    //   first,second
    //**************************

    std::ostream& write(std::ostream& os) const
    {
        os << this->first << "," << this->second;
        return os;
    }

    std::istream& read(std::istream& in)
    {
        T1 f;
        T2 s;

        if (! (in >> f))
        {
            return in;
        }

        in.ignore(std::numeric_limits<std::streamsize>::max(), ',');

        if (! (in >> s))
        {
            return in;
        }

        this->first = f;
        this->second = s;

        if (in.eof())
        {
            in.clear();
        }

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
