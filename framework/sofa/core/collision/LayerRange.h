/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
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

#ifndef SOFA_CORE_COLLISION_LAYERRANGE_H
#define SOFA_CORE_COLLISION_LAYERRANGE_H

#include <sofa/defaulttype/StructTypeInfo.h>
#include <iostream>

namespace sofa
{
namespace core
{
namespace collision
{

struct SOFA_CORE_API LayerRange
{
    typedef unsigned LayerID;

    LayerID first;
    LayerID second;
    LayerRange();
    explicit LayerRange(LayerID l);
    LayerRange(LayerID f, LayerID s);

	inline bool isLayerIncludingLayerID(LayerID lid) const
	{
		if(lid >= first && lid <= second)
			return true;

		return false;
	}

	inline bool isOverlapping(const LayerRange& l) const
	{
		if(l.second >= first && l.first <= second)
			return true;

		return false;
	}

    double middle() const
    {
        return (first+second)*0.5;
    }

    unsigned imiddle() const
    {
        return (first+second)/2;
    }

	inline bool operator==(const LayerRange& l) const
	{
		if(l.first == first && l.second == second)
			return true;

		return false;
	}

    inline bool operator!=(const LayerRange& l) const
    {
        return !(*this == l);
    }

    inline bool operator<(const LayerRange& l) const
    {
        return this->first < l.first || (this->first == l.first && this->second < l.second);
    }

    inline friend std::ostream& operator<< ( std::ostream& os, const LayerRange& p )
    {
        return os << p.first << " " << p.second;
    }

    inline friend std::istream& operator>> ( std::istream& in, LayerRange& p )
    {
        return in >> p.first >> p.second;
    }

    SOFA_STRUCT_DECL(LayerRange, first, second);
//    SOFA_STRUCT_STREAM_METHODS(LayerRange);
//    SOFA_STRUCT_COMPARE_METHOD(LayerRange);
};

} // namespace collision
} // namespace core
} // namespace sofa


SOFA_STRUCT_DEFINE_TYPEINFO(sofa::core::collision::LayerRange);

#endif // SOFA_CORE_COLLISION_LAYERRANGE_H