/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

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