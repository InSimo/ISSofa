/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include "LayerRange.h"

namespace sofa
{
namespace core
{
namespace collision
{
LayerRange::LayerRange()
: first(0), second(0)
{
}

LayerRange::LayerRange(LayerID l)
:first(l)
,second(l) 
{
}


LayerRange::LayerRange(LayerID f, LayerID s)
:first(f)
,second(s)
{
}

} // namespace collision
} // namespace core
} // namespace sofa
