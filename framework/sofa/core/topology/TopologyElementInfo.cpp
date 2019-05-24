/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#include "TopologyElementInfo.h"


namespace sofa
{
namespace core
{
namespace topology
{

std::string parseTopologyElementInfoToString( const TopologyElementInfo& info )
{
    std::ostringstream oss;
    oss << sofa::core::topology::parseTopologyObjectTypeToString( info.type ) << " id= " << info.element.id;

    return oss.str();
}

} // namespace topology
} // namespace core
} // namespace sofa