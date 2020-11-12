/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include "TopologyChangeEvents.h"

namespace sofa
{
namespace simulation
{

SOFA_EVENT_CLASS_IMPL((PreTopologyChangeEvent));
SOFA_EVENT_CLASS_IMPL((TopologyChangeEvent));
SOFA_EVENT_CLASS_IMPL((PostTopologyChangeEvent));

PreTopologyChangeEvent::PreTopologyChangeEvent(double dt_)
    : sofa::core::objectmodel::Event()
    , dt(dt_)
{
}

PreTopologyChangeEvent::~PreTopologyChangeEvent()
{
}



TopologyChangeEvent::TopologyChangeEvent(double dt_)
    : sofa::core::objectmodel::Event()
    , dt(dt_)
	, taskStatus(0)
{
}

TopologyChangeEvent::~TopologyChangeEvent()
{
}


PostTopologyChangeEvent::PostTopologyChangeEvent(double dt_)
    : sofa::core::objectmodel::Event()
    , dt(dt_)
	, taskStatus(0)
{
}

PostTopologyChangeEvent::~PostTopologyChangeEvent()
{
}

} // namespace simulation
} // namespace sofa

