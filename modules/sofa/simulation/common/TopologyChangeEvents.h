/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

// Description:
// Define events for safer topological changes propagation

#ifndef ISPHYSICS_BASE_TOPOLOGYCHANGEEVENTS_H
#define ISPHYSICS_BASE_TOPOLOGYCHANGEEVENTS_H

#include <sofa/core/objectmodel/Event.h>
#include "initPlugin.h"

ISPHYSICS_PUBLIC

namespace isphysics
{

/**
  Event fired by Simulation::animate() after AnimateBeginEvent to perform before topological changes.
*/
class SOFA_ISPHYSICS_BASE_API PreTopologyChangeEvent : public sofa::core::objectmodel::Event
{
public:
    SOFA_EVENT_CLASS_EXTERNAL((PreTopologyChangeEvent),((Event)));

    PreTopologyChangeEvent( double dt );

    ~PreTopologyChangeEvent();

    double getDt() const { return dt; }

protected:
    double dt;
};

/**
  Event fired by Simulation::animate() after PreTopologyChangeEvent to perform topological changes.
*/
class SOFA_ISPHYSICS_BASE_API TopologyChangeEvent : public sofa::core::objectmodel::Event
{
public:
    SOFA_EVENT_CLASS_EXTERNAL((TopologyChangeEvent),((Event)));

    TopologyChangeEvent( double dt );

    ~TopologyChangeEvent();

    double getDt() const { return dt; }
	void setTaskStatus(void* status) { taskStatus = status; }
	void* getTaskStatus() const { return taskStatus; }

protected:
    double dt;
	void* taskStatus;
};

/**
  Event fired by Simulation::animate() after TopologyChangeEvent to perform after topological changes.
*/
class SOFA_ISPHYSICS_BASE_API PostTopologyChangeEvent : public sofa::core::objectmodel::Event
{
public:
    SOFA_EVENT_CLASS_EXTERNAL((PostTopologyChangeEvent),((Event)));

    PostTopologyChangeEvent( double dt );

    ~PostTopologyChangeEvent();

    double getDt() const { return dt; }
	void setTaskStatus(void* status) { taskStatus = status; }
	void* getTaskStatus() const { return taskStatus; }

protected:
    double dt;
	void* taskStatus;
};

} // namespace isphysics

#endif // ISPHYSICS_BASE_TOPOLOGYCHANGEEVENTS_H
