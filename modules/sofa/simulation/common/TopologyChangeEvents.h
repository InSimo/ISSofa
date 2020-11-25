/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 MGH, INRIA, USTL, UJF, CNRS, InSimo                *
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
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
// Description:
// Define events for safer topological changes propagation

#ifndef SOFA_SIMULATION_TOPOLOGYCHANGEEVENTS_H
#define SOFA_SIMULATION_TOPOLOGYCHANGEEVENTS_H

#include <sofa/core/objectmodel/Event.h>
#include <sofa/SofaSimulation.h>


namespace sofa
{
namespace simulation
{
class TaskStatus;
/**
  Event fired by Simulation::animate() after AnimateBeginEvent to perform before topological changes.
*/
class SOFA_SIMULATION_COMMON_API PreTopologyChangeEvent : public sofa::core::objectmodel::Event
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
class SOFA_SIMULATION_COMMON_API TopologyChangeEvent : public sofa::core::objectmodel::Event
{
public:
    SOFA_EVENT_CLASS_EXTERNAL((TopologyChangeEvent),((Event)));

    TopologyChangeEvent( double dt );

    ~TopologyChangeEvent();

    double getDt() const { return dt; }
	void setTaskStatus(TaskStatus* status) { taskStatus = status; }
    TaskStatus* getTaskStatus() const { return taskStatus; }

protected:
    double dt;
    TaskStatus* taskStatus;
};

/**
  Event fired by Simulation::animate() after TopologyChangeEvent to perform after topological changes.
*/
class SOFA_SIMULATION_COMMON_API PostTopologyChangeEvent : public sofa::core::objectmodel::Event
{
public:
    SOFA_EVENT_CLASS_EXTERNAL((PostTopologyChangeEvent),((Event)));

    PostTopologyChangeEvent( double dt );

    ~PostTopologyChangeEvent();

    double getDt() const { return dt; }
	void setTaskStatus(TaskStatus* status) { taskStatus = status; }
    TaskStatus* getTaskStatus() const { return taskStatus; }

protected:
    double dt;
    TaskStatus* taskStatus;
};

} // namespace simulation
} // namespace sofa

#endif // SOFA_SIMULATION_TOPOLOGYCHANGEEVENTS_H
