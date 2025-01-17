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
//
// C++ Interface: UpdateMappingEndEvent
//
// Description:
//
//
// Author: Jeremie Allard, MGH/CIMIT, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef SOFA_SIMULATION_UPDATEMAPPINGENDEVENT_H
#define SOFA_SIMULATION_UPDATEMAPPINGENDEVENT_H

#include <sofa/core/objectmodel/Event.h>
#include <sofa/SofaSimulation.h>

namespace sofa
{

namespace simulation
{

/**
  Event fired by Simulation::animate() after computing a new animation step.
  @author Jeremie Allard
*/
class SOFA_SIMULATION_COMMON_API UpdateMappingEndEvent : public sofa::core::objectmodel::Event
{
public:
    SOFA_EVENT_CLASS_EXTERNAL((UpdateMappingEndEvent),((Event)));

    UpdateMappingEndEvent( double dt );

    ~UpdateMappingEndEvent();

    double getDt() const { return dt; }
protected:
    double dt;
};

} // namespace simulation

} // namespace sofa

#endif
