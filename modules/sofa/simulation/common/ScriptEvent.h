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
*                              SOFA :: Plugins                                *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_SIMULATION_SCRIPTEVENT_H
#define SOFA_SIMULATION_SCRIPTEVENT_H

#include <sofa/SofaFramework.h>
#include <sofa/core/objectmodel/Event.h>
#include <string>
#include <sofa/simulation/common/Node.h>


namespace sofa
{

namespace simulation
{

/**
 * @brief This event notifies about GUI interaction.
 */
class SOFA_SIMULATION_COMMON_API ScriptEvent : public sofa::core::objectmodel::Event
{
public:
    SOFA_EVENT_CLASS_EXTERNAL((ScriptEvent),((Event)));

    /**
     * @brief Constructor.
     */
    ScriptEvent(sofa::simulation::Node::SPtr sender, const char* eventName, std::string value = std::string());

    /**
     * @brief Destructor.
     */
    virtual ~ScriptEvent();

    /**
     * @brief Get the sender name
     */
    const sofa::simulation::Node::SPtr getSender(void) const {return m_sender;};

    /**
     * @brief Get the event name
     */
    const std::string getEventName(void) const {return m_eventName;};


    /**
     * @brief Get the event name
     */
    const std::string getEventString(void) const {return m_value.c_str();};

private:

    sofa::simulation::Node::SPtr m_sender;
    std::string m_eventName;
    std::string m_value;

};

} // namespace simulation

} // namespace sofa

#endif // SOFA_SIMULATION_SCRIPTEVENT_H
