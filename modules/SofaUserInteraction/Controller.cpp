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
// C++ Implementation : Controller
//
// Description:
//
//
// Author: Pierre-Jean Bensoussan, Digital Trainers (2008)
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include <SofaUserInteraction/Controller.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/JoystickEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/core/objectmodel/HapticDeviceEvent.h>
#include <sofa/core/objectmodel/GUIEvent.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/IntegrateEndEvent.h>


namespace sofa
{

namespace component
{

namespace controller
{


Controller::Controller()
    : handleEventTriggersUpdate( initData(&handleEventTriggersUpdate, false, "handleEventTriggersUpdate", "Event handling frequency controls the controller update frequency" ) )
{

}

Controller::~Controller()
{

}

void Controller::handleEvent(core::objectmodel::Event *event)
{
    if (sofa::simulation::AnimateBeginEvent* ev = sofa::simulation::AnimateBeginEvent::DynamicCast(event))
    {
        onBeginAnimationStep(ev->getDt());
    }
    else if (sofa::simulation::AnimateEndEvent* ev = sofa::simulation::AnimateEndEvent::DynamicCast(event))
    {
        onEndAnimationStep(ev->getDt());
    }
    else if (/*sofa::simulation::IntegrateEndEvent* ev =*/ sofa::simulation::IntegrateEndEvent::DynamicCast(event))
    {
        onIntegrateEndEvent();
    }
    else if (sofa::core::objectmodel::KeypressedEvent *kpev = sofa::core::objectmodel::KeypressedEvent::DynamicCast(event))
    {
        onKeyPressedEvent(kpev);
    }
    else if (sofa::core::objectmodel::KeyreleasedEvent *krev = sofa::core::objectmodel::KeyreleasedEvent::DynamicCast(event))
    {
        onKeyReleasedEvent(krev);
    }
    else if (sofa::core::objectmodel::MouseEvent *mev = sofa::core::objectmodel::MouseEvent::DynamicCast(event))
    {
        onMouseEvent(mev);
    }
    else if (sofa::core::objectmodel::JoystickEvent *jev = sofa::core::objectmodel::JoystickEvent::DynamicCast(event))
    {
        onJoystickEvent(jev);
    }
    else if (sofa::core::objectmodel::HapticDeviceEvent *oev = sofa::core::objectmodel::HapticDeviceEvent::DynamicCast(event))
    {
        onHapticDeviceEvent(oev);
    }
    else if (sofa::core::objectmodel::GUIEvent *gev = sofa::core::objectmodel::GUIEvent::DynamicCast(event))
    {
        onGUIEvent(gev);
    }
}

} // namespace controller

} // namepace component

} // namespace sofa

