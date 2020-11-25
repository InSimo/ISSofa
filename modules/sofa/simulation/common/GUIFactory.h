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
#ifndef SOFA_SIMULATION_COMMON_GUIFACTORY_H
#define SOFA_SIMULATION_COMMON_GUIFACTORY_H

#include <sofa/helper/system/config.h>
#include <sofa/helper/Factory.h>
#include <sofa/simulation/common/Node.h>

#include <map>
#include <vector>
#include <string>
#include <memory>

#include "BaseGUI.h"

namespace sofa
{

namespace simulation
{

namespace gui
{

SOFA_SIMULATION_COMMON_API BaseGUI* getCurrentGUI();

SOFA_SIMULATION_COMMON_API bool initGUI(const std::string& guiName);

SOFA_SIMULATION_COMMON_API BaseGUI* createGUI(const std::string& guiName, const std::string& programName,
                                              const std::vector<std::string>& guiOptions);

SOFA_SIMULATION_COMMON_API void closeGUI();

typedef sofa::helper::Factory< std::string, BaseGUI, const BaseGUIArgument* > GUIFactory;

} // namespace gui

} // namespace simulation

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_SIMULATION_COMMON)
namespace helper
{
extern template class SOFA_SIMULATION_COMMON_API Factory<std::string, sofa::simulation::gui::BaseGUI, const sofa::simulation::gui::BaseGUIArgument*>;
} // namespace helper
#endif

} // namespace sofa

#endif
