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
#include "GUIFactory.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>
#include <memory>

namespace sofa
{

namespace simulation
{

namespace gui
{

static std::unique_ptr<BaseGUI> currentGUI = std::unique_ptr<BaseGUI>();

BaseGUI* getCurrentGUI()
{
    return currentGUI.get();
}

static sofa::core::ObjectFactory::ClassEntry::SPtr classVisualModel;
bool initGUI(const std::string& guiName)
{
    // If the GUI name is not in the factory, try to load the corresponding plugin
    if (!GUIFactory::getInstance()->hasKey(guiName))
    {
        std::string guiPlugin;
        // special cases (legacy, to be deprecated later - 2017)
        if (guiName == "qglviewer")
            guiPlugin = "SofaGuiQt";
        else
        { // usual case
            guiPlugin = guiName;
            // Make first letter uppercase
            guiPlugin[0] = toupper(guiPlugin[0]);
            // Remove any -* suffix
            std::string::size_type pos = guiPlugin.find('-');
            if (pos != std::string::npos)
            {
                guiPlugin = guiPlugin.substr(0,pos);
            }
            // Add SofaGui prefix
            if (guiPlugin.substr(0,7) != "SofaGui")
            {
                guiPlugin = std::string("SofaGui") + guiPlugin;
            }
        }
        //std::cout << "(SofaGui): Loading GUI Plugin \""<<guiPlugin<<"\"" << std::endl;
        if (!sofa::helper::system::PluginManager::getInstance().loadPlugin(guiPlugin))
        {
            std::cerr << "ERROR(SofaGui): The GUI Plugin \""<<guiPlugin<<"\" was not loaded." << std::endl;
            return false;
        }
    }

    // If the GUI name is still not in the factory, stop here
    if (!GUIFactory::getInstance()->hasKey(guiName))
    {
        return false;
    }

    // For now, all GUIs are based on OpenGL.
    // If that is not the case for a new GUI, it should define the "VisualModel"
    // alias within its plugin init function.
    if (!sofa::core::ObjectFactory::HasCreator("VisualModel"))
    {
        // Replace generic visual models with OglModel
        sofa::core::ObjectFactory::AddAlias("VisualModel", "OglModel", true,
                                            &classVisualModel);
    }
    return true;
}

BaseGUI* createGUI(const std::string& guiName, const std::string& programName,
                   const std::vector<std::string>& guiOptions)
{
    BaseGUIArgument a { guiName, programName, guiOptions };
    currentGUI.reset(GUIFactory::getInstance()->createObject(guiName, &a));
    return currentGUI.get();
}

void closeGUI()
{
    currentGUI.reset();
}

} // namespace gui

} // namespace simulation

namespace helper
{

template class SOFA_SIMULATION_COMMON_API Factory<std::string, sofa::simulation::gui::BaseGUI, const sofa::simulation::gui::BaseGUIArgument*>;

} // namespace helper

} // namespace sofa
