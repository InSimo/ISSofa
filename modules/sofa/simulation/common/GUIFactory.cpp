/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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

void setCurrentGUI(BaseGUI* gui)
{
    currentGUI.reset(gui);
}

GUIFactory* GUIFactory::getInstance()
{
    static GUIFactory instance;
    return &instance;
}

void GUIFactory::addCreator(std::unique_ptr<Creator> creator)
{
    auto& creatorValue = m_creators[creator->name];
    creatorValue = std::move(creator);
    for (auto& alias : creatorValue->aliases)
    {
        m_creators.emplace(alias, creatorValue);
    }
}

void GUIFactory::createInstance(std::string creatorName)
{
    auto& creator = m_creators[creatorName];
    if (creator)
    {
        creator->init();
        setCurrentGUI(creator->create());
    }
}

} // namespace gui

} // namespace simulation

} // namespace sofa
