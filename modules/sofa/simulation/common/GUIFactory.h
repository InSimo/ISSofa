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
#ifndef SOFA_SIMULATION_COMMON_GUIFACTORY_H
#define SOFA_SIMULATION_COMMON_GUIFACTORY_H

#include <sofa/helper/system/config.h>
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

class SOFA_SIMULATION_COMMON_API GUIFactory
{
public:
    class Creator
    {
    public:
        std::string name;
        std::string description;
        std::string license;
        std::vector<std::string> aliases;

        virtual void init() = 0;
        virtual BaseGUI* create() = 0;
    };
    typedef std::map<std::string, std::shared_ptr<Creator> > CreatorMap;

    GUIFactory() = default;
    GUIFactory(const GUIFactory& rhs) = delete;
    GUIFactory& operator=(const GUIFactory& rhs) = delete;
    
    void addCreator(std::unique_ptr<Creator> creator);

    void createInstance(std::string creatorName);

    static GUIFactory* getInstance();

protected:
    CreatorMap m_creators;
};

template<class GUI>
class SimpleCreator : public GUIFactory::Creator
{
public:    
    void init() override
    {}

    BaseGUI* create() override
    {
        return new GUI();
    }
};

template<class GUI, class GUICreator = SimpleCreator<GUI> >
class RegisterGUI
{
public:

    /// Start the registration by giving the name and description of this gui.
    RegisterGUI(std::string name, std::string description)
    {
        creator = std::unique_ptr<GUICreator>(new GUICreator());
        creator->name = std::move(name);
        creator->description = std::move(description);
    }

    /// Specify a license (LGPL, GPL, ...)
    RegisterGUI& addLicense(std::string license)
    {
        creator->license = std::move(license);
        return *this;
    }

    /// Specify a license (LGPL, GPL, ...)
    RegisterGUI& addAlias(std::string alias)
    {
        creator->aliases.push_back(std::move(alias));
        return *this;
    }

    /// This is the final operation that will actually commit the additions to the factory.
    operator int()
    {
        if (!creator || creator->name.empty())
            return 0;
        GUIFactory::getInstance()->addCreator(std::move(creator));
        return 1;
    }

protected:
    std::unique_ptr<GUICreator> creator;
};

} // namespace gui

} // namespace simulation

} // namespace sofa

#endif
