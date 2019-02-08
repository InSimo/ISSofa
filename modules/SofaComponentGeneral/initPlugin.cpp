/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#include "initPlugin.h"
#include <sofa/core/ObjectFactory.h>


namespace sofa
{
namespace component
{
//Here are just several convenient functions to help user to know what contains the plugin

extern "C" {
    SOFA_COMPONENT_BASE_API void initExternalModule();
    SOFA_COMPONENT_BASE_API const char* getModuleName();
    SOFA_COMPONENT_BASE_API const char* getModuleVersion();
    SOFA_COMPONENT_BASE_API const char* getModuleLicense();
    SOFA_COMPONENT_BASE_API const char* getModuleDescription();
    SOFA_COMPONENT_BASE_API const char* getModuleComponentList();
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }

}

const char* getModuleName()
{
    return "SofaComponentGeneral";
}

const char* getModuleVersion()
{
    return "0.0";
}

const char* getModuleLicense()
{
    return "LGPL";
}


const char* getModuleDescription()
{
    return "Sofa Component General";
}

const char* getModuleComponentList()
{
    /// string containing the names of the classes provided by the plugin
    static std::string classes = sofa::core::ObjectFactory::getInstance()->listClassesFromTarget(sofa_tostring(SOFA_TARGET));
    return classes.c_str();
}


void initSofaComponentGeneral()
{
    initExternalModule();
}


} // namespace component

} // namespace base

