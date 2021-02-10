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
#include <sofa/helper/system/PluginManager.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/config.h>

#include <fstream>
namespace sofa
{
namespace helper
{
namespace system
{

namespace
{
#ifndef _DEBUG
const std::string pluginsIniFile = "config/default/sofaplugins_release.ini";
#else
const std::string pluginsIniFile = "config/default/sofaplugins_debug.ini";
#endif

template <class LibraryEntry>
bool getPluginEntry(LibraryEntry& entry, DynamicLibrary::Handle handle)
{
    typedef typename LibraryEntry::FuncPtr FuncPtr;
    entry.func = (FuncPtr)DynamicLibrary::getSymbolAddress(handle, entry.symbol);
    if( entry.func == 0 )
    {
        return false;
    }
    else
    {
        return true;
    }
}

} // namespace

const char* Plugin::GetModuleComponentList::symbol    = "getModuleComponentList";
const char* Plugin::InitExternalModule::symbol        = "initExternalModule";
const char* Plugin::GetModuleDescription::symbol      = "getModuleDescription";
const char* Plugin::GetModuleLicense::symbol          = "getModuleLicense";
const char* Plugin::GetModuleName::symbol             = "getModuleName";
const char* Plugin::GetModuleVersion::symbol          = "getModuleVersion";

PluginManager & PluginManager::getInstance()
{
    static PluginManager instance;

    return instance;
}

PluginManager::~PluginManager()
{
    // BUGFIX: writeToIniFile should not be called here as it will erase the file in case it was not loaded
    // Instead we write the file each time a change have been made in the GUI and should be saved
    //writeToIniFile();
}

void PluginManager::readFromIniFile()
{
    std::string path = pluginsIniFile;
    if (!DataRepository.findFile(path, "", nullptr))
    {
        path = DataRepository.getFirstPath() + "/" + pluginsIniFile;
        std::ofstream ofile(path.c_str());
        ofile << "";
        ofile.close();
    }
    else path = DataRepository.getFile( pluginsIniFile );

    std::ifstream instream(path.c_str());
    std::string pluginPath;

    while(std::getline(instream,pluginPath))
    {
        if(loadPlugin(pluginPath))
            m_pluginMap[pluginPath].initExternalModule();
    }
    instream.close();
}

void PluginManager::writeToIniFile()
{
    std::string path= pluginsIniFile;
    if (!DataRepository.findFile(path, "", nullptr))
    {
        path = DataRepository.getFirstPath() + "/" + pluginsIniFile;
        std::ofstream ofile(path.c_str(),std::ios::out);
        ofile << "";
        ofile.close();
    }
    else path = DataRepository.getFile( pluginsIniFile );
    std::ofstream outstream(path.c_str());
    PluginIterator iter;
    for( iter = m_pluginMap.begin(); iter!=m_pluginMap.end(); ++iter)
    {
        const std::string& pluginPath = (iter->first);
        outstream << pluginPath << "\n";
    }
    outstream.close();
}

/// Get the default suffix applied to plugin names to find the actual lib to load
/// (depends on platform, version, debug/release build)
std::string PluginManager::getDefaultSuffix()
{
#ifdef SOFA_LIBSUFFIX
    return sofa_tostring(SOFA_LIBSUFFIX);
#else
    return "";
#endif
}

/// Search for a plugin given a path or a name (if no path or extension specified, in which case
/// the provided suffix will be added)
/// On return the full path of the plugin is written into the path parameter
///
bool PluginManager::findPlugin(std::string& pluginPath, const std::string& suffix, std::ostream* errlog)
{
    if (sofa::helper::system::SetDirectory::GetParentDir(pluginPath.c_str()).empty() &&
        sofa::helper::system::SetDirectory::GetExtension(pluginPath.c_str()).empty())
    {
        // no path and extension -> automatically add suffix and OS-specific extension
        pluginPath += suffix;
#if defined (WIN32)
        pluginPath = pluginPath + std::string(".dll");
#elif defined (__APPLE__)
        pluginPath = std::string("lib") + pluginPath + std::string(".dylib");
#else
        pluginPath = std::string("lib") + pluginPath + std::string(".so");
#endif
        //std::cout << "System-specific plugin filename: " << pluginPath << std::endl;
    }

    if( !PluginRepository.findFile(pluginPath,"",errlog) )
    {
        if (errlog) (*errlog) << "Plugin " << pluginPath << " NOT FOUND in: " << PluginRepository << std::endl;
        return false;
    }

    return true;
}

/// Check if a plugin is loaded given a path or a name (if no path or extension specified, in which case
/// the default suffix will be added)
/// On return the full path of the plugin is written into the path parameter
bool PluginManager::hasPlugin(std::string& pluginPath, bool finalPath)
{
    if (!finalPath)
    {
        if (!findPlugin(pluginPath, getDefaultSuffix(), NULL))
        {
            return false;
        }
    }
    return (m_pluginMap.find(pluginPath) != m_pluginMap.end() );
}

/// Load a plugin given a path or a name (if finalPath is false and no path or extension is
/// specified, in which case the default suffix will be added)
/// On return the full path of the plugin is written into the path parameter
///
/// CHANGE: it is no longer an error to call loadPlugin() on an already-loaded plugin
/// this was creating annoying error messages and complex logic to avoid it
///
bool PluginManager::loadPlugin(std::string& pluginPath, std::ostream* errlog, bool finalPath)
{
    if (!finalPath)
    {
        if (!findPlugin(pluginPath, getDefaultSuffix(), errlog))
        {
            return false;
        }
    }

    if (hasPlugin(pluginPath, true))
    {
        //if(errlog) (*errlog) << "Plugin " << pluginPath << " already in PluginManager" << std::endl;
        return true;
    }

    DynamicLibrary::Handle d  = DynamicLibrary::load(pluginPath);
    Plugin p;
    if( ! d.isValid() )
    {
        if (errlog) (*errlog) << "Plugin " << pluginPath << " loading FAILED with error: " << DynamicLibrary::getLastError() << std::endl;
        return false;
    }
    else
    {
        if(! getPluginEntry(p.initExternalModule,d))
        {
            if (errlog) (*errlog) << "Plugin " << pluginPath << " method initExternalModule() NOT FOUND" << std::endl;
            return false;
        }
        getPluginEntry(p.getModuleName,d);
        getPluginEntry(p.getModuleDescription,d);
        getPluginEntry(p.getModuleLicense,d);
        getPluginEntry(p.getModuleComponentList,d);
        getPluginEntry(p.getModuleVersion,d);
    }

    p.dynamicLibrary = d;
    m_pluginMap[pluginPath] = p;

    return true;
}

/// Unload a plugin given a path or a name (if no path or extension specified, in which case
/// the default suffix will be added)
/// On return the full path of the plugin is written into the path parameter
bool PluginManager::unloadPlugin(std::string& pluginPath, std::ostream* errlog, bool finalPath)
{
    PluginMap::iterator iter;
    iter = m_pluginMap.find(pluginPath);
    if(iter == m_pluginMap.end() && !finalPath)
    {
        findPlugin(pluginPath, getDefaultSuffix(), errlog);
        iter = m_pluginMap.find(pluginPath);
    }
    if( iter == m_pluginMap.end() )
    {
        if (errlog) (*errlog) << "Plugin " << pluginPath << "not in PluginManager" << std::endl;
        return false;
    }
    else
    {
        m_pluginMap.erase(iter);
        return true;
    }
}

void PluginManager::initRecentlyOpened()
{
    readFromIniFile();
}

std::istream& PluginManager::readFromStream(std::istream & in)
{
    while(!in.eof())
    {
        std::string pluginPath;
        in >> pluginPath;
        loadPlugin(pluginPath);
    }
    return in;
}

std::ostream& PluginManager::writeToStream(std::ostream & os) const
{
    PluginMap::const_iterator iter;
    for(iter= m_pluginMap.begin(); iter!=m_pluginMap.end(); ++iter)
    {
        os << iter->first;
    }
    return os;
}

void PluginManager::init()
{
    std::vector<std::string> preloadPlugins = sofa::helper::system::FileRepository::GetEnvItems("SOFA_PRELOAD");
    for (std::string name : preloadPlugins)
    {
        if (!loadPlugin(name))
        {
            std::cerr << "SOFA_PRELOAD: FAILED to load " << name << std::endl;
        }
    }
    PluginMap::iterator iter;
    for( iter = m_pluginMap.begin(); iter!= m_pluginMap.end(); ++iter)
    {
        Plugin& plugin = iter->second;
        plugin.initExternalModule();
    }
}

void PluginManager::init(const std::string& pluginName)
{
	PluginMap::iterator iter = m_pluginMap.find(pluginName);
	if(m_pluginMap.end() != iter)
	{
        Plugin& plugin = iter->second;
        plugin.initExternalModule();
	}

}

}

}

}
