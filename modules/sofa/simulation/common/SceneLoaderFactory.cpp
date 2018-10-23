
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
#include "SceneLoaderFactory.h"




namespace sofa
{
namespace simulation
{

sofa::simulation::Node::SPtr SceneLoader::load(const char *filename)
{
    SceneArguments args;
    return load(filename, args);
}


SceneLoaderFactory* SceneLoaderFactory::getInstance()
{
    static SceneLoaderFactory instance;
    return &instance;
}

/// Get an entry given a file extension
SceneLoader* SceneLoaderFactory::createFromFileExtension(std::string extension)
{
    return this->createObject(extension, nullptr);
}

/// Get an entry given a file extension
SceneLoader* SceneLoaderFactory::createFromFileName(std::string filename)
{
    std::string ext = sofa::helper::system::SetDirectory::GetExtension(filename.c_str() );
    return createFromFileExtension(ext);
}

SceneLoader* SceneLoaderFactory::getEntryFileName(std::string filename)
{
    return createFromFileName(filename);
}

std::map<std::string, std::vector<std::string>> SceneLoaderFactory::getSupportedExtensionsMap() const
{
    std::map<std::string, std::vector<std::string>> keyAliasesMap;
    std::vector<std::string> keysAndAliases;
    this->uniqueKeys(std::back_inserter(keysAndAliases) );

    for (const std::string& k : keysAndAliases)
    {
        std::vector<std::string> extensions;

        auto entry = this->registry.find(k);
        if (entry != this->registry.end())
        {
            const std::vector<std::string>& entryAliases = entry->second->aliases();

            bool isCurrentKeyAlias = false;
            for (const std::string& entryAlias : entryAliases)
            {
                if (entryAlias == k)
                {
                    isCurrentKeyAlias = true;
                    break;
                }
            }

            if (!isCurrentKeyAlias)
            {
                for (const std::string& alias : entryAliases)
                {
                    extensions.push_back(alias);
                }

                keyAliasesMap.insert(std::pair<std::string, std::vector<std::string>>(k, extensions));
            }
        }
    }

    return keyAliasesMap;
}


} // namespace simulation

} // namespace sofa


