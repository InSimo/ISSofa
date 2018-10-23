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
#ifndef SOFA_SIMULATION_SCENELOADERFACTORY_H
#define SOFA_SIMULATION_SCENELOADERFACTORY_H

#include <sofa/SofaSimulation.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/helper/system/SetDirectory.h>


namespace sofa
{

namespace simulation
{

/**
 *  \brief Main class used to register scene file loaders
 *
 *  It uses the Factory design pattern, where each class is registered in a map,
 *  and dynamically retrieved given the type name.
 *
 */

/// Abstract interface of a scene loader
class SOFA_SIMULATION_COMMON_API SceneLoader
{
public:
    typedef std::vector<std::string> ExtensionList;

    using SceneArguments = std::vector<std::string>;

    /// Pre-loading check
    virtual bool canLoadFileName(const char *filename)
    {
        std::string ext = sofa::helper::system::SetDirectory::GetExtension(filename);
        return canLoadFileExtension(ext.c_str());
    }

    /// Pre-saving check
    virtual bool canWriteFileName(const char *filename)
    {
        std::string ext = sofa::helper::system::SetDirectory::GetExtension(filename);
        return canWriteFileExtension(ext.c_str());
    }

    virtual bool canLoadFileExtension(const char *extension) = 0;

    virtual bool canWriteFileExtension(const char * /*extension*/) { return false; }

    /// load the file with the given arguments
    virtual sofa::simulation::Node::SPtr load(const char *filename, const SceneArguments& sceneArguments) = 0;

    sofa::simulation::Node::SPtr load(const char *filename);

    /// write scene graph in the file
    virtual void write(sofa::simulation::Node* /*node*/, const char * /*filename*/) {}

    /// get the file type description 
    virtual std::string getFileTypeDesc() = 0;

    /// get the list of file extensions
    virtual void getExtensionList(ExtensionList* list) = 0;

    virtual ~SceneLoader() {};

    template <class TSceneLoader>
    static TSceneLoader* create(TSceneLoader*, const void* /*argument*/)
    {
        return new TSceneLoader();
    }
};


class SOFA_SIMULATION_COMMON_API SceneLoaderFactory : private sofa::helper::Factory<std::string, SceneLoader, void*>
{
public:
    typedef std::vector<SceneLoader*> SceneLoaderList;
    typedef sofa::helper::Factory<std::string, SceneLoader, void*> Inherit;
    typedef Inherit::Object    Object;
    typedef Inherit::ObjectPtr ObjectPtr;
    typedef Inherit::Argument  Argument;
    typedef Inherit::Key       Key;
    typedef Inherit::Creator   Creator;
    typedef Inherit::Registry  Registry;

    using  Inherit::registerCreator;

    static SceneLoaderFactory* getInstance();



    /// Returns a map containing for each SceneLoaderFactory key, 
    /// the list of file extensions that the SceneLoader would support.
    std::map<std::string, std::vector<std::string>> getSupportedExtensionsMap() const;

    /// Create a new SceneLoader instance based on a file extension.
    /// returns nullptr if no matching entry exists in factory.
    SceneLoader* createFromFileExtension(std::string extension);

    /// Create a new SceneLoader instance based on a filename.
    /// returns nullptr if no matching entry exists in factory.
    SceneLoader* createFromFileName(std::string filename);

    /// For backward compatibility with external codes.
    SceneLoader* getEntryFileName(std::string filename);
};

} // namespace simulation

} // namespace sofa


#endif // SOFA_SIMULATION_SCENELOADERFACTORY_H

