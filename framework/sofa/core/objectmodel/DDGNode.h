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
#ifndef SOFA_CORE_OBJECTMODEL_DDGNODE_H
#define SOFA_CORE_OBJECTMODEL_DDGNODE_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/helper/fixed_array.h>
#include <sofa/core/ExecParams.h>
#include <sofa/SofaFramework.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/core/objectmodel/BaseClass.h>
#include <list>

namespace sofa
{

namespace core
{

namespace objectmodel
{

class Base;
class BaseData;
class DDGNode;
class BaseObjectDescription;

template<>
class LinkTraitsPtrCasts<DDGNode>
{
public:
    static sofa::core::objectmodel::Base* getBase(sofa::core::objectmodel::DDGNode* n);
    static sofa::core::objectmodel::BaseData* getData(sofa::core::objectmodel::DDGNode* n);
};

template<>
class LinkTraitsGetClass<DDGNode, true>
{
public:
    static const BaseClass* GetObjectClass()
    {
        return NULL;
    }
    static const defaulttype::AbstractTypeInfo* GetDataType()
    {
        return NULL;
    }
};

/**
 *  \brief Abstract base to manage data dependencies. BaseData and DataEngine inherites from this class
 *
 */
class SOFA_CORE_API DDGNode
{
public:

    typedef MultiLink<DDGNode, DDGNode, BaseLink::FLAG_DOUBLELINK|BaseLink::FLAG_DATALINK|BaseLink::FLAG_OWNERDATA> DDGLink;
    typedef DDGLink::Container DDGLinkContainer;
    typedef DDGLink::const_iterator DDGLinkIterator;

    /// Constructor
    DDGNode();

    /// Destructor. Automatically remove remaining links
    virtual ~DDGNode();

    /// @name Class reflection system
    /// @{

    template<class T>
    static void dynamicCast(T*& ptr, Base* /*b*/)
    {
        ptr = NULL; // DDGNode does not derive from Base
    }

    /// Helper method to get the type name of a type derived from this class
    ///
    /// This method should be used as follow :
    /// \code  T* ptr = NULL; std::string type = T::typeName(ptr); \endcode
    /// This way derived classes can redefine the typeName method
    template<class T>
    static std::string typeName(const T* ptr= NULL)
    {
        return BaseClass::defaultTypeName(ptr);
    }

    /// Helper method to get the class name of a type derived from this class
    ///
    /// This method should be used as follow :
    /// \code  T* ptr = NULL; std::string type = T::className(ptr); \endcode
    /// This way derived classes can redefine the className method
    template<class T>
    static std::string className(const T* ptr= NULL)
    {
        return BaseClass::defaultClassName(ptr);
    }

    /// Helper method to get the namespace name of a type derived from this class
    ///
    /// This method should be used as follow :
    /// \code  T* ptr = NULL; std::string type = T::namespaceName(ptr); \endcode
    /// This way derived classes can redefine the namespaceName method
    template<class T>
    static std::string namespaceName(const T* ptr= NULL)
    {
        return BaseClass::defaultNamespaceName(ptr);
    }

    /// Helper method to get the template name of a type derived from this class
    ///
    /// This method should be used as follow :
    /// \code  T* ptr = NULL; std::string type = T::templateName(ptr); \endcode
    /// This way derived classes can redefine the templateName method
    template<class T>
    static std::string templateName(const T* ptr= NULL)
    {
        return BaseClass::defaultTemplateName(ptr);
    }
    /// @}

    /// Add a new input to this node
    void addInput(DDGNode* n);

    /// Remove an input from this node
    void delInput(DDGNode* n);

    /// Add a new output to this node
    void addOutput(DDGNode* n);

    /// Remove an output from this node
    void delOutput(DDGNode* n);

    /// Get the list of inputs for this DDGNode
    const DDGLinkContainer& getInputs();

    /// Get the list of outputs for this DDGNode
    const DDGLinkContainer& getOutputs();


    /// Request this value to be updated (thread-safe)
    void requestUpdate(const core::ExecParams* params = 0);

    /// Request this value to be updated if it is dirty (thread-safe)
    void requestUpdateIfDirty(const core::ExecParams* params = 0);

private:
    /// Update this value. For thread-safety, should no longer be called directly. Use requestUpdate() instead.
    virtual void update() = 0;

    /// Set dirty flag to false ( internal method )
    void doCleanDirty(const core::ExecParams* params, bool warnBadUse);

public:
    /// Returns true if the DDGNode needs to be updated
    bool isDirty(const core::ExecParams* params = 0) const
    {
        return dirtyFlags[currentAspect(params)].dirtyValue != 0;
    }

    /// Indicate the value needs to be updated
    virtual void setDirtyValue(const core::ExecParams* params = 0);

    /// Indicate the outputs needs to be updated. This method must be called after changing the value of this node.
    virtual void setDirtyOutputs(const core::ExecParams* params = 0);

    /// Set dirty flag to false ( note that this is no longer required, done automatically at the end of requestUpdate() / requestUpdateIfDirty() )
    void cleanDirty(const core::ExecParams* params = 0);

    /// Force set dirty flag to false. Use with caution (beware of thread safety)
    void forceCleanDirty(const core::ExecParams* params = 0);

    /// Utility method to call update if necessary. This method should be called before reading of writing the value of this node.
    void updateIfDirty(const core::ExecParams* params = 0) const
    {
        if (isDirty(params))
        {
            const_cast <DDGNode*> (this)->requestUpdateIfDirty(params);
        }
    }

    /// Copy the value of an aspect into another one.
    virtual void copyAspect(int destAspect, int srcAspect);

    static int currentAspect()
    {
        return core::ExecParams::currentAspect();
    }
    static int currentAspect(const core::ExecParams* params)
    {
        return core::ExecParams::currentAspect(params);
    }

    virtual const std::string& getName() const = 0;

    virtual Base* getOwner() const = 0;

    virtual BaseData* getData() const = 0;

    virtual bool findDataLinkDest(DDGNode*& ptr, const std::string& path, const BaseLink* link);

    void addLink(BaseLink* l);

protected:

    BaseLink::InitLink<DDGNode>
    initLink(const char* name, const char* help)
    {
        return BaseLink::InitLink<DDGNode>(this, name, help);
    }

    //std::list<DDGNode*> inputs;
    //std::list<DDGNode*> outputs;
    DDGLink inputs;
    DDGLink outputs;

    virtual void doAddInput(DDGNode* n);

    virtual void doDelInput(DDGNode* n);

    virtual void doAddOutput(DDGNode* n);

    virtual void doDelOutput(DDGNode* n);

private:

    typedef sofa::helper::system::atomic<int> FlagType;

    struct DirtyFlags
    {
        DirtyFlags() : dirtyValue(0), dirtyOutputs(0) {}

        FlagType dirtyValue;
        FlagType dirtyOutputs;
    };
    helper::fixed_array<DirtyFlags, SOFA_DATA_MAX_ASPECTS> dirtyFlags;

    // Thread safety structure, private to implementation
    struct UpdateState;
    UpdateState* updateStates;
};

} // namespace objectmodel

} // namespace core

} // namespace sofa

#endif
