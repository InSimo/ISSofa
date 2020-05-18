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
#ifndef SOFA_CORE_OBJECTMODEL_EVENT_H
#define SOFA_CORE_OBJECTMODEL_EVENT_H

#include <sofa/SofaFramework.h>
#include <sofa/core/objectmodel/BaseClass.h>

namespace sofa
{

namespace core
{

namespace objectmodel
{

#define SOFA_EVENT_CLASS_EXTERNAL(T,Parents) SOFA_SIMPLE_CLASS_EXTERNAL((::sofa::core::objectmodel::Event),T,Parents)
#define SOFA_EVENT_CLASS_UNIQUE(T,Parents) SOFA_SIMPLE_CLASS_UNIQUE((::sofa::core::objectmodel::Event),T,Parents)
#define SOFA_EVENT_CLASS_IMPL(T) SOFA_CLASS_EXTERNAL_IMPL(T)

/**
 *  \brief Base class for all events received by the objects.
 *
 * When created, the status is initialized as not handled. It is then propagated along the objects until it is handled.
 *
 * @author Francois Faure
 */
class SOFA_CORE_API Event
{
public:
    SOFA_ROOT_CLASS_EXTERNAL((Event));

    Event();

    virtual ~Event();

    /// Tag the event as handled, i.e. the event needs not be propagated further
    void setHandled();

    /// Returns true of the event has been handled
    bool isHandled() const;

    virtual const char* getClassName() const { return getClass()->className.c_str(); }
    
protected:
    bool m_handled;
};

} // namespace objectmodel

} // namespace core

namespace defaulttype
{

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_BUILD_CORE) 
extern template class SOFA_CORE_API BaseRootClass< sofa::core::objectmodel::Event >;
#endif

}

} // namespace sofa

#endif
