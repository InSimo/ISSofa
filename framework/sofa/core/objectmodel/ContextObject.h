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
#ifndef SOFA_CORE_OBJECTMODEL_CONTEXTOBJECT_H
#define SOFA_CORE_OBJECTMODEL_CONTEXTOBJECT_H

#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace core
{

namespace objectmodel
{

/**
 *  \brief Base class for simulation objects that modify the shared context (such as gravity, local coordinate system, ...).
 *
 */
class ContextObject : public virtual BaseObject
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((ContextObject), ((BaseObject)));
protected:
    ContextObject()
        : BaseObject()
    {}

    virtual ~ContextObject()
    {}
public:
    /// modify the Context
    virtual void apply()=0;

};


} // namespace objectmodel

} // namespace core

} // namespace sofa

#endif

