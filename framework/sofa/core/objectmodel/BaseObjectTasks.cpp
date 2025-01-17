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
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/BaseContext.h>

namespace sofa
{

namespace core
{

namespace objectmodel
{

BaseObjectTasks::~BaseObjectTasks()
{}

Iterative::IterativePartition* BaseObject::prepareTask()
{
    Iterative::IterativePartition *p=NULL;
    sofa::core::objectmodel::Context *context=sofa::core::objectmodel::Context ::DynamicCast(this->getContext());
    if(this->getPartition())
    {
        p=this->getPartition();
    }
    else if(context&&context->is_partition())
    {
        p=context->getPartition();
    }
    return p;
}

} // namespace objectmodel

} // namespace core

} // namespace sofa

