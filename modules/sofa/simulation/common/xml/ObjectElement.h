/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 MGH, INRIA, USTL, UJF, CNRS, InSimo                *
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
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_SIMULATION_COMMON_XML_OBJECTELEMENT_H
#define SOFA_SIMULATION_COMMON_XML_OBJECTELEMENT_H

#include <sofa/simulation/common/xml/Element.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace simulation
{

namespace xml
{

class SOFA_SIMULATION_COMMON_API ObjectElement : public Element<core::objectmodel::BaseObject>
{
public:
    ObjectElement(const std::string& name, const std::string& type, BaseElement* parent=NULL);

    virtual ~ObjectElement();

    virtual bool initNode();

    virtual bool init();

    virtual const char* getClass() const;
};

} // namespace xml

} // namespace simulation

} // namespace sofa

#endif
