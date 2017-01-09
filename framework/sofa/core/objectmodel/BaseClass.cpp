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
#include <sofa/core/objectmodel/BaseClass.h>

namespace sofa
{

namespace core
{

namespace objectmodel
{

BaseClass::BaseClass()
{
}

BaseClass::~BaseClass()
{
}

void BaseClass::logNewClass()
{
    /*std::cout << "NEW class " << className;
    if (!templateName.empty()) std::cout << '<' << templateName << '>';
    if (!namespaceName.empty()) std::cout << " in " << namespaceName;
    if (!shortName.empty()) std::cout << " short " << shortName;
    for (std::size_t i = 0; i < parents.size(); ++i)
    {
        std::cout << ((i == 0) ? ": " : ", ");
        std::cout << parents[i]->className;
        if (!parents[i]->templateName.empty()) std::cout << '<' << parents[i]->templateName << '>';
    }
    std::cout << std::endl;*/
}

} // namespace objectmodel

} // namespace core

} // namespace sofa

