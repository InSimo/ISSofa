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
#include <sofa/defaulttype/AbstractTypeInfo.h>
#include <unordered_map>

namespace sofa
{
namespace defaulttype
{

namespace typeIdHelper
{
static std::unordered_map<std::size_t, const AbstractTypeInfo*> id2TypeInfo;
static inline const AbstractTypeInfo* get_type(std::size_t id)
{
    auto it = id2TypeInfo.find(id);
    return it != id2TypeInfo.cend() ? it->second : nullptr;
}
} // namespace typeIdHelper


AbstractTypeInfo::AbstractTypeInfo()
{
}

AbstractTypeInfo::~AbstractTypeInfo()
{
}

void AbstractTypeInfo::registerTypeInfoId(std::size_t id)
{
    typeIdHelper::id2TypeInfo.emplace(id, this);
}

const AbstractTypeInfo* AbstractTypeInfo::GetType(std::size_t id)
{
    return typeIdHelper::get_type(id);
}

} // namespace defaulttype

} // namespace sofa
