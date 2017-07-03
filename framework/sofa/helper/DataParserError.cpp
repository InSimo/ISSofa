/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
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
#include "DataParserError.h"

namespace sofa
{

namespace helper
{

std::string DataParserErrorCategory::message(int value) const
{
    switch (static_cast<DataParserError>(value))
    {
    case DataParserError::container_size_mismatch: return std::string("Error: stream value size mismatch with container size");
    case DataParserError::multivalue_size_mismatch: return std::string("Error: stream value size mismatch with multivalue size"); 
    case DataParserError::structure_size_mismatch: return std::string("Error: stream value size mismatch with struct size");
    case DataParserError::incorrect_type_info: return std::string("Error: Abstract type could not be used to parse data");
    default: return std::string("Incorrect error value");
    }
}

const std::error_category& getDataParserErrorCategory()
{
    static DataParserErrorCategory categorySingleton;
    return categorySingleton;
}

} // namespace helper

} // namespace sofa