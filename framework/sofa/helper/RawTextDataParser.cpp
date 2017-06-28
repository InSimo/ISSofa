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
#include "RawTextDataParser.h"

namespace sofa
{

namespace helper
{

bool result = DataParserRegistry::addParser(std::unique_ptr<DataParser>(new RawTextDataParser()));

DataParser::ParserId RawTextDataParser::getId()
{
    return 0; // TODO
}

bool RawTextDataParser::toData(std::istream& is, void* data, const defaulttype::AbstractTypeInfo* typeInfo)
{
    std::string input(std::istreambuf_iterator<char>(is), {});
    return toData(input, data, typeInfo);
}

bool RawTextDataParser::toData(const std::string& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo)
{
    typeInfo->setDataValueString(data, input);
    return true;
}

bool RawTextDataParser::fromData(std::ostream& os, const void* data, const defaulttype::AbstractTypeInfo* typeInfo)
{
    std::string output;
    bool success = fromData(output, data, typeInfo);
    os << output;
    return success;
}

bool RawTextDataParser::fromData(std::string& output, const void* data, const defaulttype::AbstractTypeInfo* typeInfo)
{
    typeInfo->getDataValueString(data, output);
    return true;
}

} // namespace helper

} // namespace sofa
