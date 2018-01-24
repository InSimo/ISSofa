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
#include <sofa/defaulttype/AbstractTypeInfo.h>

namespace sofa
{

namespace core
{

namespace dataparser
{

bool resultRawText = DataParserRegistry::addParser(std::unique_ptr<DataParser>(new RawTextDataParser(std::string("raw_text_data_parser"))));

RawTextDataParser::RawTextDataParser(std::string name) : DataParser(std::move(name))
{}

std::error_code RawTextDataParser::toData(std::istream& is, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    return make_error_code(DataParserError::unsupported_operation);
    // doing the following would consume the entire istream, so better return an error until there is a proper implementation of the parser
    //std::string input(std::istreambuf_iterator<char>(is), {});
    //return toData(input, data, typeInfo);
}

std::error_code RawTextDataParser::toData(const std::string& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    typeInfo->setDataValueString(data, input);
    return std::error_code();
}

std::error_code RawTextDataParser::fromData(std::ostream& os, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    std::string output;
    auto error_code = fromData(output, data, typeInfo);
    os << output;
    return error_code;
}

std::error_code RawTextDataParser::fromData(std::string& output, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    typeInfo->getDataValueString(data, output);
    return std::error_code();
}

} // namespace dataparser

} // namespace helper

} // namespace sofa
