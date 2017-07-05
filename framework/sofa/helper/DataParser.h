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
#ifndef SOFA_HELPER_DATAPARSER_H
#define SOFA_HELPER_DATAPARSER_H

#include <iostream>
#include <string>

#include "DataParserError.h"

namespace sofa
{

namespace defaulttype
{
class AbstractTypeInfo;
} // namespace defaulttype

namespace helper
{

class DataParser
{
public:
    using ParserId = size_t;

    DataParser(std::string name);

    ParserId getId();

    virtual std::error_code toData(std::istream& is, void* data, const defaulttype::AbstractTypeInfo* typeInfo) = 0;
    virtual std::error_code toData(const std::string& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo) = 0;

    virtual std::error_code fromData(std::ostream& os, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) = 0;
    virtual std::error_code fromData(std::string& output, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) = 0;
private:
    ParserId m_id;
};

inline DataParser::ParserId generateDataParserId(std::string name)
{
    return std::hash<std::string>{}(std::move(name));
}

inline DataParser::ParserId DataParser::getId()
{
    return m_id;
}

inline DataParser::DataParser(std::string name) : m_id(generateDataParserId(std::move(name)))
{
}

} // namespace helper

} // namespace sofa

#endif //SOFA_HELPER_DATAPARSER_H
