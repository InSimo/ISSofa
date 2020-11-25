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
#ifndef SOFA_CORE_JSONDATAPARSER_H
#define SOFA_CORE_JSONDATAPARSER_H

#include "DataParser.h"
#include <sofa/SofaFramework.h>

#define RAPIDJSON_HAS_STDSTRING 1
#include <rapidjson/rapidjson.h>
#include <rapidjson/stringbuffer.h>

namespace sofa
{

namespace core
{

namespace dataparser
{

class SOFA_CORE_API JsonDataParser : public DataParser
{
public:
    JsonDataParser(std::string name);

    // Uses RapidJSON wrappers for std::basic_istream. The performance will be lower than the other methods.
    std::error_code toData(std::istream& is, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const override;
    std::error_code toData(const std::string& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const override;
    std::error_code toData(rapidjson::StringStream& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const;


    // Uses RapidJSON wrappers for std::basic_ostream. The performance will be lower than the other methods.
    std::error_code fromData(std::ostream& os, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const override;
    // Will do a copy of memory allocated by StringBuffer to std::string. If it's possible, StringBuffer should be used directly
    std::error_code fromData(std::string& output, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const override;
    std::error_code fromData(rapidjson::StringBuffer& os, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const;
};

} // namespace dataparser

} // namespace core

} // namespace sofa

#endif //SOFA_CORE_JSONDATAPARSER_H
