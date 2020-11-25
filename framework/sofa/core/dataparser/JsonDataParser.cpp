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
#include "JsonDataParser.h"
#include "DataParserRegistry.h"
#include "DataParserError.h"
#include <iostream>
#include <sstream>
#include <sofa/defaulttype/AbstractTypeInfo.h>
#include <sofa/defaulttype/VirtualTypeInfo.h>

#include <rapidjson/writer.h>
#include <rapidjson/reader.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/error/en.h>

#include "JsonDataHandler.h"

namespace sofa
{

namespace core
{

namespace dataparser
{

using namespace rapidjson;

bool resultJson = DataParserRegistry::addParser(std::unique_ptr<DataParser>(new JsonDataParser(std::string("json_data_parser"))));

JsonDataParser::JsonDataParser(std::string name) : DataParser(std::move(name))
{}

std::error_code JsonDataParser::toData(std::istream& is, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    IStreamWrapper streamWrapper(is);
    DataHandler handler(data, typeInfo);
    Reader reader;
    if (!reader.Parse(streamWrapper, handler) && !handler.getErrorCode())
    {
        // ParseErrorCode e = reader.GetParseErrorCode();
        // std::cout << "RapidJson parsing error: " << GetParseError_En(e) << std::endl;
        return make_error_code(DataParserError::invalid_json);
    }
    return handler.getErrorCode();
}

std::error_code JsonDataParser::toData(const std::string& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    rapidjson::StringStream stream(input.c_str());
    return toData(stream, data, typeInfo);
}

std::error_code JsonDataParser::toData(rapidjson::StringStream& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    DataHandler handler(data, typeInfo);
    Reader reader;
    if (!reader.Parse(input, handler) && !handler.getErrorCode())
    {
        // ParseErrorCode e = reader.GetParseErrorCode();
        // std::cout << "RapidJson parsing error: " << GetParseError_En(e) << std::endl;
        return make_error_code(DataParserError::invalid_json);
    }
    return handler.getErrorCode();
}

template <class Stream>
std::error_code dataWrite(Writer<Stream>& writer, const void* data, const defaulttype::AbstractTypeInfo* typeInfo);

template <class Stream>
std::error_code fromData(Writer<Stream>& writer, const void* data, const defaulttype::AbstractValueTypeInfo* typeInfo)
{
    if (typeInfo->IsEnum())
    {
        std::string names;
        typeInfo->getDataValueString(data, names);
        writer.String(names);

    }
    else if (typeInfo->String())
    {
        writer.String(typeInfo->getFinalValueString(data, 0));
    }
    else if (typeInfo->Scalar())
    {
        writer.Double(typeInfo->getFinalValueScalar(data, 0));
    }
    else if (typeInfo->Integer())
    {
        writer.Int64(typeInfo->getFinalValueInteger(data, 0));
    }
    else
    {
        return make_error_code(DataParserError::incorrect_type_info);
    }
    return std::error_code();
}

template <class Stream>
void fromData(Writer<Stream>& writer, const void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo, size_t id)
{
    if (typeInfo->Scalar())
    {
        writer.Double(typeInfo->getFinalValueScalar(data, id));
    }
    else if (typeInfo->Integer())
    {
        writer.Int64(typeInfo->getFinalValueInteger(data, id));
    }
    else if (typeInfo->String())
    {
        writer.String(typeInfo->getFinalValueString(data, id));
    }
}

template <class Stream>
std::error_code fromData(Writer<Stream>& writer, const void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo)
{
    const size_t rowWidth = typeInfo->FinalSize();
    const size_t dataFinalSize = typeInfo->finalSize(data);
    const size_t nbRows = dataFinalSize / rowWidth;

    writer.StartArray();
    if (nbRows == 1 || rowWidth == 1)
    {
        for (size_t i = 0; i < dataFinalSize; i++)
        {
            fromData(writer, data, typeInfo, i);
        }
    }
    else
    {
        for (size_t i = 0; i < nbRows; i++)
        {
            writer.StartArray();
            for (size_t j = 0; j < rowWidth; j++)
            {
                fromData(writer, data, typeInfo, i * rowWidth + j);
            }
            writer.EndArray();
        }
    }
    writer.EndArray();
    return std::error_code();
}

template <class Stream>
std::error_code fromData(Writer<Stream>& writer, const void* data, const defaulttype::AbstractContainerTypeInfo* typeInfo)
{
    if (typeInfo->ContainerKind() == sofa::defaulttype::ContainerKindEnum::Map)
    {
        writer.StartObject();
        const sofa::defaulttype::AbstractTypeInfo *keyTypeInfo = typeInfo->getKeyType();
        const sofa::defaulttype::AbstractTypeInfo *mappedTypeInfo = typeInfo->getMappedType();
        for (sofa::defaulttype::AbstractContainerTypeInfo::const_iterator it = typeInfo->cbegin(data); it != typeInfo->cend(data); ++it)
        {
            const void* keyData = typeInfo->getItemKey(it);
            const void* mappedData = typeInfo->getItemValue(it);

            std::string allocatedString;
            keyTypeInfo->getDataValueString(keyData, allocatedString);
            writer.Key(allocatedString);
            dataWrite(writer, mappedData, mappedTypeInfo);
        }
        writer.EndObject();
    }
    else
    {
        writer.StartArray();
        const sofa::defaulttype::AbstractTypeInfo *mappedTypeInfo = typeInfo->getMappedType();
        if (typeInfo->containerSize(data) != 0)
        {
            for (sofa::defaulttype::AbstractContainerTypeInfo::const_iterator it = typeInfo->cbegin(data); it != typeInfo->cend(data); ++it)
            {
                const void* mappedData = typeInfo->getItemValue(it);
                dataWrite(writer, mappedData, mappedTypeInfo);
            }
        }
        writer.EndArray();
    }
    return std::error_code();
}

template <class Stream>
std::error_code fromData(Writer<Stream>& writer, const void* data, const defaulttype::AbstractStructureTypeInfo* typeInfo)
{
    writer.StartObject();
    for (size_t i = 0u; i < typeInfo->structSize(); i++)
    {
        const sofa::defaulttype::AbstractTypeInfo* memberTypeInfo = typeInfo->getMemberTypeForIndex(i);
        writer.Key(typeInfo->getMemberName(data, i));
        dataWrite(writer, typeInfo->getMemberValue(data, i), memberTypeInfo);
    }
    writer.EndObject();
    return std::error_code();
}

template <class Stream>
std::error_code dataWrite(Writer<Stream>& writer, const void* data, const defaulttype::AbstractTypeInfo* typeInfo)
{
    if (typeInfo->ValidInfo())
    {
        if (typeInfo->IsSingleValue())
        {
            return fromData(writer, data, typeInfo->SingleValueType());
        }
        else if (typeInfo->IsContainer())
        {
            return fromData(writer, data, typeInfo->ContainerType());
        }
        else if (typeInfo->IsMultiValue())
        {
            return fromData(writer, data, typeInfo->MultiValueType());
        }
        else if (typeInfo->IsStructure())
        {
            return fromData(writer, data, typeInfo->StructureType());
        }
    }
    return make_error_code(DataParserError::incorrect_type_info);
}

std::error_code JsonDataParser::fromData(StringBuffer& stringBuffer, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    Writer<StringBuffer> writer(stringBuffer); 
    return dataWrite(writer, data, typeInfo);
}

std::error_code JsonDataParser::fromData(std::ostream& os, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    OStreamWrapper ostreamWrapper(os);
    Writer<OStreamWrapper> writer(ostreamWrapper);
    return dataWrite(writer, data, typeInfo);
}

std::error_code JsonDataParser::fromData(std::string& output, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    StringBuffer s;
    auto error_code = fromData(s, data, typeInfo);
    output.append(s.GetString());
    return error_code;
}

} // namespace dataparser

} // namespace core

} // namespace sofa
