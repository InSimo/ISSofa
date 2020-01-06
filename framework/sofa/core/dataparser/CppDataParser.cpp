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
#include "CppDataParser.h"
#include "DataParserRegistry.h"
#include "DataParserError.h"
#include <rapidjson/internal/dtoa.h>
#include <sofa/defaulttype/VirtualTypeInfo.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <cctype>

namespace sofa
{

namespace core
{

namespace dataparser
{

bool resultCpp = DataParserRegistry::addParser(std::unique_ptr<DataParser>(new CppDataParser(std::string("cpp_data_parser"), false)));
bool resultCpp20 = DataParserRegistry::addParser(std::unique_ptr<DataParser>(new CppDataParser(std::string("cpp20_data_parser"), true)));

CppDataParser::CppDataParser(std::string name, bool useNamed)
: DataParser(std::move(name))
, m_useNamed(useNamed)
{}

////////////////////////////////////////
//////////    READ TO DATA    //////////
////////////////////////////////////////

bool CppDataParser::fromDataInternal(std::error_code& result, std::ostream& out, const void* data, const defaulttype::AbstractValueTypeInfo* typeInfo) const
{
    if (typeInfo->IsEnum())
    {
        const sofa::defaulttype::AbstractEnumTypeInfo* enumTypeInfo = typeInfo->EnumType();
        out << enumTypeInfo->getDataEnumeratorString(data);
        return true;
    }
    else return fromDataInternal(result, out, data, typeInfo, 0); // AbstractValueTypeInfo derives from AbstractMultiValueTypeInfo
}

bool CppDataParser::fromDataInternal(std::error_code& result, std::ostream& out, const void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo, size_t id) const
{
    switch(typeInfo->FinalValueKind())
    {
    case defaulttype::ValueKindEnum::Void:
    case defaulttype::ValueKindEnum::Pointer:
    {
        result = make_error_code(DataParserError::incorrect_type_info);
        return false;
    }
    case defaulttype::ValueKindEnum::Integer:
    {
        long long value = typeInfo->getFinalValueInteger(data, id);
        if (typeInfo->Unsigned())
        {
            out << (unsigned long long)value;
        }
        else
        {
            out << value;
        }
        return true;
    }
    case defaulttype::ValueKindEnum::Scalar:
    {
        double value = typeInfo->getFinalValueScalar(data, id);
        //out << value;
        char buffer[400];
        *rapidjson::internal::dtoa(value, buffer) = 0;
        out << buffer;
        return true;
    }
    case defaulttype::ValueKindEnum::String:
    {
        std::string value = typeInfo->getFinalValueString(data, id);
        out << std::quoted(value);
        return true;
    }
    case defaulttype::ValueKindEnum::Bool:
    {
        bool value = (bool)typeInfo->getFinalValueInteger(data, id);
        out << (value ? "true":"false");
        return true;
    }
    case defaulttype::ValueKindEnum::Enum:
    {
        std::string value = typeInfo->getFinalValueString(data, id);
        out << value;
        return true;
    }
    }
    result = make_error_code(DataParserError::incorrect_type_info);
    return false;
}

bool CppDataParser::fromDataInternal(std::error_code& result, std::ostream& out, const void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo) const
{
    const size_t rowWidth = typeInfo->FinalSize();
    const size_t dataFinalSize = typeInfo->finalSize(data);
    const size_t nbRows = dataFinalSize / rowWidth;

    if (dataFinalSize == 0)
    {
        out << "{}";
        return true;
    }

    out << "{ ";
    if (nbRows == 1 || rowWidth == 1)
    {
        for (size_t i = 0; i < dataFinalSize; i++)
        {
            if (i) out << ", ";
            if (!fromDataInternal(result, out, data, typeInfo, i)) return false;
        }
    }
    else
    {
        for (size_t i = 0; i < nbRows; i++)
        {
            if (i) out << ", ";
            out << "{ ";
            for (size_t j = 0; j < rowWidth; j++)
            {
                if (j) out << ", ";
                if (!fromDataInternal(result, out, data, typeInfo, i * rowWidth + j)) return false;
            }
            out << " }";
        }
    }
    out << " }";
    return true;
}

bool CppDataParser::fromDataInternal(std::error_code& result, std::ostream& out, const void* data, const defaulttype::AbstractContainerTypeInfo* typeInfo) const
{
    if (typeInfo->containerSize(data) == 0)
    {
        out << "{}";
        return true;

    }
    else if (typeInfo->ContainerKind() == sofa::defaulttype::ContainerKindEnum::Map)
    {
        out << "{ ";
        const sofa::defaulttype::AbstractTypeInfo *keyTypeInfo = typeInfo->getKeyType();
        const sofa::defaulttype::AbstractTypeInfo *mappedTypeInfo = typeInfo->getMappedType();
        size_t i = 0;
        for (sofa::defaulttype::AbstractContainerTypeInfo::const_iterator it = typeInfo->cbegin(data); it != typeInfo->cend(data); ++it, ++i)
        {
            const void* keyData = typeInfo->getItemKey(it);
            const void* mappedData = typeInfo->getItemValue(it);
            if (i) out << ", ";

            out << "{ ";
            if (!fromDataDispatch(result, out, keyData, keyTypeInfo)) return false;
            out << ", ";
            if (!fromDataDispatch(result, out, mappedData, mappedTypeInfo)) return false;
            out << " }";
        }
        out << " }";
    }
    else
    {
        out << "{ ";
        const sofa::defaulttype::AbstractTypeInfo *mappedTypeInfo = typeInfo->getMappedType();
        if (typeInfo->containerSize(data) != 0)
        {
            int i = 0;
            for (sofa::defaulttype::AbstractContainerTypeInfo::const_iterator it = typeInfo->cbegin(data); it != typeInfo->cend(data); ++it, ++i)
            {
                if (i) out << ", ";
                const void* mappedData = typeInfo->getItemValue(it);
                if (!fromDataDispatch(result, out, mappedData, mappedTypeInfo)) return false;
            }
        }
        out << " }";
    }
    return true;
}

bool CppDataParser::fromDataInternal(std::error_code& result, std::ostream& out, const void* data, const defaulttype::AbstractStructureTypeInfo* typeInfo) const
{
    if (typeInfo->structSize() == 0)
    {
        out << "{}";
        return true;

    }
    out << "{ ";
    for (size_t i = 0u; i < typeInfo->structSize(); i++)
    {
        if (i) out << ", ";
        const sofa::defaulttype::AbstractTypeInfo* memberTypeInfo = typeInfo->getMemberTypeForIndex(i);
        if (m_useNamed)
        {
            out << "." << typeInfo->getMemberName(data, i) << "=";
        }
        if (!fromDataDispatch(result, out, typeInfo->getMemberValue(data, i), memberTypeInfo)) return false;
    }
    out << " }";
    return true;
}

bool CppDataParser::fromDataDispatch(std::error_code& result, std::ostream& out, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    if (typeInfo->ValidInfo())
    {
        if (typeInfo->IsSingleValue())
        {
            return fromDataInternal(result, out, data, typeInfo->SingleValueType());
        }
        else if (typeInfo->IsContainer())
        {
            return fromDataInternal(result, out, data, typeInfo->ContainerType());
        }
        else if (typeInfo->IsMultiValue())
        {
            return fromDataInternal(result, out, data, typeInfo->MultiValueType());
        }
        else if (typeInfo->IsStructure())
        {
            return fromDataInternal(result, out, data, typeInfo->StructureType());
        }
    }
    result = make_error_code(DataParserError::incorrect_type_info);
    return false;
}

std::error_code CppDataParser::fromData(std::ostream& os, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    std::error_code result{};
    fromDataDispatch(result, os, data, typeInfo);
    if (result)
    {
        std::cerr << "CppDataParser fromData FAILED for " << typeInfo->name() << ": "
            << result.category().name() << ' ' << result.value() << ' ' << result.message() << std::endl;
    }
    return result;
}

std::error_code CppDataParser::fromData(std::string& output, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    std::ostringstream stream;
    auto error_code = fromData(stream, data, typeInfo);
    output.append(stream.str());
    return error_code;
}

////////////////////////////////////////
//////////   WRITE TO DATA    //////////
////////////////////////////////////////

bool CppDataParser::toDataInternal(std::error_code& result, std::istream& in, void* data, const defaulttype::AbstractValueTypeInfo* typeInfo) const
{
    if (typeInfo->IsEnum())
    {
        const sofa::defaulttype::AbstractEnumTypeInfo* enumTypeInfo = typeInfo->EnumType();
        std::string value;
        in >> value;
        enumTypeInfo->resetValue(data);
        enumTypeInfo->setDataEnumeratorString(data, value);
        return true;
    }
    else return toDataInternal(result, in, data, typeInfo, 0); // AbstractValueTypeInfo derives from AbstractMultiValueTypeInfo
}

bool CppDataParser::toDataInternal(std::error_code& result, std::istream& in, void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo, size_t id) const
{
    switch(typeInfo->FinalValueKind())
    {
    case defaulttype::ValueKindEnum::Void:
    case defaulttype::ValueKindEnum::Pointer:
    {
        result = make_error_code(DataParserError::incorrect_type_info);
        return false;
    }
    case defaulttype::ValueKindEnum::Integer:
    {
        if (typeInfo->Unsigned())
        {
            unsigned long long value;
            in >> value;
            typeInfo->setFinalValueInteger(data, id, (long long)value);
            return true;
        }
        else
        {
            long long value;
            in >> value;
            typeInfo->setFinalValueInteger(data, id, value);
            return true;
        }
    }
    case defaulttype::ValueKindEnum::Scalar:
    {
        double value;
        in >> value;
        typeInfo->setFinalValueScalar(data, id, value);
        return true;
    }
    case defaulttype::ValueKindEnum::String:
    {
        std::string value;
        in >> std::quoted(value);
        typeInfo->setFinalValueString(data, id, value);
        return true;
    }
    case defaulttype::ValueKindEnum::Bool:
    {
        int c = in.peek();
        bool value;
        if (c == 't' || c == 'T' || c == 'f' || c == 'F')
        {
            value = (c == 't' || c == 'T');
            for (const char* text = value ? "true" : "false"; *text; ++text)
            {
                c = in.get();
                if (c != *text && c != (*text + ('A'-'a')))
                {
                    result = make_error_code(DataParserError::invalid_json);
                    return false;
                }
            }
        }
        else if (c == '-' || (c >= '0' && c <= '9'))
        {
            in >> c;
            value = (c != 0);
        }
        typeInfo->setFinalValueInteger(data, id, value);
        return true;
    }
    case defaulttype::ValueKindEnum::Enum:
    {
        std::string value;
        in >> value;
        typeInfo->setFinalValueString(data, id, value);
        return true;
    }
    }
    result = make_error_code(DataParserError::incorrect_type_info);
    return false;
}

bool CppDataParser::toDataInternal(std::error_code& result, std::istream& in, void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo) const
{
    typeInfo->resetValue(data);
    const bool fixedFinalSize = typeInfo->FixedFinalSize();
    const size_t rowWidth = typeInfo->FinalSize();
    in.ignore(std::numeric_limits<std::streamsize>::max(), '{');
    if (in.eof())
    {
        result = make_error_code(DataParserError::invalid_json);
        return false;
    }
    int index = 0;
    bool rowOpen = false;
    for(;;)
    {
        while (std::isspace(in.peek())) { in.get(); }
        if (index && in.peek() == ',')
        {
            in.get();
            while (std::isspace(in.peek())) { in.get(); }
        }
        if (in.eof())
        {
            result = make_error_code(DataParserError::invalid_json);
            return false;
        }
        if (in.peek() == '}')
        {
            in.get();
            break;
        }
        if (fixedFinalSize && index >= rowWidth)
        {
            result = make_error_code(DataParserError::multivalue_size_mismatch);
            return false;
        }

        if (!fixedFinalSize && rowWidth > 1 && index % rowWidth == 0)
        {
            while (std::isspace(in.peek())) { in.get(); }
            if (in.peek() == '{')
            {
                in.get();
                rowOpen = true;
            }
        }
        if (!fixedFinalSize)
        {
            typeInfo->setFinalSize(data, (rowWidth <= 1) ? index + 1 : ((index + rowWidth)/rowWidth) * rowWidth); // Not optimal
        }

        if (!toDataInternal(result, in, data, typeInfo, index)) return false;
        ++index;
        if (!fixedFinalSize && rowWidth > 1 && index % rowWidth == 0 && rowOpen)
        {
            while (std::isspace(in.peek())) { in.get(); }
            if (in.get() != '}')
            {
                result = make_error_code(DataParserError::invalid_json);
                return false;
            }
            rowOpen = false;
        }
    }
    return true;
}

bool CppDataParser::toDataInternal(std::error_code& result, std::istream& in, void* data, const defaulttype::AbstractContainerTypeInfo* typeInfo) const
{
    const sofa::defaulttype::ContainerKindEnum containerKind = typeInfo->ContainerKind();
    const bool fixedContainerSize = typeInfo->FixedContainerSize();
    const sofa::defaulttype::AbstractTypeInfo *keyTypeInfo = typeInfo->getKeyType();
    const sofa::defaulttype::AbstractTypeInfo *mappedTypeInfo = typeInfo->getMappedType();
    sofa::helper::SSOBuffer<> keyBuffer, mappedBuffer;
    typeInfo->resetValue(data);
    in.ignore(std::numeric_limits<std::streamsize>::max(), '{');
    int index = 0;
    for(;;)
    {
        while (std::isspace(in.peek())) { in.get(); }
        if (index && in.peek() == ',')
        {
            in.get();
            while (std::isspace(in.peek())) { in.get(); }
        }
        if (in.eof())
        {
            result = make_error_code(DataParserError::invalid_json);
            return false;
        }
        if (in.peek() == '}')
        {
            in.get();
            break;
        }
        if (containerKind == sofa::defaulttype::ContainerKindEnum::Map)
        {
            char c = 0;
            in >> c;
            if (c != '{')
            {
                result = make_error_code(DataParserError::invalid_json);
                return false;
            }

            void* keyData = typeInfo->newKey(data, keyBuffer);
            if (!toDataDispatch(result, in, keyData, keyTypeInfo)) return false;

            in >> c;
            if (c != ',')
            {
                result = make_error_code(DataParserError::invalid_json);
                return false;
            }

            void* mappedData = typeInfo->insertItem(data, keyData);
            if (!toDataDispatch(result, in, mappedData, mappedTypeInfo)) return false;

            in >> c;
            if (c != '}')
            {
                result = make_error_code(DataParserError::invalid_json);
                return false;
            }
        }
        else if (containerKind == sofa::defaulttype::ContainerKindEnum::Set)
        {
            void* keyData = typeInfo->newKey(data, keyBuffer);
            if (!toDataDispatch(result, in, keyData, keyTypeInfo)) return false;
            typeInfo->insertItem(data, keyData);
        }
        else //if (containerKind == sofa::defaulttype::ContainerKindEnum::Array )
        {
            if (!fixedContainerSize)
            {
                typeInfo->setContainerSize(data, index+1);
            }
            else if (index >= typeInfo->containerSize(data))
            {
                result = make_error_code(DataParserError::container_size_mismatch);
                return false;
            }
            void* mappedData = typeInfo->editItemValue(data, index);
            if (mappedData == nullptr)
            {
                result = make_error_code(DataParserError::container_size_mismatch);
                return false;
            }
            if (!toDataDispatch(result, in, mappedData, mappedTypeInfo)) return false;
        }
        ++index;
    }
    return true;
}

bool CppDataParser::toDataInternal(std::error_code& result, std::istream& in, void* data, const defaulttype::AbstractStructureTypeInfo* typeInfo) const
{
    const size_t structSize = typeInfo->structSize();
    typeInfo->resetValue(data);
    in.ignore(std::numeric_limits<std::streamsize>::max(), '{');
    int index = 0;
    for(;;)
    {
        while (std::isspace(in.peek())) { in.get(); }
        if (index && in.peek() == ',')
        {
            in.get();
            while (std::isspace(in.peek())) { in.get(); }
        }
        if (in.eof())
        {
            result = make_error_code(DataParserError::invalid_json);
            return false;
        }
        if (in.peek() == '}')
        {
            in.get();
            break;
        }
        int memberIndex = index;
        if (m_useNamed && in.peek() == '.')
        {
            in.get();
            std::string name;
            while(in.peek() != '=' && !std::isspace(in.peek()) && !in.eof())
            {
                name += (char)in.get();
            }
            in.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            memberIndex = 0;
            while ( memberIndex < structSize && typeInfo->getMemberName(data, memberIndex) != name) ++memberIndex;
        }
        if (memberIndex >= structSize)
        {
            result = make_error_code(DataParserError::structure_size_mismatch);
            return false;
        }
        if (!toDataDispatch(result, in, typeInfo->editMemberValue(data, memberIndex), typeInfo->getMemberTypeForIndex(memberIndex))) return false;
        ++index;
    }
    return true;
}

bool CppDataParser::toDataDispatch(std::error_code& result, std::istream& in, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    if (typeInfo->ValidInfo())
    {
        if (typeInfo->IsSingleValue())
        {
            return toDataInternal(result, in, data, typeInfo->SingleValueType());
        }
        else if (typeInfo->IsContainer())
        {
            return toDataInternal(result, in, data, typeInfo->ContainerType());
        }
        else if (typeInfo->IsMultiValue())
        {
            return toDataInternal(result, in, data, typeInfo->MultiValueType());
        }
        else if (typeInfo->IsStructure())
        {
            return toDataInternal(result, in, data, typeInfo->StructureType());
        }
    }
    result = make_error_code(DataParserError::incorrect_type_info);
    return false;
}

std::error_code CppDataParser::toData(std::istream& is, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    std::error_code result{};
    toDataDispatch(result, is, data, typeInfo);
    return result;
}

std::error_code CppDataParser::toData(const std::string& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    std::istringstream stream(input);
    return toData(stream, data, typeInfo);
}

} // namespace dataparser

} // namespace core

} // namespace sofa
