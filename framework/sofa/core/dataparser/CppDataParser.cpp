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

struct CppDataParser::FromDataState
{
    std::error_code result {};
    int level = 0;
    int indent = 4;
};
struct CppDataParser::ToDataState
{
    std::error_code result {};
};

bool CppDataParser::isLargeType(const defaulttype::AbstractTypeInfo* typeInfo) const
{
    bool result = false;
    if (typeInfo->ValidInfo())
    {
        if (typeInfo->IsSingleValue())
        {
            return false;
        }
        else if (typeInfo->IsContainer())
        {
            auto t = typeInfo->ContainerType();
            const bool fixedContainerSize = t->FixedContainerSize();
            auto keyTypeInfo = t->getKeyType();
            auto mappedTypeInfo = t->getMappedType();
            result |= !fixedContainerSize;
            if (!result)
            { // recurse only if we don't already know this is a large type
                result |= isLargeType(keyTypeInfo);
                result |= isLargeType(mappedTypeInfo);
            }
        }
        else if (typeInfo->IsMultiValue())
        {
            auto t = typeInfo->MultiValueType();
            const bool fixedFinalSize = t->FixedFinalSize();
            const size_t rowWidth = t->FinalSize();
            result |= !fixedFinalSize;
            result |= (rowWidth >= 10);
        }
        else if (typeInfo->IsStructure())
        {
            auto t = typeInfo->StructureType();
            const std::size_t ssize = t->structSize();
            result |= (ssize >= 10);
            // recurse only if we don't already know this is a large type
            for (size_t i = 0u; i < ssize && !result; i++)
            {
                result |= isLargeType(t->getMemberTypeForIndex(i));
            }
        }
    }
    return result;
}

////////////////////////////////////////
//////////   READ FROM DATA   //////////
////////////////////////////////////////

bool CppDataParser::fromDataInternal(FromDataState& state, std::ostream& out, const void* data, const defaulttype::AbstractValueTypeInfo* typeInfo) const
{
    if (typeInfo->IsEnum())
    {
        const sofa::defaulttype::AbstractEnumTypeInfo* enumTypeInfo = typeInfo->EnumType();
        out << enumTypeInfo->getDataEnumeratorString(data);
        return true;
    }
    else return fromDataInternal(state, out, data, typeInfo, 0); // AbstractValueTypeInfo derives from AbstractMultiValueTypeInfo
}

bool CppDataParser::fromDataInternal(FromDataState& state, std::ostream& out, const void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo, size_t id) const
{
    switch(typeInfo->FinalValueKind())
    {
    case defaulttype::ValueKindEnum::Void:
    case defaulttype::ValueKindEnum::Pointer:
    {
        state.result = make_error_code(DataParserError::incorrect_type_info);
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
        bool value = (typeInfo->getFinalValueInteger(data, id) != 0);
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
    state.result = make_error_code(DataParserError::incorrect_type_info);
    return false;
}

bool CppDataParser::fromDataInternal(FromDataState& state, std::ostream& out, const void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo) const
{
    const size_t rowWidth = typeInfo->FinalSize();
    const size_t dataFinalSize = typeInfo->finalSize(data);
    const size_t nbRows = dataFinalSize / rowWidth;

    if (dataFinalSize == 0)
    {
        out << "{}";
        return true;
    }
    bool newlines = (state.level <= 1 && rowWidth >= 10 && nbRows > 1);
    out << (newlines ? std::string("\n") + std::string( state.level*state.indent, ' ') + std::string("{") + std::string( state.indent-1, ' ') : std::string("{ "));
    ++state.level;
    std::string separator = (newlines ? std::string(",\n") + std::string( state.level*state.indent, ' ') : std::string(", "));
    if (nbRows == 1 || rowWidth == 1)
    {
        for (size_t i = 0; i < dataFinalSize; i++)
        {
            if (i) out << (newlines ? ",\n  " : ", ");
            if (!fromDataInternal(state, out, data, typeInfo, i)) return false;
        }
    }
    else
    {
        for (size_t i = 0; i < nbRows; i++)
        {
            if (i) out << separator;
            out << "{ ";
            ++state.level;
            for (size_t j = 0; j < rowWidth; j++)
            {
                if (j) out << ", ";
                if (!fromDataInternal(state, out, data, typeInfo, i * rowWidth + j)) return false;
            }
            --state.level;
            out << " }";
        }
    }
    --state.level;
    out << (newlines ? std::string("\n") + std::string( state.level*state.indent, ' ') + std::string("}") : std::string(" }"));
    return true;
}

bool CppDataParser::fromDataInternal(FromDataState& state, std::ostream& out, const void* data, const defaulttype::AbstractContainerTypeInfo* typeInfo) const
{
    std::size_t csize = typeInfo->containerSize(data);
    bool isMap = (typeInfo->ContainerKind() == sofa::defaulttype::ContainerKindEnum::Map);
    if (csize == 0)
    {
        out << "{}";
        return true;

    }
    else
    {
        const sofa::defaulttype::AbstractTypeInfo *keyTypeInfo = typeInfo->getKeyType();
        const sofa::defaulttype::AbstractTypeInfo *mappedTypeInfo = typeInfo->getMappedType();
        bool newlines = (state.level <= 1 && (isLargeType(keyTypeInfo) || isLargeType(mappedTypeInfo)));
        out << (newlines ? std::string("\n") + std::string( state.level*state.indent, ' ') + std::string("{") + std::string( state.indent-1, ' ') : std::string("{ "));
        ++state.level;
        std::string separator = (newlines ? std::string(",\n") + std::string( state.level*state.indent, ' ') : std::string(", "));
        size_t i = 0;
        for (sofa::defaulttype::AbstractContainerTypeInfo::const_iterator it = typeInfo->cbegin(data); it != typeInfo->cend(data); ++it, ++i)
        {
            if (i) out << separator;
            const void* mappedData = typeInfo->getItemValue(it);
            if (isMap)
            {
                const void* keyData = typeInfo->getItemKey(it);
                out << "{ ";
                ++state.level;
                if (!fromDataDispatch(state, out, keyData, keyTypeInfo)) return false;
                out << ", ";
                if (!fromDataDispatch(state, out, mappedData, mappedTypeInfo)) return false;
                --state.level;
                out << " }";
            }
            else
            {
                if (!fromDataDispatch(state, out, mappedData, mappedTypeInfo)) return false;
            }
        }
        --state.level;
        out << (newlines ? std::string("\n") + std::string( state.level*state.indent, ' ') + std::string("}") : std::string(" }"));
    }
    return true;
}

bool CppDataParser::fromDataInternal(FromDataState& state, std::ostream& out, const void* data, const defaulttype::AbstractStructureTypeInfo* typeInfo) const
{
    const std::size_t ssize = typeInfo->structSize();
    if (ssize == 0)
    {
        out << "{}";
        return true;

    }
    bool newlines = (state.level <= 1 && isLargeType(typeInfo));
    out << (newlines ? std::string("\n") + std::string( state.level*state.indent, ' ') + std::string("{") + std::string( state.indent-1, ' ') : std::string("{ "));
    ++state.level;
    std::string separator = (newlines ? std::string(",\n") + std::string( state.level*state.indent, ' ') : std::string(", "));
    for (size_t i = 0u; i < ssize; i++)
    {
        if (i) out << separator;
        const sofa::defaulttype::AbstractTypeInfo* memberTypeInfo = typeInfo->getMemberTypeForIndex(i);
        if (m_useNamed)
        {
            out << "." << typeInfo->getMemberName(data, i) << "=";
        }
        if (!fromDataDispatch(state, out, typeInfo->getMemberValue(data, i), memberTypeInfo)) return false;
    }
    --state.level;
    out << (newlines ? std::string("\n") + std::string( state.level*state.indent, ' ') + std::string("}") : std::string(" }"));
    return true;
}

bool CppDataParser::fromDataDispatch(FromDataState& state, std::ostream& out, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    if (typeInfo->ValidInfo())
    {
        if (typeInfo->IsSingleValue())
        {
            return fromDataInternal(state, out, data, typeInfo->SingleValueType());
        }
        else if (typeInfo->IsContainer())
        {
            return fromDataInternal(state, out, data, typeInfo->ContainerType());
        }
        else if (typeInfo->IsMultiValue())
        {
            return fromDataInternal(state, out, data, typeInfo->MultiValueType());
        }
        else if (typeInfo->IsStructure())
        {
            return fromDataInternal(state, out, data, typeInfo->StructureType());
        }
    }
    state.result = make_error_code(DataParserError::incorrect_type_info);
    return false;
}

std::error_code CppDataParser::fromData(std::ostream& os, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    FromDataState state;
    fromDataDispatch(state, os, data, typeInfo);
    if (state.result)
    {
        std::cerr << "CppDataParser fromData FAILED for " << typeInfo->name() << ": "
            << state.result.category().name() << ' ' << state.result.value() << ' ' << state.result.message() << std::endl;
    }
    else if (state.level != 0)
    {
        std::cerr << "CppDataParser fromData recursion level MISMATCHED for " << typeInfo->name() << std::endl;
    }
    return state.result;
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

bool CppDataParser::toDataInternal(ToDataState& state, std::istream& in, void* data, const defaulttype::AbstractValueTypeInfo* typeInfo) const
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
    else return toDataInternal(state, in, data, typeInfo, 0); // AbstractValueTypeInfo derives from AbstractMultiValueTypeInfo
}

bool CppDataParser::toDataInternal(ToDataState& state, std::istream& in, void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo, size_t id) const
{
    switch(typeInfo->FinalValueKind())
    {
    case defaulttype::ValueKindEnum::Void:
    case defaulttype::ValueKindEnum::Pointer:
    {
        state.result = make_error_code(DataParserError::incorrect_type_info);
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
                    state.result = make_error_code(DataParserError::invalid_json);
                    return false;
                }
            }
        }
        else if (c == '-' || (c >= '0' && c <= '9'))
        {
            in >> c;
            value = (c != 0);
        }
        else
        {
            state.result = make_error_code(DataParserError::invalid_json);
            return false;
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
    state.result = make_error_code(DataParserError::incorrect_type_info);
    return false;
}

bool CppDataParser::toDataInternal(ToDataState& state, std::istream& in, void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo) const
{
    typeInfo->resetValue(data);
    const bool fixedFinalSize = typeInfo->FixedFinalSize();
    const size_t rowWidth = typeInfo->FinalSize();
    in.ignore(std::numeric_limits<std::streamsize>::max(), '{');
    if (in.eof())
    {
        state.result = make_error_code(DataParserError::invalid_json);
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
            state.result = make_error_code(DataParserError::invalid_json);
            return false;
        }
        if (in.peek() == '}')
        {
            in.get();
            break;
        }
        if (fixedFinalSize && index >= rowWidth)
        {
            state.result = make_error_code(DataParserError::multivalue_size_mismatch);
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

        if (!toDataInternal(state, in, data, typeInfo, index)) return false;
        ++index;
        if (!fixedFinalSize && rowWidth > 1 && index % rowWidth == 0 && rowOpen)
        {
            while (std::isspace(in.peek())) { in.get(); }
            if (in.get() != '}')
            {
                state.result = make_error_code(DataParserError::invalid_json);
                return false;
            }
            rowOpen = false;
        }
    }
    return true;
}

bool CppDataParser::toDataInternal(ToDataState& state, std::istream& in, void* data, const defaulttype::AbstractContainerTypeInfo* typeInfo) const
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
            state.result = make_error_code(DataParserError::invalid_json);
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
                state.result = make_error_code(DataParserError::invalid_json);
                return false;
            }

            void* keyData = typeInfo->newKey(data, keyBuffer);
            if (!toDataDispatch(state, in, keyData, keyTypeInfo)) return false;

            in >> c;
            if (c != ',')
            {
                state.result = make_error_code(DataParserError::invalid_json);
                return false;
            }

            void* mappedData = typeInfo->insertItem(data, keyData);
            if (!toDataDispatch(state, in, mappedData, mappedTypeInfo)) return false;

            in >> c;
            if (c != '}')
            {
                state.result = make_error_code(DataParserError::invalid_json);
                return false;
            }
        }
        else if (containerKind == sofa::defaulttype::ContainerKindEnum::Set)
        {
            void* keyData = typeInfo->newKey(data, keyBuffer);
            if (!toDataDispatch(state, in, keyData, keyTypeInfo)) return false;
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
                state.result = make_error_code(DataParserError::container_size_mismatch);
                return false;
            }
            void* mappedData = typeInfo->editItemValue(data, index);
            if (mappedData == nullptr)
            {
                state.result = make_error_code(DataParserError::container_size_mismatch);
                return false;
            }
            if (!toDataDispatch(state, in, mappedData, mappedTypeInfo)) return false;
        }
        ++index;
    }
    return true;
}

bool CppDataParser::toDataInternal(ToDataState& state, std::istream& in, void* data, const defaulttype::AbstractStructureTypeInfo* typeInfo) const
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
            state.result = make_error_code(DataParserError::invalid_json);
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
            state.result = make_error_code(DataParserError::structure_size_mismatch);
            return false;
        }
        if (!toDataDispatch(state, in, typeInfo->editMemberValue(data, memberIndex), typeInfo->getMemberTypeForIndex(memberIndex))) return false;
        ++index;
    }
    return true;
}

bool CppDataParser::toDataDispatch(ToDataState& state, std::istream& in, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    if (typeInfo->ValidInfo())
    {
        if (typeInfo->IsSingleValue())
        {
            return toDataInternal(state, in, data, typeInfo->SingleValueType());
        }
        else if (typeInfo->IsContainer())
        {
            return toDataInternal(state, in, data, typeInfo->ContainerType());
        }
        else if (typeInfo->IsMultiValue())
        {
            return toDataInternal(state, in, data, typeInfo->MultiValueType());
        }
        else if (typeInfo->IsStructure())
        {
            return toDataInternal(state, in, data, typeInfo->StructureType());
        }
    }
    state.result = make_error_code(DataParserError::incorrect_type_info);
    return false;
}

std::error_code CppDataParser::toData(std::istream& is, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    ToDataState state;
    toDataDispatch(state, is, data, typeInfo);
    return state.result;
}

std::error_code CppDataParser::toData(const std::string& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo) const
{
    std::istringstream stream(input);
    return toData(stream, data, typeInfo);
}

} // namespace dataparser

} // namespace core

} // namespace sofa
