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
#include "JsonDataHandler.h"
#include <sofa/defaulttype/AbstractTypeInfo.h>

namespace sofa
{

namespace core
{

namespace dataparser
{

DataHandler::DataHandler(void* data, const defaulttype::AbstractTypeInfo* typeInfo)
{
    m_stack.emplace();
    m_stack.top().data = data;
    m_stack.top().typeInfo = typeInfo;
}

// RapidJson handler API
bool DataHandler::Null() { m_error_code = make_error_code(DataParserError::incorrect_type_info); return false; }
bool DataHandler::Bool(bool /*b*/) { m_error_code = make_error_code(DataParserError::incorrect_type_info); return false; }

template <typename T>
bool DataHandler::AllInteger(T i)
{
    BeginIteration();
    auto& state = m_stack.top();
    if (state.type == State::Type::SingleValueType)
    {
        state.typeInfo->SingleValueType()->setDataValueInteger(state.data, i);
        EndIteration();
        return true;
    }
    else if (state.type == State::Type::MultiValueType)
    {
        state.typeInfo->MultiValueType()->setFinalValueInteger(state.data, state.index, i);
        EndIteration();
        return true;
    }
    return false;
}

bool DataHandler::Int(int i)
{
    return AllInteger(i);
}

bool DataHandler::Uint(unsigned u)
{
    return AllInteger(u);
}

bool DataHandler::Int64(int64_t i)
{
    return AllInteger(i);
}

bool DataHandler::Uint64(uint64_t u)
{
    return AllInteger(u);
}

bool DataHandler::Double(double d)
{
    BeginIteration();
    auto& state = m_stack.top();
    if (state.type == State::Type::SingleValueType)
    {
        state.typeInfo->SingleValueType()->setDataValueScalar(state.data, d);
        EndIteration();
        return true;
    }
    else if (state.type == State::Type::MultiValueType)
    {
        state.typeInfo->MultiValueType()->setFinalValueScalar(state.data, state.index, d);
        EndIteration();
        return true;
    }
    return false;
}

bool DataHandler::RawNumber(const char* /*str*/, SizeType /*length*/, bool /*copy*/) { m_error_code = make_error_code(DataParserError::incorrect_type_info); return false; }

bool DataHandler::String(const char* str, SizeType length, bool /*copy*/)
{
    BeginIteration();
    auto& state = m_stack.top();
    if (state.type == State::Type::SingleValueType)
    {
        state.typeInfo->SingleValueType()->setDataValueString(state.data, std::string(str, length));
        EndIteration();
        return true;
    }
    else if (state.type == State::Type::MultiValueType)
    {
        state.typeInfo->MultiValueType()->setFinalValueString(state.data, state.index, std::string(str, length));
        EndIteration();
        return true;
    }
    return false;
}

bool DataHandler::Key(const char* str, SizeType length, bool /*copy*/)
{
    auto& state = m_stack.top();
    switch (state.type) {
    case State::Type::ContainerType:
    {
        auto* containerTypeinfo = state.typeInfo->ContainerType();

        if (containerTypeinfo->ContainerKind() == sofa::defaulttype::ContainerKindEnum::Map)
        {
            defaulttype::AbstractTypeInfo* keyTypeInfo = containerTypeinfo->getKeyType();
            void* keyData = containerTypeinfo->newKey(state.data, state.buffer);
            keyTypeInfo->setDataValueString(keyData, std::string(str, length));
            void* valueData = containerTypeinfo->insertItem(state.data, keyData);
            void** ptr = state.buffer.create<void*>();
            *ptr = valueData;
        }
        else
        {
            m_error_code = make_error_code(DataParserError::incorrect_type_info);
            return false;
        }
        break;
    }
    case State::Type::StructType:
    {
        std::string memberName(str, length);
        auto* structTypeinfo = state.typeInfo->StructureType();
        size_t structSize = structTypeinfo->structSize();
        for (size_t id = 0u; id < structSize; id++)
        {
            if (structTypeinfo->getMemberName(state.data, id) == memberName)
            {
                state.index = id;
                void* valueData = structTypeinfo->editMemberValue(state.data, id);
                void** ptr = state.buffer.create<void*>();
                *ptr = valueData;
                break;
            }
        }
        break;
    }
    default:
        m_error_code = make_error_code(DataParserError::incorrect_type_info);
        return false;
    }
    return true;
}

bool DataHandler::StartObject()
{
    BeginIteration();
    auto& state = m_stack.top();
    if (state.type == State::Type::ContainerType)
    {
        auto* containerTypeinfo = state.typeInfo->ContainerType();
        if (containerTypeinfo->ContainerKind() == sofa::defaulttype::ContainerKindEnum::Map)
        {
            containerTypeinfo->resetValue(state.data);
        }
    }
    return true;
}
bool DataHandler::EndObject(SizeType /*memberCount*/)
{
    EndIteration();
    return true;
}
bool DataHandler::StartArray()
{
    BeginIteration();
    auto& state = m_stack.top();
    switch (state.type) {
    case State::Type::ContainerType:
    {
        auto* containerTypeinfo = state.typeInfo->ContainerType();
        containerTypeinfo->resetValue(state.data);
        state.index = 0u;
        break;
    }
    case State::Type::MultiValueType:
    {
        auto* multiValueTypeinfo = state.typeInfo->MultiValueType();
        multiValueTypeinfo->resetValue(state.data);
        state.index = 0u;
        break;
    }
    default:
        m_error_code = make_error_code(DataParserError::incorrect_type_info);
        return false;
    }

    return true;
}
bool DataHandler::EndArray(SizeType /*elementCount*/)
{
    EndIteration();
    return true;
}
// End RapidJson handler API

void DataHandler::BeginIteration()
{
    if (!isInRoot)
    {
        auto& state = m_stack.top();
        switch (state.type) {
        case State::Type::ContainerType:
        {
            State nextState;
            auto* containerTypeinfo = state.typeInfo->ContainerType();
            nextState.typeInfo = containerTypeinfo->getMappedType();

            switch (containerTypeinfo->ContainerKind()) {
            case sofa::defaulttype::ContainerKindEnum::Array:
                nextState.data = containerTypeinfo->insertItem(state.data, &state.index);
                break;
            case sofa::defaulttype::ContainerKindEnum::Set:
                nextState.data = containerTypeinfo->newKey(state.data, state.buffer);
                break;
            case sofa::defaulttype::ContainerKindEnum::Map:
                nextState.data = *(state.buffer.get<void*>());
                break;
            default:
                break;
            }

            m_stack.emplace(std::move(nextState));
            FindType();
            break;
        }
        case State::Type::StructType:
        {
            State nextState;
            auto* structTypeinfo = state.typeInfo->StructureType();
            nextState.typeInfo = structTypeinfo->getMemberTypeForIndex(state.index);
            nextState.data = *(state.buffer.get<void*>());
            m_stack.emplace(std::move(nextState));
            FindType();
            break;
        }
        case State::Type::MultiValueType:
        {
            auto* multiValueTypeinfo = state.typeInfo->MultiValueType();
            multiValueTypeinfo->setFinalSize(state.data, state.index + 1); // Not optimal
            break;
        }
        default:
            m_error_code = make_error_code(DataParserError::incorrect_type_info);
        }
    }
    else
    {
        FindType();
        isInRoot = false;
    }
}

void DataHandler::EndIteration()
{
    auto& currentState = m_stack.top();
    if (currentState.type == State::Type::MultiValueType)
    {
        currentState.index++;
    }
    else
    {
        m_stack.pop();
        if (!m_stack.empty())
        {
            auto& state = m_stack.top();
            if (state.type == State::Type::ContainerType) // Switch?
            {
                state.index++;
                auto* containerTypeinfo = state.typeInfo->ContainerType();
                if (containerTypeinfo->ContainerKind() == sofa::defaulttype::ContainerKindEnum::Set)
                {
                    void* key = containerTypeinfo->newKey(state.data, state.buffer); // Will return the correct key
                    containerTypeinfo->insertItem(state.data, key);
                }
            }
        }
    }
}

void DataHandler::FindType()
{
    auto& state = m_stack.top();
    if (state.typeInfo->IsSingleValue())
    {
        state.type = State::Type::SingleValueType;
    }
    else if (state.typeInfo->IsContainer())
    {
        state.type = State::Type::ContainerType;
    }
    else if (state.typeInfo->IsMultiValue())
    {
        state.type = State::Type::MultiValueType;
    }
    else if (state.typeInfo->IsStructure())
    {
        state.type = State::Type::StructType;
    }
}

std::error_code DataHandler::getErrorCode()
{
    return m_error_code;
}

} // namespace dataparser

} // namespace core

} // namespace sofa
