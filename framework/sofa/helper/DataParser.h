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
class AbstractMultiValueTypeInfo;
class AbstractContainerTypeInfo;
template<class TypeInfo, typename DataPtr>
class TypeInfoItemIterator;
} // namespace defaulttype


namespace helper
{

class DataParser
{
public:
    using ParserId = size_t;
    using ContainerIterator      = defaulttype::TypeInfoItemIterator<defaulttype::AbstractContainerTypeInfo,       void*>;
    using ContainerConstIterator = defaulttype::TypeInfoItemIterator<defaulttype::AbstractContainerTypeInfo, const void*>;

    DataParser(std::string name);

    ParserId getId() const;

    // Read input stream or string into the Data
    virtual std::error_code toData(std::istream& is, void* data, const defaulttype::AbstractTypeInfo* typeInfo) = 0;
    virtual std::error_code toData(const std::string& input, void* data, const defaulttype::AbstractTypeInfo* typeInfo) = 0;

    // Read input stream or string into a subrange of the Data
    virtual std::error_code toDataRange(std::istream& os, void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo, size_t begin, size_t end);
    virtual std::error_code toDataRange(std::istream& os, void* data, const defaulttype::AbstractContainerTypeInfo* typeInfo, ContainerIterator& begin, ContainerIterator& end);
    virtual std::error_code toDataRange(const std::string& input, void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo, size_t begin, size_t end);
    virtual std::error_code toDataRange(const std::string& input, void* data, const defaulttype::AbstractContainerTypeInfo* typeInfo, ContainerIterator& begin, ContainerIterator& end);

    // Write the Data into an output stream or string
    virtual std::error_code fromData(std::ostream& os, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) = 0;
    virtual std::error_code fromData(std::string& output, const void* data, const defaulttype::AbstractTypeInfo* typeInfo) = 0;

    // Write a subrange of the Data into an output stream or string
    virtual std::error_code fromDataRange(std::ostream& os, const void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo, size_t begin, size_t end);
    virtual std::error_code fromDataRange(std::ostream& os, const void* data, const defaulttype::AbstractContainerTypeInfo* typeInfo, ContainerConstIterator& begin, ContainerConstIterator& end);
    virtual std::error_code fromDataRange(std::string& output, const void* data, const defaulttype::AbstractMultiValueTypeInfo* typeInfo, size_t begin, size_t end);
    virtual std::error_code fromDataRange(std::string& output, const void* data, const defaulttype::AbstractContainerTypeInfo* typeInfo, ContainerConstIterator& begin, ContainerConstIterator& end);

private:
    ParserId m_id;
};

static inline DataParser::ParserId generateDataParserId(std::string name)
{
    return std::hash<std::string>{}(std::move(name));
}

inline DataParser::ParserId DataParser::getId() const
{
    return m_id;
}

inline DataParser::DataParser(std::string name) : m_id(generateDataParserId(std::move(name)))
{
}

// toDataRange and fromDataRange default to returning an "unsupported_operation" error
inline std::error_code DataParser::toDataRange(std::istream&, void*, const defaulttype::AbstractMultiValueTypeInfo*, size_t, size_t)
{
    return make_error_code(DataParserError::unsupported_operation);
}
inline std::error_code DataParser::toDataRange(std::istream&, void*, const defaulttype::AbstractContainerTypeInfo*, DataParser::ContainerIterator&, DataParser::ContainerIterator& )
{
    return make_error_code(DataParserError::unsupported_operation);
}
inline std::error_code DataParser::toDataRange(const std::string&, void*, const defaulttype::AbstractMultiValueTypeInfo*, size_t, size_t )
{
    return make_error_code(DataParserError::unsupported_operation);
}
inline std::error_code DataParser::toDataRange(const std::string&, void*, const defaulttype::AbstractContainerTypeInfo*, DataParser::ContainerIterator&, DataParser::ContainerIterator& )
{
    return make_error_code(DataParserError::unsupported_operation);
}
inline std::error_code DataParser::fromDataRange(std::ostream&, const void*, const defaulttype::AbstractMultiValueTypeInfo*, size_t, size_t)
{
    return make_error_code(DataParserError::unsupported_operation);
}
inline std::error_code DataParser::fromDataRange(std::ostream&, const void*, const defaulttype::AbstractContainerTypeInfo*, DataParser::ContainerConstIterator&, DataParser::ContainerConstIterator&)
{
    return make_error_code(DataParserError::unsupported_operation);
}
inline std::error_code DataParser::fromDataRange(std::string&, const void*, const defaulttype::AbstractMultiValueTypeInfo*, size_t, size_t)
{
    return make_error_code(DataParserError::unsupported_operation);
}
inline std::error_code DataParser::fromDataRange(std::string&, const void*, const defaulttype::AbstractContainerTypeInfo*, DataParser::ContainerConstIterator&, DataParser::ContainerConstIterator&)
{
    return make_error_code(DataParserError::unsupported_operation);
}

} // namespace helper

} // namespace sofa

#endif //SOFA_HELPER_DATAPARSER_H
