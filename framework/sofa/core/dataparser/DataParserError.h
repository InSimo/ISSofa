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
#ifndef SOFA_CORE_DATAPARSERERROR_H
#define SOFA_CORE_DATAPARSERERROR_H

#include <sofa/SofaFramework.h>
#include <system_error>

namespace sofa
{

namespace core
{

namespace dataparser
{

enum class DataParserError : int
{
    container_size_mismatch = 1,
    multivalue_size_mismatch,
    structure_size_mismatch,
    incorrect_type_info,
    unsupported_operation,
    invalid_json
};

SOFA_CORE_API const std::error_category& getDataParserErrorCategory();

SOFA_CORE_API std::string getDataParserErrorMessage(int value);

class DataParserErrorCategory : public std::error_category
{
public:
    const char* name() const noexcept override
    {
        return "sofa_data_parser";
    }
    std::string message(int value) const override
    {
        return getDataParserErrorMessage(value);
    }

    std::error_condition default_error_condition(int /*value*/) const noexcept override
    {
        return std::errc::io_error;
    }
};

inline std::error_code make_error_code(DataParserError ec)
{
    return std::error_code(static_cast<int>(ec), getDataParserErrorCategory());
}

inline std::error_condition make_error_condition(DataParserError ec)
{
    return std::error_condition(static_cast<int>(ec), getDataParserErrorCategory());
}

} // namespace dataparser

} // namespace core

} // namespace sofa

namespace std
{
    template <> struct is_error_code_enum<sofa::core::dataparser::DataParserError> : std::true_type {};
}

#endif //SOFA_CORE_DATAPARSERERROR_H
