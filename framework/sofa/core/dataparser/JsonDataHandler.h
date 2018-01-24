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
#ifndef SOFA_CORE_JSONDATAHANDLER_H
#define SOFA_CORE_JSONDATAHANDLER_H

#include <stack>
#include <sofa/SofaFramework.h>
#include <sofa/helper/SSOBuffer.h>
#include "DataParserError.h"

namespace sofa
{
namespace defaulttype
{
class AbstractTypeInfo;
}
}

namespace sofa
{

namespace core
{

namespace dataparser
{

class SOFA_CORE_API DataHandler
{
    public:
        using SizeType = unsigned int;

        DataHandler(void* data, const defaulttype::AbstractTypeInfo* typeInfo);

        // RapidJson handler API
        bool Null();
        bool Bool(bool b);
        bool Int(int i);
        bool Uint(unsigned u);
        bool Int64(int64_t i);
        bool Uint64(uint64_t u);
        bool Double(double d);
        bool RawNumber(const char* str, SizeType length, bool copy);
        bool String(const char* str, SizeType length, bool copy);
        bool Key(const char* str, SizeType length, bool copy);
        bool StartObject();
        bool EndObject(SizeType memberCount);
        bool StartArray();
        bool EndArray(SizeType elementCount);
        // End RapidJson handler API

        template <typename T>
        bool AllInteger(T i);

        void BeginIteration();
        void EndIteration();
        void FindType();

        std::error_code getErrorCode();

    protected:
        struct State
        {
            enum class Type
            {
                SingleValueType,
                MultiValueType,
                ContainerType,
                StructType
            };

            void* data;
            const defaulttype::AbstractTypeInfo* typeInfo;

            Type type;

            size_t index;
            sofa::helper::SSOBuffer<> buffer; // Buffer is either a key or a pointer to data
        };

        // This stack uses a deque as a container
        // It's slow but needed. Using a vector will not work because of memory reallocation.
        // Some data point to SSOBuffer and the pointer needs to stay the same
        // NOTE: To improve the performance, we could use something like C++17 pmr::unsynchronized_pool_resource as a member of JsonDataParser
        std::stack<State> m_stack;
        bool isInRoot = true;

        std::error_code m_error_code;
};

} // namespace dataparser

} // namespace core

} // namespace sofa

#endif //SOFA_CORE_JSONDATAHANDLER_H
