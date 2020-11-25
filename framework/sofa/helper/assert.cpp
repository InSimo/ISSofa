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
#include "assert.h"

#include <iostream>
#include <stdarg.h>
#include <stdio.h>

namespace sofa
{
namespace assertion
{

Action defaultAssertionHandler(const char* condition, const char* file, unsigned int line, const char* msg);

Action silentHalt(const char* /*condition*/, const char* /*file*/, unsigned int /*line*/, const char* /*msg*/)
{
    return sofa::assertion::HALT;
}


namespace
{

//////////////////////////////////////////////////
FnAssertionHandler& getHandler()
{
    static FnAssertionHandler handler = &defaultAssertionHandler;
    return handler;
}

} // namespace

//////////////////////////////////////////////////
void setHandler(FnAssertionHandler handler)
{
    getHandler() = handler;
}

//////////////////////////////////////////////////
Action onAssertionFailure(const char* condition, const char* file, unsigned int line, const char* msg)
{
    const FnAssertionHandler& fnHandler = getHandler();
    if (fnHandler != NULL) {
        return fnHandler(condition, file, line, msg);
    }

    return HALT;
}

//////////////////////////////////////////////////
Action onAssertionFailureFormatted(const char* condition, const char* file, unsigned int line, const char* msg, ...)
{
    if (msg != NULL)
    {
        char buffer[1024];

        {
            va_list args;
            va_start(args, msg);
            vsnprintf(buffer, sizeof(buffer), msg, args);
            va_end(args);
        }

        return onAssertionFailure(condition, file, line, buffer);
    }

    return onAssertionFailure(condition, file, line, msg);
}

} // namespace assertion
} // namespace sofa
