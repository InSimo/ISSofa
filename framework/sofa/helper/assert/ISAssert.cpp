/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#include "ISAssert.h"

#include <iostream>
#include <stdarg.h>
#include <stdio.h>

namespace issystem
{
namespace assertion
{

Action defaultAssertionHandler(const char* condition, const char* file, unsigned int line, const char* msg);

Action silentHalt(const char* /*condition*/, const char* /*file*/, unsigned int /*line*/, const char* /*msg*/)
{
    return issystem::assertion::HALT;
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
} // namespace issystem
