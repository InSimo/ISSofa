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

/**
Assertions are meant to detect programming errors by specifying conditions that are always supposed to be true.
Their sole purpose is to alert the programmer when the application is in a invalid state,
they must not be used to deal with user related errors, such as clicking the wrong button.

Two categories of assertions macros are declared here : normal and fast.
Both have the same behavior, but either one or the other is the most adequate depending on the context:

- Fast assertions should be used to check statements that don't have any impact on performance,
  so that they can be activated even in release mode if desired.
  The corresponding macros are SOFA_ASSERT_FAST and SOFA_ASSERT_FAST_MSG.
  They can be enabled by defining SOFA_FAST_ASSERTION_ENABLED.

- Normal assertions should be used in all other cases and are usually activated in debug mode only.
  The corresponding macros are SOFA_ASSERT and SOFA_ASSERT_MSG.
  They can be enabled by defining SOFA_ASSERTION_ENABLED.

NOTE: Fast assertions are implicitly enabled along with normal assertions.

In case a condition is false, a handler callback is invoked.
A handler is provided by default that prints the assertion condition and a possible message,
but another can be set as such:

    sofa::assertion::setHandler(&myHandler);

The handler must be a function with the following signature:

    Action myHandler(const char* condition, const char* file, unsigned int line, const char* msg);

A handler can return:
- HALT to break execution of the application
- CONTINUE to move along after the assertion

Usage:
    SOFA_ASSERT(condition);
    SOFA_ASSERT_MSG(condition, message);
    SOFA_ASSERT_MSG(condition, message, argument1, argument2, ..., argument_n);

Examples:
    SOFA_ASSERT(i >= 0 && i < container.size());
    SOFA_ASSERT_MSG(i >= 0 && i < container.size(), "Index out of bound");
    SOFA_ASSERT_MSG(i >= 0 && i < container.size(), "Index out of bound: index is %u but size is %u", i, container.size());

Thanks to Noel Llopis and Charles Nicholson of Power of Two Games: http://cnicholson.net/2009/02/stupid-c-tricks-adventures-in-assert/.
*/

#ifndef SOFA_HELPER_ASSERT_H
#define SOFA_HELPER_ASSERT_H

#include <sofa/SofaFramework.h>

#if defined(__GNUC__)
#   include <sofa/helper/assert/GCC/AssertPlatform.h>
#elif defined(_WIN32)
#   include <sofa/helper/assert/MSVC/AssertPlatform.h>
#else
#   error Unknown compiler
#endif

#if defined (__clang__)
// Ignore warning : "token pasting of ',' and __VA_ARGS__ is a GNU extension" caused by the note below
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-zero-variadic-macro-arguments"
#endif

#if _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4100 )
#pragma warning( disable : 4505 )
#endif

#define SOFA_ASSERT_UNUSED(x) (0 && (x))

// Implicitly enable fast assertions along with normal assertions
#ifdef SOFA_ASSERTION_ENABLED
#   ifndef SOFA_FAST_ASSERTION_ENABLED
#       define SOFA_FAST_ASSERTION_ENABLED
#   endif
#endif

#ifdef SOFA_FAST_ASSERTION_ENABLED

#define SOFA_ASSERT_FAST(condition) \
    do \
    { \
        if (!(condition)) \
        { \
            if (sofa::assertion::onAssertionFailure(#condition, __FILE__, __LINE__, NULL) == sofa::assertion::HALT) \
            { \
                DEBUG_BREAK(); \
            } \
        } \
    } while(sofa::assertion::detail::getFalse())

// Note: in this macro, '##' is required before __VA_ARGS__ on some compilers (i.e. gcc) for the comma before __VA_ARGS__ to be eliminated when it is empty
#define SOFA_ASSERT_FAST_MSG(condition, ...) \
    do \
    { \
        if (!(condition)) \
        { \
            if (sofa::assertion::onAssertionFailureFormatted(#condition, __FILE__, __LINE__, ##__VA_ARGS__) == sofa::assertion::HALT) \
            { \
                DEBUG_BREAK(); \
            } \
        } \
    } while(sofa::assertion::detail::getFalse())

#else

#define SOFA_ASSERT_FAST(condition)             do { SOFA_ASSERT_UNUSED(condition); } while(sofa::assertion::detail::getFalse())
#define SOFA_ASSERT_FAST_MSG(condition, ...)    do { SOFA_ASSERT_UNUSED(condition); } while(sofa::assertion::detail::getFalse())

#endif

// Macro to be used to declare assertions with negligible impact on performance
#ifdef SOFA_ASSERTION_ENABLED

#define SOFA_ASSERT(condition)             SOFA_ASSERT_FAST(condition)
#define SOFA_ASSERT_MSG(condition, ...)    SOFA_ASSERT_FAST_MSG(condition, ##__VA_ARGS__)

#else

#define SOFA_ASSERT(condition)                  do { SOFA_ASSERT_UNUSED(condition); } while(sofa::assertion::detail::getFalse())
#define SOFA_ASSERT_MSG(condition, ...)    do { SOFA_ASSERT_UNUSED(condition); } while(sofa::assertion::detail::getFalse())

#endif

#if _MSC_VER
#pragma warning( pop )
#endif

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

namespace sofa
{
namespace assertion
{

namespace detail
{

/**
This method is used instead of value 0 in the do-while statements to prevent warning C4127 ("conditional expression is constant").
MSVC optimizes it out in release mode so it should have no impact on performance.
*/
inline bool getFalse() { return false; }

/// A method that does nothing but helps avoid warnings when a variable is declared but used only in an assertion.
template<typename T>
void unused(T)
{}

} // namespace detail

enum Action { HALT, CONTINUE };

SOFA_HELPER_API Action onAssertionFailure(const char* condition, const char* file, unsigned int line, const char* msg);
SOFA_HELPER_API Action onAssertionFailureFormatted(const char* condition, const char* file, unsigned int line, const char* msg, ...);

typedef Action (*FnAssertionHandler)(const char* condition, const char* file, unsigned int line, const char* msg);
SOFA_HELPER_API void setHandler(FnAssertionHandler handler);

/// An assertion handler that does not display an error message
SOFA_HELPER_API Action silentHalt(const char* /*condition*/, const char* /*file*/, unsigned int /*line*/, const char* /*msg*/);

} // namespace assertion
} // namespace sofa

#endif
