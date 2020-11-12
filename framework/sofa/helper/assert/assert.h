/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

/**
Assertions are meant to detect programming errors by specifying conditions that are always supposed to be true.
Their sole purpose is to alert the programmer when the application is in a invalid state,
they must not be used to deal with user related errors, such as clicking the wrong button.

Two categories of assertions macros are declared here : normal and fast.
Both have the same behavior, but either one or the other is the most adequate depending on the context:

- Fast assertions should be used to check statements that don't have any impact on performance,
  so that they can be activated even in release mode if desired.
  The corresponding macros are ISASSERT_FAST and ISASSERT_FAST_MSG.
  They can be enabled by defining ISTK_FAST_ASSERTION_ENABLED.

- Normal assertions should be used in all other cases and are usually activated in debug mode only.
  The corresponding macros are ISASSERT and ISASSERT_MSG.
  They can be enabled by defining ISTK_ASSERTION_ENABLED.

NOTE: Fast assertions are implicitly enabled along with normal assertions.

In case a condition is false, a handler callback is invoked.
A handler is provided by default that prints the assertion condition and a possible message,
but another can be set as such:

    issystem::assertion::setHandler(&myHandler);

The handler must be a function with the following signature:

    Action myHandler(const char* condition, const char* file, unsigned int line, const char* msg);

A handler can return:
- HALT to break execution of the application
- CONTINUE to move along after the assertion

Usage:
    ISASSERT(condition);
    ISASSERT_MSG(condition, message);
    ISASSERT_MSG(condition, message, argument1, argument2, ..., argument_n);

Examples:
    ISASSERT(i >= 0 && i < container.size());
    ISASSERT_MSG(i >= 0 && i < container.size(), "Index out of bound");
    ISASSERT_MSG(i >= 0 && i < container.size(), "Index out of bound: index is %u but size is %u", i, container.size());

Thanks to Noel Llopis and Charles Nicholson of Power of Two Games: http://cnicholson.net/2009/02/stupid-c-tricks-adventures-in-assert/.
*/

#ifndef ISSYSTEM_ISASSERT_H
#define ISSYSTEM_ISASSERT_H

#include "initPlugin.h"

#if defined(__GNUC__)
#   include "Platform/GCC/ISAssertPlatform.h"
#elif defined(_WIN32)
#   include "Platform/MSVC/ISAssertPlatform.h"
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

#define ISTK_UNUSED(x) (0 && (x))

// Implicitly enable fast assertions along with normal assertions
#ifdef ISTK_ASSERTION_ENABLED
#   ifndef ISTK_FAST_ASSERTION_ENABLED
#       define ISTK_FAST_ASSERTION_ENABLED
#   endif
#endif

#ifdef ISTK_FAST_ASSERTION_ENABLED

#define ISASSERT_FAST(condition) \
    do \
    { \
        if (!(condition)) \
        { \
            if (issystem::assertion::onAssertionFailure(#condition, __FILE__, __LINE__, NULL) == issystem::assertion::HALT) \
            { \
                DEBUG_BREAK(); \
            } \
        } \
    } while(issystem::assertion::detail::getFalse())

// Note: in this macro, '##' is required before __VA_ARGS__ on some compilers (i.e. gcc) for the comma before __VA_ARGS__ to be eliminated when it is empty
#define ISASSERT_FAST_MSG(condition, ...) \
    do \
    { \
        if (!(condition)) \
        { \
            if (issystem::assertion::onAssertionFailureFormatted(#condition, __FILE__, __LINE__, ##__VA_ARGS__) == issystem::assertion::HALT) \
            { \
                DEBUG_BREAK(); \
            } \
        } \
    } while(issystem::assertion::detail::getFalse())

#else

#define ISASSERT_FAST(condition)             do { ISTK_UNUSED(condition); } while(issystem::assertion::detail::getFalse())
#define ISASSERT_FAST_MSG(condition, ...)    do { ISTK_UNUSED(condition); } while(issystem::assertion::detail::getFalse())

#endif

// Macro to be used to declare assertions with negligible impact on performance
#ifdef ISTK_ASSERTION_ENABLED

#define ISASSERT(condition)             ISASSERT_FAST(condition)
#define ISASSERT_MSG(condition, ...)    ISASSERT_FAST_MSG(condition, ##__VA_ARGS__)

#else

#define ISASSERT(condition)                  do { ISTK_UNUSED(condition); } while(issystem::assertion::detail::getFalse())
#define ISASSERT_MSG(condition, ...)    do { ISTK_UNUSED(condition); } while(issystem::assertion::detail::getFalse())

#endif

#if _MSC_VER
#pragma warning( pop )
#endif

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

namespace issystem
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

SOFA_ISSYSTEM_API Action onAssertionFailure(const char* condition, const char* file, unsigned int line, const char* msg);
SOFA_ISSYSTEM_API Action onAssertionFailureFormatted(const char* condition, const char* file, unsigned int line, const char* msg, ...);

typedef Action (*FnAssertionHandler)(const char* condition, const char* file, unsigned int line, const char* msg);
SOFA_ISSYSTEM_API void setHandler(FnAssertionHandler handler);

/// An assertion handler that does not display an error message
SOFA_ISSYSTEM_API Action silentHalt(const char* /*condition*/, const char* /*file*/, unsigned int /*line*/, const char* /*msg*/);

} // namespace assertion
} // namespace issystem

#endif
