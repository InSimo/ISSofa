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
#ifndef SOFA_HELPER_SYSTEM_CONFIG_H
#define SOFA_HELPER_SYSTEM_CONFIG_H

#include <sofa/SofaFramework.h>
#include <sofa/helper/static_assert.h>

// to define NULL
#include <cstring>

#ifdef WIN32
#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif
#include <windows.h>
#endif

#ifdef BOOST_NO_EXCEPTIONS
#include<exception>

namespace boost
{
	inline void throw_exception(std::exception const & e)
	{
		return;
	}
}
#endif

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
# define _USE_MATH_DEFINES 1 // required to get M_PI from math.h
#endif
// Visual C++ does not include stdint.h
typedef signed __int8		int8_t;
typedef signed __int16		int16_t;
typedef signed __int32		int32_t;
typedef signed __int64		int64_t;
typedef unsigned __int8		uint8_t;
typedef unsigned __int16	uint16_t;
typedef unsigned __int32	uint32_t;
typedef unsigned __int64	uint64_t;
#else
#include <stdint.h>
#endif

#ifdef SOFA_FLOAT
typedef float SReal;
#else
typedef double SReal;
#endif

#define sofa_do_concat2(a,b) a ## b
#define sofa_do_concat(a,b) sofa_do_concat2(a,b)
#define sofa_concat(a,b) sofa_do_concat(a,b)

#define sofa_tostring(a) sofa_do_tostring(a)
#define sofa_do_tostring(a) #a

#define SOFA_DECL_CLASS(name) extern "C" { int sofa_concat(class_,name) = 0; }
#define SOFA_LINK_CLASS(name) extern "C" { extern int sofa_concat(class_,name); int sofa_concat(link_,name) = sofa_concat(class_,name); }

#if !defined(MAKEFOURCC)
    #define MAKEFOURCC(ch0, ch1, ch2, ch3) \
        (uint(uint8_t(ch0)) | (uint(uint8_t(ch1)) << 8) | \
        (uint(uint8_t(ch2)) << 16) | (uint(uint8_t(ch3)) << 24 ))
#endif

// Prevent compiler warnings about 'unused variables'.
// This should be used when a parameter name is needed (e.g. for
// documentation purposes) even if it is not used in the code.
#define SOFA_UNUSED(x) (void)(x)


// utility for debug tracing
#ifdef _MSC_VER
    #define SOFA_CLASS_METHOD ( std::string(this->getClassName()) + "::" + __FUNCTION__ + " " )
#else
    #define SOFA_CLASS_METHOD ( std::string(this->getClassName()) + "::" + __func__ + " " )
#endif

#ifdef __cpp_if_constexpr
#define SOFA_IF_CONSTEXPR if constexpr
#else
#define SOFA_IF_CONSTEXPR if
#endif

/// macros to locally disable warnings
#if defined(__clang__) || defined(__GNUC__)
    #define SOFA_PRAGMA(x) _Pragma(sofa_tostring(x))
    #if defined(__clang__)
        #define SOFA_WARNING_PUSH() SOFA_PRAGMA(clang diagnostic push)
        #define SOFA_WARNING_POP() SOFA_PRAGMA(clang diagnostic pop)
        #define SOFA_WARNING_DISABLE_CLANG(x) SOFA_PRAGMA(clang diagnostic ignored sofa_tostring(sofa_concat(-W,x)))
        #define SOFA_WARNING_DISABLE_GCC(x)
        #define SOFA_WARNING_DISABLE_MSC(x)
    #else
        #define SOFA_WARNING_PUSH() SOFA_PRAGMA(GCC diagnostic push)
        #define SOFA_WARNING_POP() SOFA_PRAGMA(GCC diagnostic pop)
        #define SOFA_WARNING_DISABLE_CLANG(x)
        #define SOFA_WARNING_DISABLE_GCC(x) SOFA_PRAGMA(GCC diagnostic ignored sofa_tostring(sofa_concat(-W,x)))
        #define SOFA_WARNING_DISABLE_MSC(x)
    #endif
#elif defined(_MSC_VER)
    #define SOFA_PRAGMA(x) __pragma(x)
    #define SOFA_WARNING_PUSH() SOFA_PRAGMA(warning(push))
    #define SOFA_WARNING_POP() SOFA_PRAGMA(warning(pop))
    #define SOFA_WARNING_DISABLE_CLANG(x)
    #define SOFA_WARNING_DISABLE_GCC(x)
    #define SOFA_WARNING_DISABLE_MSC(x) SOFA_PRAGMA(warning(disable: x))
#else
    #define SOFA_WARNING_PUSH()
    #define SOFA_WARNING_POP()
    #define SOFA_WARNING_DISABLE_CLANG(x)
    #define SOFA_WARNING_DISABLE_GCC(x)
    #define SOFA_WARNING_DISABLE_MSC(x)
#endif

// Prevent the compiler from inlining a function
#ifdef _MSC_VER
    #define SOFA_NO_INLINE __declspec((noinline)
#else
    #define SOFA_NO_INLINE __attribute__((noinline))
#endif

#endif
