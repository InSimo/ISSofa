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
#ifndef SOFA_HELPER_SYSTEM_LOCK_GUARD_H
#define SOFA_HELPER_SYSTEM_LOCK_GUARD_H

#ifdef SOFA_HAVE_BOOST_THREAD
#include <boost/thread/lock_guard.hpp> 
#else
#include <mutex>
#endif

namespace sofa
{
namespace helper
{
// NB: sofa::helper::system::thread is already used as a namespace name  
// for consistency import everything related to threading in sofa::helper
#ifdef SOFA_HAVE_BOOST_THREAD
template<class T>
using lock_guard = boost::lock_guard<T>;
#else
template<class T>
using lock_guard = std::lock_guard<T>;
#endif

}

}

#endif // SOFA_HELPER_SYSTEM_LOCK_GUARD_H
