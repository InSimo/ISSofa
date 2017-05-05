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

///////////////////////////////////////////////////////////////////////////////
// operator new used by EASTL
// These are minimal implementations that just redirect calls
// to standard operator new
///////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <new>

void* operator new[](size_t size, const char* /*name*/, int /*flags*/,
					 unsigned /*debugFlags*/, const char* /*file*/, int /*line*/)
{
    return ::operator new[](size);
}

namespace
{

// A union that makes it easy to switch between memory pointer and address
union Memory
{
    void*       ptr;
    uintptr_t   address;
};

} // namespace

void* operator new[](size_t size, size_t alignment, size_t alignmentOffset, const char* /*name*/,
					 int flags, unsigned /*debugFlags*/, const char* /*file*/, int /*line*/)
{
    // Allocate memory
    const Memory mem = { ::operator new[](size) };

    // Return the aligned pointer
    const uintptr_t misalignment = mem.address & (alignment - 1);
    return (static_cast<uint8_t*>(mem.ptr) + (alignment - misalignment));
}
