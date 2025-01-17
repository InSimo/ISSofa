/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/helper/test.h>
#include <sofa/helper/BackTrace.h>

namespace sofa
{

namespace helper
{

// Should be called before any tests using Sofa is executed.
// This setup the environment for optimal testing (logger, backtrace)
SOFA_HELPER_API void initBeforeTests()
{
    // Some tests are designed to crash the program
    // Therefore, disable stopping the current process when this happens to prevent the tests from hanging indefinitely
    sofa::helper::BackTrace::autodump(false);
}

// Should be called after any tests using Sofa is executed.
SOFA_HELPER_API void cleanupAfterTests()
{
}

} // namespace helper

} // namespace sofa
