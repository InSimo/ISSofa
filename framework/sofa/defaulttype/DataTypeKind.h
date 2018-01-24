/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_DEFAULTTYPE_DATATYPEKIND_H
#define SOFA_DEFAULTTYPE_DATATYPEKIND_H

namespace sofa
{

namespace defaulttype
{

/** The different types exposed by DataTypeInfo.
 */
enum class ContainerKindEnum
{
    Single = 0,
    Array,       ///< An ordered list of arbitrary values, all having the same type (vector, array)
    Set,         ///< An unordered list of unique keys, all having the same type (set, unordered_set)
    Map          ///< An unordered list of key/value pairs, all having the same type (map, unordered_map)
};

/** The different types for DTK_VALUE
 */
enum class ValueKindEnum
{
    Void = 0,
    Integer,
    Scalar,
    String,
    Enum,
    Bool,
    Pointer,
};

} // namespace defaulttype

} // namespace sofa

#endif  // SOFA_DEFAULTTYPE_DATATYPEKIND_H
