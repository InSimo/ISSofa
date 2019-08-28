/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_CORE_COLLISION_BASECOLLISIONELEMENTSHAPE_H
#define SOFA_CORE_COLLISION_BASECOLLISIONELEMENTSHAPE_H

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/fixed_array.h>

namespace sofa
{

namespace core
{

namespace collision
{

/**
 *  \brief Collision element shape interface.
 *
 *  Collision element shapes contain functions used to interpolate state vectors onto the collision element
 *  using barycentric coordinates.
 */
class SOFA_CORE_API BaseCollisionElementShape
{
public:
    using Vec3 = sofa::defaulttype::Vector3;
    using BaryCoords = Vec3; // 3 is enough because we simulate a three-dimensional space

    /// \brief Interpolate the position on the shape at the given barycentric coordinates
    virtual Vec3 interpolateX(const BaryCoords& bary) const = 0;
    /// \brief Interpolate the velocity on the shape at the given barycentric coordinates
    virtual Vec3 interpolateV(const BaryCoords& bary) const = 0;

    /// \brief Compute the gradient on the shape at the given barycentric coordinates
    /// \param bary the barycentric coordinates at which to compute the gradient
    virtual sofa::helper::fixed_array<Vec3, 3> gradient(const BaryCoords& bary) const { return {gradient0(bary), gradient1(bary), gradient2(bary)}; }
    /// \brief Compute the first component of the gradient on the shape at the given barycentric coordinates
    /// \param bary the barycentric coordinates at which to compute the gradient
    ///  This vector corresponds to the partial derivative with respect to the first barycentric coordinate
    virtual Vec3 gradient0(const BaryCoords& bary) const = 0;
    /// \brief Compute the second component of the gradient on the shape at the given barycentric coordinates
    /// \param bary the barycentric coordinates at which to compute the gradient
    ///  This vector corresponds to the partial derivative with respect to the second barycentric coordinate
    virtual Vec3 gradient1(const BaryCoords& bary) const = 0;
    /// \brief Compute the third component of the gradient on the shape at the given barycentric coordinates
    /// \param bary the barycentric coordinates at which to compute the gradient
    ///  This vector corresponds to the partial derivative with respect to the third barycentric coordinate
    virtual Vec3 gradient2(const BaryCoords& bary) const = 0;

    /// \brief Returns true iff normals are defined on the shape
    virtual bool hasNormal() const { return false; }
    /// \brief Interpolate the normal on the shape at the given barycentric coordinates
    virtual Vec3 normal(const BaryCoords& /*bary*/) const { return {}; }

    /// \brief Returns true iff tangents are defined on the shape
    virtual bool hasTangent() const { return false; }
    /// \brief Interpolate the tangent on the shape at the given barycentric coordinates
    virtual Vec3 tangent(const BaryCoords& /*bary*/) const { return {}; }
protected:
    virtual ~BaseCollisionElementShape() = default;
};

} // namespace collision

} // namespace core

} // namespace sofa

#endif

