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
#ifndef SOFA_CORE_COLLISION_DETECTIONOUTPUT_H
#define SOFA_CORE_COLLISION_DETECTIONOUTPUT_H

#include <sofa/SofaFramework.h>
#include <sofa/helper/system/config.h>
#include <sofa/core/CollisionElement.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/vector.h>
#include <sofa/helper/integer_id.h>
#include <iostream>

namespace sofa
{

namespace core
{

namespace collision
{

// uncomment if you want to use the freePoint information
// #define DETECTIONOUTPUT_FREEMOTION
//#define DETECTIONOUTPUT_BARYCENTRICINFO

/**
 *  \brief Generic description of a contact point, used for most collision models except special cases such as GPU-based collisions.
 *
 *  Each contact point is described by :
 *
 *  - elem: pair of colliding elements.
 *  - id: unique id of the contact for the given pair of collision models.
 *  - point: contact points on the surface of each model.
 *  - normal: normal of the contact, pointing outward from the first model.
 *  - value: signed distance (negative if objects are interpenetrating).
 *  - deltaT: estimated of time of contact.
 *
 *  The contact id is used to filter redundant contacts (only the contact with
 *  the smallest distance is kept), and to store persistant data over time for
 *  the response.
 *
 */

class SOFA_CORE_API DetectionOutput
{
public:
    typedef sofa::defaulttype::Vector3 Vector3;
    /// Pair of colliding elements.
    std::pair<core::CollisionElementIterator, core::CollisionElementIterator> elem;
    typedef int64_t ContactId;
    /// Unique id of the contact for the given pair of collision models.
    ContactId id;
    /// Contact points on the surface of each model. They are expressed in the local coordinate system of the model if any is defined..
    Vector3 point[2];
#ifdef DETECTIONOUTPUT_FREEMOTION
    Vector3 freePoint[2]; ///< free Point in contact on each element
#endif
#ifdef DETECTIONOUTPUT_BARYCENTRICINFO
    Vector3 baryCoords[2]; ///< provides the barycentric Coordinates (alpha, beta, gamma) of each contact points over the element
    ///< (alpha is used for lines / alpha and beta for triangles / alpha, beta and gamma for tetrahedrons)
#endif

    /// Normal of the contact, pointing outward from the first model
    Vector3 normal;
    /*
    /// Signed distance (negative if objects are interpenetrating). If using a proximity-based detection, this is the actual distance between the objets minus the specified contact distance.
    */
    /// Store information for the collision Response. Depending on the kind of contact, can be a distance, or a pression, ...
    double value;
    /// If using a continuous collision detection, estimated of time of contact.
    double deltaT;
    DetectionOutput()
        : elem( (sofa::core::CollisionModel* )NULL,
                (sofa::core::CollisionModel* ) NULL), id(0), value(0.0), deltaT(0.0)
    {
    }
};

/**
 *  \brief Abstract description of a set of contact point.
 */
class SOFA_CORE_API DetectionOutputContainer
{
protected:
    virtual ~DetectionOutputContainer() {}
public:
    /// Clear the contents of this container
    virtual void clear() = 0;
    /// Current size (number of detected contacts)
    virtual unsigned int size() const = 0;
    /// Test if the container is empty
    bool empty() const { return size()==0; }
    /// Delete this container from memory once the contact pair is no longer active
    virtual void release() { delete this; }
    /// Swap the contents of both containers
    virtual void swap(DetectionOutputContainer& other) = 0;
    /// Copy one contact to a DetectionOutput (inefficient,
    /// not supported by all subclasses, use only for debugging)
    /// @return false if not supported
    virtual bool getDetectionOutput(unsigned int /*i*/, DetectionOutput& /*o*/) const { return false; }
    /// Invalidate the last DetectionOutput added to the container
    virtual void invalidateLastDetectionOutput() = 0;
};

/**
 *  \brief Generic description of a set of contact point between two specific collision models
 */
template<class CM1, class CM2>
class TDetectionOutputContainer : public DetectionOutputContainer, protected sofa::helper::vector<DetectionOutput>
{
protected:
    using Vector = sofa::helper::vector<DetectionOutput>;
public:
    using value_type = Vector::value_type;
    using iterator = Vector::iterator;
    using const_iterator = Vector::const_iterator;
    constexpr static const char* DetectionOutputIDName() { return "DetectionOutputID"; }
    using DetectionOutputID = sofa::helper::integer_id<DetectionOutputIDName>;

    virtual ~TDetectionOutputContainer() {}
    /// Clear the contents of this container
    virtual void clear() override
    {
        return Vector::clear();
    }
    /// Current size (number of detected contacts)
    virtual unsigned int size() const override
    {
        return (unsigned int)Vector::size();
    }
    /// Swap the contents of both containers
    virtual void swap(DetectionOutputContainer& other) override
    {
        TDetectionOutputContainer<CM1, CM2>* otherAsVector = dynamic_cast<TDetectionOutputContainer<CM1, CM2>*>(&other);
        assert(otherAsVector != nullptr);
        Vector::swap(*otherAsVector);
    }
    /// Copy one contact to a DetectionOutput (inefficient,
    /// not supported by all subclasses, use only for debugging)
    /// @return false if not supported
    virtual bool getDetectionOutput(unsigned int i, DetectionOutput& o) const override
    {
        o = Vector::operator[](i);
        return true;
    }
    /// Invalidate the last DetectionOutput added to the container
    virtual void invalidateLastDetectionOutput() override
    {
        // Remove the DO from the vector rather than marking it invalid since the operation is cheap
        Vector::pop_back();
    }

    virtual iterator begin()
    {
        return Vector::begin();
    }
    virtual iterator end()
    {
        return Vector::end();
    }
    virtual const_iterator begin() const
    {
        return Vector::begin();
    }
    virtual const_iterator end() const
    {
        return Vector::end();
    }
    virtual const_iterator cbegin() const
    {
        return Vector::cbegin();
    }
    virtual const_iterator cend() const
    {
        return Vector::cend();
    }
    virtual DetectionOutput& operator[](DetectionOutputID id)
    {
        return Vector::operator[](id.getId());
    }
    virtual const DetectionOutput& operator[](DetectionOutputID id) const
    {
        return Vector::operator[](id.getId());
    }
    virtual DetectionOutput& addDetectionOutput()
    {
        Vector::emplace_back(); // TODO return here in C++17
        return Vector::back();
    }
};

} // namespace collision

} // namespace core

} // namespace sofa

#endif
