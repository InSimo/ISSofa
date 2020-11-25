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
#ifndef SOFA_CORE_COLLISIONMODELTRAITS_H
#define SOFA_CORE_COLLISIONMODELTRAITS_H

#include <sofa/core/topology/Topology.h>

namespace sofa
{
namespace core
{
namespace collision
{

struct collision_void_topology_tag {};
struct collision_point_topology_tag    : public collision_void_topology_tag {};
struct collision_line_topology_tag     : public collision_point_topology_tag {};
struct collision_triangle_topology_tag : public collision_line_topology_tag {};
struct collision_quad_topology_tag     : public collision_line_topology_tag {};
struct collision_hexa_topology_tag     : public collision_triangle_topology_tag, collision_quad_topology_tag {};

template < typename TopologyInfo >
struct CollisionTopologyTagTraits
{
    typedef collision_void_topology_tag collision_topology_category;
};

template<>
struct CollisionTopologyTagTraits< sofa::core::topology::Topology::Point >
{
    typedef collision_point_topology_tag collision_topology_category;
};

template<>
struct CollisionTopologyTagTraits< sofa::core::topology::Topology::Edge >
{
    typedef collision_line_topology_tag collision_topology_category;
};

template<>
struct CollisionTopologyTagTraits< sofa::core::topology::Topology::Triangle >
{
    typedef collision_triangle_topology_tag collision_topology_category;
};

template<>
struct CollisionTopologyTagTraits< sofa::core::topology::Topology::Quad >
{
    typedef collision_quad_topology_tag collision_topology_category;
};

template<>
struct CollisionTopologyTagTraits< sofa::core::topology::Topology::Hexahedron >
{
    typedef collision_hexa_topology_tag collision_topology_category;
};


/// Trait class to have "flat" TriangleModel and LineModel data structures compatible with 
/// the one that are used for bezier collision model.
template< class TCollisionModel > 
struct CollisionModelTraits
{
    enum { NBARY = TCollisionModel::NBARY };
    typedef typename TCollisionModel::DataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real  Real;
    typedef sofa::defaulttype::Vec<NBARY,Real>    BaryCoord;
    typedef sofa::core::topology::Topology::Point TopologyElementType;
    typedef typename CollisionTopologyTagTraits<TopologyElementType>::collision_topology_category collision_topology_category;
};


///
/// Interpolation methods wrappers
///

template< class QBCollisionModel >
struct InterpX
{
    typedef typename CollisionModelTraits<QBCollisionModel>::Coord     Coord;
    typedef typename CollisionModelTraits<QBCollisionModel>::BaryCoord BaryCoord;
    typedef typename QBCollisionModel::Element Element;
    
    static Coord eval(const Element& e, const BaryCoord& baryCoords)
    {
        return e.shape().interpX(baryCoords);
    }

};


template< class QBCollisionModel >
struct InterpV
{
    typedef typename CollisionModelTraits<QBCollisionModel>::Deriv     Deriv;
    typedef typename CollisionModelTraits<QBCollisionModel>::BaryCoord BaryCoord;
    typedef typename QBCollisionModel::Element Element;
    
    static Deriv eval( const Element& e, const BaryCoord& baryCoords)
    {
        return e.shape().interpD(baryCoords, e.shape().cpV() ); 
    }
};

///
/// PointModel
///
template < class TDataTypes>
struct CollisionPointModelTraits
{
    enum { NBARY = 0 };
    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real  Real;
    typedef std::array<Real, NBARY>  BaryCoord;
    typedef sofa::core::topology::Topology::Point TopologyElementType;
    typedef typename CollisionTopologyTagTraits<TopologyElementType>::collision_topology_category collision_topology_category;
};





///
/// LineModel
///

template <class TDataTypes>
struct CollisionLineModelTraits
{
    enum { NBARY = 1 };
    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real  Real;
    typedef sofa::defaulttype::Vec<NBARY,Real> BaryCoord;
    typedef sofa::core::topology::Topology::Edge         TopologyElementType;
    typedef typename CollisionTopologyTagTraits<TopologyElementType>::collision_topology_category collision_topology_category;
};



///
/// TriangleModel
///

template <class TDataTypes> 
struct CollisionTriangleModelTraits
{
    enum { NBARY = 2 };
    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real  Real;
    typedef sofa::defaulttype::Vec<NBARY, Real> BaryCoord;
    typedef sofa::core::topology::Topology::Triangle  TopologyElementType;
    typedef typename CollisionTopologyTagTraits<TopologyElementType>::collision_topology_category collision_topology_category;
};




/// Distance Grid 
template< class TDataTypes >
struct CollisionDistanceGridTraits
{
    enum { NBARY = 3 };
    typedef TDataTypes DataTypes;
    typedef typename   DataTypes::Coord Coord;
    typedef typename   DataTypes::Deriv Deriv;
    typedef typename   DataTypes::Real  Real;
    typedef sofa::defaulttype::Vec<NBARY, Real> BaryCoord;
    typedef sofa::core::topology::Topology::Hexa  TopologyElementType;
    typedef typename CollisionTopologyTagTraits<TopologyElementType>::collision_topology_category collision_topology_category;
};




} // namespace collision
} // namespace core
} // namespace sofa

#endif //SOFA_CORE_COLLISIONMODELTRAITS_H
