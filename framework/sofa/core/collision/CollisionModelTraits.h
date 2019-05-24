/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#ifndef SOFA_CORE_COLLISIONMODELTRAITS_H
#define SOFA_CORE_COLLISIONMODELTRAITS_H

#include <sofa/core/topology/TopologyHandler.h>

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
    typedef sofa::defaulttype::Vec<NBARY,Real>  BaryCoord;
    typedef sofa::core::topology::Point         TopologyElementType;
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
    typedef typename CollisionModelTraits<QBCollisionModel>::Coord     Coord;
    typedef typename CollisionModelTraits<QBCollisionModel>::BaryCoord BaryCoord;
    typedef typename QBCollisionModel::Element Element;
    
    static Coord eval( const Element& e, const BaryCoord& baryCoords)
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
    typedef sofa::defaulttype::Vec<1,Real>  BaryCoord;
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
