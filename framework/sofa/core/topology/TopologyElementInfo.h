/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#ifndef SOFA_CORE_TOPOLOGY_TOPOLOGYELEMENTINFO_H
#define SOFA_CORE_TOPOLOGY_TOPOLOGYELEMENTINFO_H

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/core/collision/CollisionModelTraits.h>

namespace sofa
{
namespace core
{
namespace topology
{

/**
 * @class TopologyElementInfo
 * @brief sofa::Datastructure which stores information about a topology. Since there is no type safety for
 * topology identifier ( PointID, EdgeID, TriangleID ), this data structure is an aggregate of a
 * TopologyObjectType ( Point, Edge, Triangle...) and an element index.
 */
struct SOFA_CORE_API TopologyElementInfo
{
    typedef sofa::core::topology::TopologyObjectType           TopologyObjectType;
    typedef sofa::core::topology::BaseMeshTopology::index_type ElemID;
    typedef sofa::core::topology::BaseMeshTopology::PointID    PointID;
    typedef sofa::core::topology::BaseMeshTopology::EdgeID     EdgeID;
    typedef sofa::core::topology::BaseMeshTopology::TriangleID TriangleID;
    enum{ InvalidID = sofa::core::topology::BaseMeshTopology::InvalidID };

    union Element
    {
        PointID            pid;
        EdgeID             eid;
        TriangleID         tid;
        ElemID             id;
        Element():id(InvalidID)
        {
        }
        explicit Element(ElemID i):id(i)
        {
        }
    };

    TopologyObjectType type;
    Element            element;

    TopologyElementInfo():type(sofa::core::topology::POINT)
    {
    }

    explicit TopologyElementInfo(TopologyObjectType t):type(t)
    {
    }

    TopologyElementInfo(TopologyObjectType t, ElemID i):type(t), element(i)
    {
    }

    static TopologyElementInfo Point(PointID i)       { return TopologyElementInfo(sofa::core::topology::POINT,    i); }
    static TopologyElementInfo Edge(EdgeID i)         { return TopologyElementInfo(sofa::core::topology::EDGE,     i); }
    static TopologyElementInfo Triangle(TriangleID i) { return TopologyElementInfo(sofa::core::topology::TRIANGLE, i); }
};

inline bool operator== ( const TopologyElementInfo& lhs, const TopologyElementInfo& rhs)
{
    return lhs.type==rhs.type && lhs.element.id==rhs.element.id;
}

inline bool operator!= ( const TopologyElementInfo& lhs, const TopologyElementInfo& rhs)
{
    return !(lhs==rhs);
}

inline bool operator< ( const TopologyElementInfo& lhs, const TopologyElementInfo& rhs)
{
    return lhs.type<rhs.type || (!(rhs.type<lhs.type) && lhs.element.id<rhs.element.id);
}

inline bool operator<= ( const TopologyElementInfo& lhs, const TopologyElementInfo& rhs)
{
    return !(rhs<lhs);
}

inline bool operator> ( const TopologyElementInfo& lhs, const TopologyElementInfo& rhs)
{
    return rhs<lhs;
}

inline bool operator>= ( const TopologyElementInfo& lhs, const TopologyElementInfo& rhs)
{
    return !(lhs<rhs);
}


std::string SOFA_CORE_API parseTopologyElementInfoToString( const TopologyElementInfo& info);

inline std::ostream& operator<< ( std::ostream& out, const TopologyElementInfo& o)
{
    out << parseTopologyElementInfoToString(o) << " END\n";
    return out;
}

namespace internal
{

template<class TDataTypes >
inline TopologyElementInfo _getTopologyElementInfo(sofa::core::CollisionModel* /*model*/, int /*index*/, const sofa::defaulttype::Vector3& /*baryCoord*/,
                                                   typename TDataTypes::Real /*epsilon*/, collision::collision_void_topology_tag )
{
    return TopologyElementInfo();
}

template<class TDataTypes >
inline TopologyElementInfo _getTopologyElementInfo(sofa::core::CollisionModel* /*model*/, int index, const sofa::defaulttype::Vector3& /*baryCoord*/,
                                                   typename TDataTypes::Real /*epsilon*/, collision::collision_point_topology_tag )
{
    TopologyElementInfo topologyInfo;
    topologyInfo.type         = sofa::core::topology::POINT;
    topologyInfo.element.pid  = sofa::core::topology::BaseMeshTopology::PointID(index);

    return topologyInfo;
}

template<class TDataTypes >
inline TopologyElementInfo _getTopologyElementInfo(sofa::core::CollisionModel*  model,  int index, const sofa::defaulttype::Vector3& baryCoord,
                                                    typename TDataTypes::Real epsilon, collision::collision_line_topology_tag )
{
    typedef typename TDataTypes::Real Real;
    sofa::core::topology::BaseMeshTopology* topo = model->getTopology();
    const sofa::core::topology::Topology::Edge& e = topo->getEdge(index);

    Real b = baryCoord[0];
    TopologyElementInfo topologyInfo;
    if( b < epsilon )
    {
        // point 0
        topologyInfo.type = sofa::core::topology::POINT;
        topologyInfo.element.pid  = e[0];
    }
    else if( b > (Real(1)-epsilon) )
    {
        // point 1
        topologyInfo.type = sofa::core::topology::POINT;
        topologyInfo.element.pid = e[1];
    }
    else
    {
        // inside edge
        topologyInfo.type         = sofa::core::topology::EDGE;
        topologyInfo.element.eid  = sofa::core::topology::BaseMeshTopology::EdgeID(index);
    }
    return topologyInfo;
}

template<class TDataTypes>
inline TopologyElementInfo _getTopologyElementInfo(sofa::core::CollisionModel* model,
    int index, const sofa::defaulttype::Vector3& baryCoord, typename TDataTypes::Real epsilon, collision::collision_triangle_topology_tag  )
{
    typedef typename TDataTypes::Real Real;
    sofa::core::topology::BaseMeshTopology* topo = model->getTopology();
    const sofa::core::topology::Topology::Triangle& t = topo->getTriangle(index);
    const sofa::core::topology::BaseMeshTopology::EdgesInTriangle& eit = topo->getEdgesInTriangle(index);
    Real b1 = baryCoord[0];
    Real b2 = baryCoord[1];
    Real b0 = Real(1.0)-(b1+b2);

    TopologyElementInfo topologyInfo;

    if (b0 > (Real(1.0) - epsilon) )
    { // point 0
        topologyInfo.type = sofa::core::topology::POINT;
        topologyInfo.element.pid = t[0];
    }
    else if (b1 > (Real(1.0) - epsilon)  )
    { // point 1
        topologyInfo.type = sofa::core::topology::POINT;
        topologyInfo.element.pid = t[1];
    }
    else if (b2 > (Real(1.0) - epsilon) )
    { // point 2
        topologyInfo.type = sofa::core::topology::POINT;
        topologyInfo.element.pid = t[2];
    }
    else if (b0 < epsilon )
    { // edge 0
        topologyInfo.type = sofa::core::topology::EDGE;
        topologyInfo.element.eid = eit[0];
    }
    else if (b1 < epsilon )
    { // edge 1
        topologyInfo.type = sofa::core::topology::EDGE;
        topologyInfo.element.eid = eit[1];
    }
    else if (b2 < epsilon )
    { // edge 2
        topologyInfo.type = sofa::core::topology::EDGE;
        topologyInfo.element.eid = eit[2];
    }
    else
    {
        // inside triangle
        topologyInfo.type        = sofa::core::topology::TRIANGLE;
        topologyInfo.element.tid = sofa::core::topology::BaseMeshTopology::TriangleID(index);
    }

    return topologyInfo;
}

} // namespace internal

template < class TCollisionModel >
inline TopologyElementInfo getTopologyElementInfo( TCollisionModel* model, int index, const sofa::defaulttype::Vector3& baryCoord,
                                                   typename collision::CollisionModelTraits<TCollisionModel>::Real epsilon = typename collision::CollisionModelTraits<TCollisionModel>::Real(1e-6) )
{
    typename collision::CollisionModelTraits<TCollisionModel>::collision_topology_category topology_category;
    return internal::_getTopologyElementInfo<typename collision::CollisionModelTraits<TCollisionModel>::DataTypes >( model, index, baryCoord, epsilon, topology_category);
}


} // namespace topology
} // namespace core
} // namespace sofa

#endif // SOFA_CORE_TOPOLOGY_TOPOLOGYELEMENTINFO_H
