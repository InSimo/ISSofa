/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 MGH, INRIA, USTL, UJF, CNRS, InSimo                *
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
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_MAPPING_BARYCENTRICMAPPING_H
#define SOFA_COMPONENT_MAPPING_BARYCENTRICMAPPING_H

#include <SofaBaseTopology/TopologyData.h>
#include <SofaBaseTopology/TopologyDataHandler.h>
#include <SofaBaseTopology/RegularGridTopology.h>
#include <sofa/defaulttype/CompressedRowSparseMatrixMechanical.h>

#ifdef SOFA_HAVE_EIGEN2
#include <SofaEigen2Solver/EigenSparseMatrix.h>
#endif

#include <sofa/core/Mapping.h>
#include <sofa/core/MechanicalParams.h>

#include <sofa/defaulttype/StructTypeInfo.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>

#include <sofa/helper/vector.h>
#include <sofa/helper/unordered_set.h>
#include <memory>

// forward declarations
namespace sofa
{
namespace core
{
namespace topology
{
class BaseMeshTopology;
}
}

namespace component
{
namespace topology
{
class MeshTopology;
class RegularGridTopology;
class SparseGridTopology;

class PointSetTopologyContainer;
template <class T>
class PointSetGeometryAlgorithms;

class EdgeSetTopologyContainer;
template <class T>
class EdgeSetGeometryAlgorithms;

class TriangleSetTopologyContainer;
template <class T>
class TriangleSetGeometryAlgorithms;

class QuadSetTopologyContainer;
template <class T>
class QuadSetGeometryAlgorithms;

class TetrahedronSetTopologyContainer;
template <class T>
class TetrahedronSetGeometryAlgorithms;

class HexahedronSetTopologyContainer;
template <class T>
class HexahedronSetGeometryAlgorithms;
}
}
}

namespace sofa
{

namespace component
{

namespace mapping
{

template<class Real, int NC,  int NP>
class BMMappingData;

/// Base class for barycentric mapping topology-specific mappers

template<class In, class Out>
class BarycentricMapper : public virtual core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapper,In,Out),core::objectmodel::BaseObject);

    typedef typename In::Real Real;
    typedef typename In::Real InReal;
    typedef typename Out::Real OutReal;

    typedef typename In::VecDeriv InVecDeriv;
    typedef typename In::Deriv InDeriv;

    typedef typename Out::VecDeriv OutVecDeriv;
    typedef typename Out::Deriv OutDeriv;

    enum { NIn = sofa::defaulttype::DataTypeInfo<InDeriv>::FinalSize };
    enum { NOut = sofa::defaulttype::DataTypeInfo<OutDeriv>::FinalSize };
    typedef defaulttype::Mat<NOut, NIn, Real> MBloc;

public:
    typedef BMMappingData<Real, 1,2> LineData;
    typedef BMMappingData<Real, 2,3> TriangleData;
    typedef BMMappingData<Real, 2,4> QuadData;
    typedef BMMappingData<Real, 3,4> TetraData;
    typedef BMMappingData<Real, 3,8> CubeData;
    typedef BMMappingData<Real, 1,0> MappingData1D;
    typedef BMMappingData<Real, 2,0> MappingData2D;
    typedef BMMappingData<Real, 3,0> MappingData3D;

protected:
    BarycentricMapper() {}
    virtual ~BarycentricMapper() {}
	
private:
	BarycentricMapper(const BarycentricMapper& n) ;
	BarycentricMapper& operator=(const BarycentricMapper& n) ;
	
public:
    virtual void init(const typename Out::VecCoord& out, const typename In::VecCoord& in) = 0;
    /// Called if the mapper should setup handling of topological changes
    virtual void initTopologyChange() {}
    virtual void apply( typename Out::VecCoord& out, const typename In::VecCoord& in ) = 0;
    virtual const sofa::defaulttype::BaseMatrix* getJ(int /*outSize*/, int /*inSize*/)
    {
        std::cerr << "BarycentricMapper::getJ() NOT IMPLEMENTED BY " << sofa::core::objectmodel::BaseClass::decodeClassName(typeid(*this)) << std::endl;
        return NULL;
    }
    virtual void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in ) = 0;
    virtual void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in ) = 0;
    virtual void applyJT( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in ) = 0;
    virtual void draw(const core::visual::VisualParams*, const typename Out::VecCoord& out, const typename In::VecCoord& in) = 0;

    //-- test mapping partiel
    virtual void applyOnePoint( const unsigned int& /*hexaId*/, typename Out::VecCoord& /*out*/, const typename In::VecCoord& /*in*/)
    {};
    //--


    virtual void clear( int reserve=0 ) =0;

    //Nothing to do
    inline friend std::istream& operator >> ( std::istream& in, BarycentricMapper< In, Out > & ) {return in;}
    inline friend std::ostream& operator << ( std::ostream& out, const BarycentricMapper< In, Out > &  ) { return out; }
};



/// Template class for barycentric mapping topology-specific mappers.
template<class In, class Out>
class TopologyBarycentricMapper : public BarycentricMapper<In,Out>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(TopologyBarycentricMapper,In,Out),SOFA_TEMPLATE2(BarycentricMapper,In,Out));

    typedef BarycentricMapper<In,Out> Inherit;
    typedef typename Inherit::Real Real;

protected:
    virtual ~TopologyBarycentricMapper() {}
public:

    virtual int addPointInLine(const int /*lineIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int setPointInLine(const int /*pointIndex*/, const int /*lineIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int createPointInLine(const typename Out::Coord& /*p*/, int /*lineIndex*/, const typename In::VecCoord* /*points*/) {return 0;}

    virtual int addPointInTriangle(const int /*triangleIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int setPointInTriangle(const int /*pointIndex*/, const int /*triangleIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int createPointInTriangle(const typename Out::Coord& /*p*/, int /*triangleIndex*/, const typename In::VecCoord* /*points*/) {return 0;}

    virtual int addPointInQuad(const int /*quadIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int setPointInQuad(const int /*pointIndex*/, const int /*quadIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int createPointInQuad(const typename Out::Coord& /*p*/, int /*quadIndex*/, const typename In::VecCoord* /*points*/) {return 0;}

    virtual int addPointInTetra(const int /*tetraIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int setPointInTetra(const int /*pointIndex*/, const int /*tetraIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int createPointInTetra(const typename Out::Coord& /*p*/, int /*tetraIndex*/, const typename In::VecCoord* /*points*/) {return 0;}

    virtual int addPointInCube(const int /*cubeIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int setPointInCube(const int /*pointIndex*/, const int /*cubeIndex*/, const SReal* /*baryCoords*/) {return 0;}
    virtual int createPointInCube(const typename Out::Coord& /*p*/, int /*cubeIndex*/, const typename In::VecCoord* /*points*/) {return 0;}

    virtual void setToTopology( topology::PointSetTopologyContainer* toTopology) {this->toTopology = toTopology;}

    virtual unsigned int getFromTopologyIndex(unsigned int /*toId*/) {std::cout<<"WARNING[TopologyBarycentricMapper] getFromTopologyIndex is called but not implemented"<<std::endl; return -1;}

    virtual defaulttype::Vector3 getFromTopologyBary(unsigned int /*toId*/) {std::cout<<"WARNING[TopologyBarycentricMapper] getFromTopologyBary is called but not implemented"<<std::endl; return {0, 0, 0};}

    const topology::PointSetTopologyContainer *getToTopology() const {return toTopology;}

    core::topology::BaseMeshTopology* getFromTopology() const { return fromTopology; }

protected:
    TopologyBarycentricMapper(core::topology::BaseMeshTopology* fromTopology, topology::PointSetTopologyContainer* toTopology = NULL)
        : fromTopology(fromTopology), toTopology(toTopology)
    {}

protected:
    core::topology::BaseMeshTopology* fromTopology;
    topology::PointSetTopologyContainer* toTopology;
};



/// Class allowing barycentric mapping computation on a MeshTopology
template<class In, class Out>
class BarycentricMapperMeshTopology : public TopologyBarycentricMapper<In,Out>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapperMeshTopology,In,Out),SOFA_TEMPLATE2(TopologyBarycentricMapper,In,Out));

    typedef TopologyBarycentricMapper<In,Out> Inherit;
    typedef typename Inherit::Real Real;
    typedef typename Inherit::OutReal OutReal;
    typedef typename Inherit::OutDeriv  OutDeriv;

    typedef typename Inherit::InDeriv  InDeriv;
    typedef typename Inherit::MappingData1D MappingData1D;
    typedef typename Inherit::MappingData2D MappingData2D;
    typedef typename Inherit::MappingData3D MappingData3D;

    enum { NIn = Inherit::NIn };
    enum { NOut = Inherit::NOut };
    typedef typename Inherit::MBloc MBloc;

protected:

    sofa::helper::vector< MappingData1D >  map1d;
    sofa::helper::vector< MappingData2D >  map2d;
    sofa::helper::vector< MappingData3D >  map3d;

    BarycentricMapperMeshTopology(core::topology::BaseMeshTopology* fromTopology,
            topology::PointSetTopologyContainer* toTopology)
        : TopologyBarycentricMapper<In,Out>(fromTopology, toTopology)
    {
    }

    virtual ~BarycentricMapperMeshTopology()
    {
    }
public:

    void clear(int reserve=0);

    int addPointInLine(const int lineIndex, const SReal* baryCoords);
    int createPointInLine(const typename Out::Coord& p, int lineIndex, const typename In::VecCoord* points);

    int addPointInTriangle(const int triangleIndex, const SReal* baryCoords);
    int createPointInTriangle(const typename Out::Coord& p, int triangleIndex, const typename In::VecCoord* points);

    int addPointInQuad(const int quadIndex, const SReal* baryCoords);
    int createPointInQuad(const typename Out::Coord& p, int quadIndex, const typename In::VecCoord* points);

    int addPointInTetra(const int tetraIndex, const SReal* baryCoords);

    int addPointInCube(const int cubeIndex, const SReal* baryCoords);

    void init(const typename Out::VecCoord& out, const typename In::VecCoord& in);

    void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );
    void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );
    void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );
    void applyJT( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in );
    void draw(const core::visual::VisualParams*,const typename Out::VecCoord& out, const typename In::VecCoord& in);

	sofa::helper::vector< MappingData3D > const* getMap3d() const { return &map3d; }

    virtual unsigned int getFromTopologyIndex(unsigned int toId) override;
    virtual defaulttype::Vector3 getFromTopologyBary(unsigned int toId) override;


    inline friend std::istream& operator >> ( std::istream& in, BarycentricMapperMeshTopology<In, Out> &b )
    {
        unsigned int size_vec;
        in >> size_vec;
        b.map1d.clear();
        MappingData1D value1d;
        for (unsigned int i=0; i<size_vec; i++)
        {
            in >> value1d;
            b.map1d.push_back(value1d);
        }

        in >> size_vec;
        b.map2d.clear();
        MappingData2D value2d;
        for (unsigned int i=0; i<size_vec; i++)
        {
            in >> value2d;
            b.map2d.push_back(value2d);
        }

        in >> size_vec;
        b.map3d.clear();
        MappingData3D value3d;
        for (unsigned int i=0; i<size_vec; i++)
        {
            in >> value3d;
            b.map3d.push_back(value3d);
        }
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const BarycentricMapperMeshTopology<In, Out> & b )
    {

        out << b.map1d.size();
        out << " " ;
        out << b.map1d;
        out << " " ;
        out << b.map2d.size();
        out << " " ;
        out << b.map2d;
        out << " " ;
        out << b.map3d.size();
        out << " " ;
        out << b.map3d;

        return out;
    }

private:
    void clear1d(int reserve=0);
    void clear2d(int reserve=0);
    void clear3d(int reserve=0);

};



/// Class allowing barycentric mapping computation on a RegularGridTopology
template<class In, class Out>
class BarycentricMapperRegularGridTopology : public TopologyBarycentricMapper<In,Out>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapperRegularGridTopology,In,Out),SOFA_TEMPLATE2(TopologyBarycentricMapper,In,Out));
    typedef TopologyBarycentricMapper<In,Out> Inherit;
    typedef typename Inherit::Real Real;
    typedef typename Inherit::OutReal OutReal;
    typedef typename Inherit::OutDeriv  OutDeriv;
    typedef typename Inherit::InDeriv  InDeriv;
    typedef typename Inherit::CubeData CubeData;

    enum { NIn = Inherit::NIn };
    enum { NOut = Inherit::NOut };
    typedef typename Inherit::MBloc MBloc;

protected:

    sofa::helper::vector<CubeData> map;
    topology::RegularGridTopology* fromTopology;
    
    BarycentricMapperRegularGridTopology(topology::RegularGridTopology* fromTopology,
            topology::PointSetTopologyContainer* toTopology)
        : Inherit(fromTopology, toTopology),fromTopology(fromTopology)
    {
    }

public:

    void clear(int reserve=0);

    bool isEmpty() {return this->map.size() == 0;}
    void setTopology(topology::RegularGridTopology* _topology) {this->fromTopology = _topology;}
    topology::RegularGridTopology *getTopology() {return topology::RegularGridTopology::DynamicCast(this->fromTopology);}

    int addPointInCube(const int cubeIndex, const SReal* baryCoords);

    void init(const typename Out::VecCoord& out, const typename In::VecCoord& in);
    void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );
    void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );
    void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );
    void applyJT( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in );
    void draw(const core::visual::VisualParams*,const typename Out::VecCoord& out, const typename In::VecCoord& in);

    inline friend std::istream& operator >> ( std::istream& in, BarycentricMapperRegularGridTopology<In, Out> &b )
    {
        in >> b.map;
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const BarycentricMapperRegularGridTopology<In, Out> & b )
    {
        out << b.map;
        return out;
    }

};



/// Class allowing barycentric mapping computation on a SparseGridTopology
template<class In, class Out>
class BarycentricMapperSparseGridTopology : public TopologyBarycentricMapper<In,Out>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapperSparseGridTopology,In,Out),SOFA_TEMPLATE2(TopologyBarycentricMapper,In,Out));
    typedef TopologyBarycentricMapper<In,Out> Inherit;
    typedef typename Inherit::Real Real;
    typedef typename Inherit::OutReal OutReal;
    typedef typename Inherit::OutDeriv  OutDeriv;
    typedef typename Inherit::InDeriv  InDeriv;

    typedef typename Inherit::CubeData CubeData;

    enum { NIn = Inherit::NIn };
    enum { NOut = Inherit::NOut };
    typedef typename Inherit::MBloc MBloc;

protected:

    sofa::helper::vector<CubeData> map;
    topology::SparseGridTopology* fromTopology;

    BarycentricMapperSparseGridTopology(topology::SparseGridTopology* fromTopology,
            topology::PointSetTopologyContainer* _toTopology)
        : TopologyBarycentricMapper<In,Out>(fromTopology, _toTopology),
          fromTopology(fromTopology)
    {
    }

public:

    void clear(int reserve=0);

    int addPointInCube(const int cubeIndex, const SReal* baryCoords);

    void init(const typename Out::VecCoord& out, const typename In::VecCoord& in);

    void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );
    void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );
    void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );
    void applyJT( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in );
    void draw(const core::visual::VisualParams*,const typename Out::VecCoord& out, const typename In::VecCoord& in);

    inline friend std::istream& operator >> ( std::istream& in, BarycentricMapperSparseGridTopology<In, Out> &b )
    {
        in >> b.map;
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const BarycentricMapperSparseGridTopology<In, Out> & b )
    {
        out << b.map;
        return out;
    }

};

template<class Real>
struct BMBaryElementInfo;

/// Class allowing barycentric mapping computation on a EdgeSetTopology
template<class In, class Out>
class BarycentricMapperEdgeSetTopology : public TopologyBarycentricMapper<In,Out>

{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapperEdgeSetTopology,In,Out),SOFA_TEMPLATE2(TopologyBarycentricMapper,In,Out));
    typedef TopologyBarycentricMapper<In,Out> Inherit;
    typedef typename Inherit::Real Real;
    typedef typename Inherit::OutReal OutReal;
    typedef typename Inherit::OutDeriv  OutDeriv;
    typedef typename Inherit::InDeriv  InDeriv;
    typedef typename Inherit::MappingData1D MappingData;
    typedef BMBaryElementInfo<Real> BaryElementInfo; // only used for vPointsIncluded, another structure could be used instead

    typedef topology::Topology::EdgeID EdgeID;
    typedef topology::Topology::Edge Edge;
    typedef topology::Topology::PointID PointID;

    enum { NIn = Inherit::NIn };
    enum { NOut = Inherit::NOut };
    typedef typename Inherit::MBloc MBloc;

    // topologyData mechanism to handle topology changes (public)
    typedef typename sofa::helper::vector<BaryElementInfo> VecBaryEdgeInfo;
    sofa::component::topology::EdgeData<VecBaryEdgeInfo> d_vBaryEdgeInfo;
    // END topologyData mechanism (public)

protected:
    topology::PointData< sofa::helper::vector<MappingData > > map;
    topology::EdgeSetTopologyContainer*         _fromContainer;
    topology::EdgeSetGeometryAlgorithms<In>*    _fromGeomAlgo;

    bool m_useRestPosition;
    core::State< In >* m_stateFrom = nullptr;
    core::State< Out >* m_stateTo  = nullptr;

    struct Jacobian
    {
        std::vector< std::array<InDeriv,2> > jacobianVector; /// only used in template VecNormal3Types
    };

    Jacobian   J;          ///< Jacobian of the mapping, in a vector

    // topologyData mechanism to handle topology changes (protected)
    class EdgeInfoHandler : public sofa::component::topology::TopologyDataHandler<Edge, VecBaryEdgeInfo>
    {
    public:
        typedef topology::Topology::Edge Edge;

        typedef typename sofa::component::topology::TopologyDataHandler<Edge, VecBaryEdgeInfo> TopologyDataHandler;
        EdgeInfoHandler(BarycentricMapperEdgeSetTopology* o, sofa::component::topology::EdgeData<VecBaryEdgeInfo>* d)
            : TopologyDataHandler(d), obj(o)
        {}

        virtual void applyCreateFunction(unsigned int e,
            BaryElementInfo& baryEdgeInfo,
            const Edge& edge,
            const sofa::helper::vector< unsigned int >& ancestors,
            const sofa::helper::vector< double >& coeffs) override;
        virtual void applyDestroyFunction(unsigned int e, BaryElementInfo& baryEdgeInfo) override;
        virtual void swap(unsigned int i1, unsigned int i2) override;

        protected:
        BarycentricMapperEdgeSetTopology* obj;
    };

    class MapPointInfoHandler : public sofa::component::topology::TopologyDataHandler<topology::Topology::Point, sofa::helper::vector<MappingData> >
    {
    public:
        typedef topology::Topology::Point Point;

        typedef typename sofa::component::topology::TopologyDataHandler<Point, sofa::helper::vector<MappingData>> TopologyDataHandler;
        MapPointInfoHandler(BarycentricMapperEdgeSetTopology* o, sofa::component::topology::PointData<sofa::helper::vector<MappingData>>* d)
            : TopologyDataHandler(d), obj(o)
        {}

        virtual void applyCreateFunction(unsigned int p, MappingData& poinInfo,
            const Point & point,
            const sofa::helper::vector< unsigned int >& ancestors,
            const sofa::helper::vector< double >& coeffs) override;
        virtual void applyDestroyFunction(unsigned int p, MappingData& pointInfo) override;
        virtual void swap(unsigned int i1, unsigned int i2) override;

    protected:
        BarycentricMapperEdgeSetTopology* obj;
    };

    std::unique_ptr<EdgeInfoHandler> m_edgeInfoHandler;
    std::unique_ptr<MapPointInfoHandler> m_vMapInfoHandler;
    std::set< PointID > m_dirtyPoints;
    // END topologyData mechanism (protected)

    BarycentricMapperEdgeSetTopology(topology::EdgeSetTopologyContainer* fromTopology, topology::PointSetTopologyContainer* _toTopology,
                                     core::State< In >* stateFrom = nullptr, core::State< Out >* stateTo = nullptr,
                                     bool useRestPosition = false)
        : TopologyBarycentricMapper<In,Out>(fromTopology, _toTopology),
          d_vBaryEdgeInfo(initData(&d_vBaryEdgeInfo, "vBaryEdgeInfo", "Vector of edge information dedicated to topological changes")),
          map(initData(&map,"map", "mapper data")),
          _fromContainer(fromTopology),
          _fromGeomAlgo(NULL),
          m_useRestPosition(useRestPosition),
          m_stateFrom(stateFrom),
          m_stateTo(stateTo)
    {
        m_edgeInfoHandler = std::unique_ptr<EdgeInfoHandler>(new EdgeInfoHandler(this, &d_vBaryEdgeInfo));
        m_vMapInfoHandler = std::unique_ptr<MapPointInfoHandler>(new MapPointInfoHandler(this, &map));
    }

public:

    void clear(int reserve=0);

    int addPointInLine(const int edgeIndex, const SReal* baryCoords);
    int setPointInLine(const int pointIndex, const int lineIndex, const SReal* baryCoords) override;
    int createPointInLine(const typename Out::Coord& p, int edgeIndex, const typename In::VecCoord* points);

    void init(const typename Out::VecCoord& out, const typename In::VecCoord& in);
    /// Called if the mapper should setup handling of topological changes
    void initTopologyChange();

    void projectDirtyPoints(const typename Out::VecCoord& out, const typename In::VecCoord& in);

    void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );
    void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );
    void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );
    void applyJT( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in );

    helper::ReadAccessor< Data< sofa::helper::vector<MappingData> > > readPoint2EdgeMap(){ return helper::ReadAccessor< Data< sofa::helper::vector<MappingData> > >(map);}
    helper::WriteAccessor< Data< sofa::helper::vector<MappingData> > > writePoint2EdgeMap() { return helper::WriteAccessor< Data< sofa::helper::vector<MappingData> > >(map);}

    helper::ReadAccessor< Data< VecBaryEdgeInfo > > readEdge2PointsMap(){ return helper::ReadAccessor< Data< VecBaryEdgeInfo > >(d_vBaryEdgeInfo);}
    helper::WriteAccessor< Data< VecBaryEdgeInfo > > writeEdge2PointsMap() { return helper::WriteAccessor< Data< VecBaryEdgeInfo > >(d_vBaryEdgeInfo);}

    void draw(const core::visual::VisualParams*,const typename Out::VecCoord& out, const typename In::VecCoord& in);

    virtual unsigned int getFromTopologyIndex(unsigned int toId) override;
    virtual sofa::defaulttype::Vector3 getFromTopologyBary(unsigned int toId) override;

    inline friend std::istream& operator >> ( std::istream& in, BarycentricMapperEdgeSetTopology<In, Out> &b )
    {
        unsigned int size_vec;

        in >> size_vec;
        sofa::helper::vector<MappingData>& m = *(b.map.beginEdit());
        m.clear();

        MappingData value;
        for (unsigned int i=0; i<size_vec; i++)
        {
            in >> value;
            m.push_back(value);
        }
        b.map.endEdit();
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const BarycentricMapperEdgeSetTopology<In, Out> & b )
    {

        out << b.map.getValue().size();
        out << " " ;
        out << b.map;

        return out;
    }
};

/// Class allowing barycentric mapping computation on a TriangleSetTopology
template<class In, class Out>
class BarycentricMapperTriangleSetTopology : public TopologyBarycentricMapper<In,Out>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapperTriangleSetTopology,In,Out),SOFA_TEMPLATE2(TopologyBarycentricMapper,In,Out));
    typedef TopologyBarycentricMapper<In,Out> Inherit;
    typedef typename Inherit::Real Real;
    typedef typename Inherit::OutReal OutReal;
    typedef typename Inherit::OutDeriv  OutDeriv;
    typedef typename Inherit::InDeriv  InDeriv;
    typedef typename Inherit::MappingData2D MappingData;
    typedef BMBaryElementInfo<Real> BaryElementInfo;

    typedef topology::Topology::TriangleID TriangleID;
    typedef topology::Topology::Triangle Triangle;
    typedef topology::Topology::PointID PointID;

    enum { NIn = Inherit::NIn };
    enum { NOut = Inherit::NOut };
    typedef typename Inherit::MBloc MBloc;

    // topologyData mechanism to handle topology changes (public)
    typedef typename sofa::helper::vector<BaryElementInfo> VecBaryTriangleInfo;
    sofa::component::topology::TriangleData<VecBaryTriangleInfo> d_vBaryTriangleInfo;
    // END topologyData mechanism (public)

protected:
    topology::PointData< sofa::helper::vector<MappingData> > map;
    topology::TriangleSetTopologyContainer*	        m_fromContainer;
    topology::PointSetTopologyContainer*            m_toContainer;

    bool m_useRestPosition;

    core::State< In >* m_stateFrom = nullptr;
    core::State< Out >* m_stateTo = nullptr;

    topology::TriangleSetGeometryAlgorithms<In>*    _fromGeomAlgo;

    struct Jacobian
    {
        std::vector< std::array<InDeriv,3> > jacobianVector; /// only used in template VecNormal3Types
    };

    Jacobian   J;          ///< Jacobian of the mapping, in a vector

    // topologyData mechanism to handle topology changes (protected) 
    class TriangleInfoHandler : public sofa::component::topology::TopologyDataHandler<Triangle, VecBaryTriangleInfo>
    {
    public:
        typedef topology::Topology::Triangle Triangle;

        typedef typename sofa::component::topology::TopologyDataHandler<Triangle, VecBaryTriangleInfo> TopologyDataHandler;
        TriangleInfoHandler(BarycentricMapperTriangleSetTopology* o, sofa::component::topology::TriangleData<VecBaryTriangleInfo>* d)
            : TopologyDataHandler(d), obj(o)
        {}

        virtual void applyCreateFunction(unsigned int t,
            BaryElementInfo& baryTriangleInfo,
            const Triangle & triangle,
            const sofa::helper::vector< unsigned int >& ancestors,
            const sofa::helper::vector< double >& coeffs) override;

        virtual void applyDestroyFunction(unsigned int t, BaryElementInfo& baryTriangleInfo) override;
        virtual void swap(unsigned int i1, unsigned int i2) override;

        //TODO
        //virtual void move(const sofa::helper::vector<unsigned int> &indexList,
        //    const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
        //    const sofa::helper::vector< sofa::helper::vector< double > >& coefs);

    protected:
        BarycentricMapperTriangleSetTopology* obj;
    };


    // topologyData mechanism to handle topology changes (protected)
    class MapPointInfoHandler : public sofa::component::topology::TopologyDataHandler<topology::Topology::Point, sofa::helper::vector<MappingData> >
    {
    public:
        typedef topology::Topology::Point Point;

        MapPointInfoHandler(BarycentricMapperTriangleSetTopology* o, sofa::component::topology::PointData<sofa::helper::vector<MappingData>>* d)
            : sofa::component::topology::TopologyDataHandler<Point, sofa::helper::vector<MappingData>>(d), obj(o)
        {}

        virtual void applyCreateFunction(unsigned int p, MappingData& poinInfo,
            const Point & point,
            const sofa::helper::vector< unsigned int > &,
            const sofa::helper::vector< double > &) override;
        virtual void applyDestroyFunction(unsigned int p, MappingData& pointInfo) override;
        virtual void swap(unsigned int i1, unsigned int i2) override;

        //TODO
        //virtual void move(const sofa::helper::vector<unsigned int> &indexList,
        //    const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
        //    const sofa::helper::vector< sofa::helper::vector< double > >& coefs);

    protected:
        BarycentricMapperTriangleSetTopology* obj;
    };

    std::unique_ptr<TriangleInfoHandler> m_vBTInfoHandler;
    std::unique_ptr<MapPointInfoHandler> m_vMapInfoHandler;
    std::set< PointID > m_dirtyPoints;

    // END topologyData mechanism (protected)


    BarycentricMapperTriangleSetTopology(topology::TriangleSetTopologyContainer* fromTopology,
            topology::PointSetTopologyContainer* toTopology, core::State< In >* stateFrom = nullptr, core::State< Out >* stateTo = nullptr, bool useRestPosition = false)
        : TopologyBarycentricMapper<In,Out>(fromTopology, toTopology),
          d_vBaryTriangleInfo(initData(&d_vBaryTriangleInfo, "vBaryTriangleInfo", "Vector of triangle information dedicated to topological changes")),
          map(initData(&map,"map", "mapper data")),
          m_fromContainer(fromTopology),
          m_toContainer(toTopology),
          m_useRestPosition(useRestPosition),
          m_stateFrom(stateFrom),
          m_stateTo(stateTo),
          _fromGeomAlgo(NULL)
    {
        m_vBTInfoHandler = std::unique_ptr<TriangleInfoHandler>(new TriangleInfoHandler(this, &d_vBaryTriangleInfo));
        m_vMapInfoHandler = std::unique_ptr<MapPointInfoHandler>(new MapPointInfoHandler(this, &map));
    }

    virtual ~BarycentricMapperTriangleSetTopology(){}

public:
    void clear(int reserve=0);

    virtual int addPointInTriangle(const int triangleIndex, const SReal* baryCoords);
    virtual int setPointInTriangle(const int pointIndex, const int triangleIndex, const SReal* baryCoords);
    int createPointInTriangle(const typename Out::Coord& p, int triangleIndex, const typename In::VecCoord* points);

    void init(const typename Out::VecCoord& out, const typename In::VecCoord& in);
    void projectDirtyPoints(const typename Out::VecCoord& out, const typename In::VecCoord& in);

    /// Called if the mapper should setup handling of topological changes
    void initTopologyChange();

    void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );
    void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );
    void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );
    void applyJT( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in );

    virtual unsigned int getFromTopologyIndex(unsigned int toId) override;
    virtual defaulttype::Vector3 getFromTopologyBary(unsigned int toId) override;

    void draw(const core::visual::VisualParams*,const typename Out::VecCoord& out, const typename In::VecCoord& in);

    helper::ReadAccessor< Data< sofa::helper::vector<MappingData> > > readPoint2TriangleMap(){ return helper::ReadAccessor< Data< sofa::helper::vector<MappingData> > >(map);}
    helper::WriteAccessor< Data< sofa::helper::vector<MappingData> > > writePoint2TriangleMap() { return helper::WriteAccessor< Data< sofa::helper::vector<MappingData> > >(map);}

    helper::ReadAccessor< Data< VecBaryTriangleInfo > > readTriangle2PointsMap(){ return helper::ReadAccessor< Data< VecBaryTriangleInfo > >(d_vBaryTriangleInfo);}
    helper::WriteAccessor< Data< VecBaryTriangleInfo > > writeTriangle2PointsMap() { return helper::WriteAccessor< Data< VecBaryTriangleInfo > >(d_vBaryTriangleInfo);}


    inline friend std::istream& operator >> ( std::istream& in, BarycentricMapperTriangleSetTopology<In, Out> &b )
    {
        unsigned int size_vec;

        in >> size_vec;

        sofa::helper::vector<MappingData>& m = *(b.map.beginEdit());
        m.clear();
        MappingData value;
        for (unsigned int i=0; i<size_vec; i++)
        {
            in >> value;
            m.push_back(value);
        }
        b.map.endEdit();
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const BarycentricMapperTriangleSetTopology<In, Out> & b )
    {

        out << b.map.getValue().size();
        out << " " ;
        out << b.map;

        return out;
    }
};



/// Class allowing barycentric mapping computation on a QuadSetTopology
template<class In, class Out>
class BarycentricMapperQuadSetTopology : public TopologyBarycentricMapper<In,Out>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapperQuadSetTopology,In,Out),SOFA_TEMPLATE2(TopologyBarycentricMapper,In,Out));
    typedef TopologyBarycentricMapper<In,Out> Inherit;
    typedef typename Inherit::Real Real;
    typedef typename Inherit::OutReal OutReal;
    typedef typename Inherit::OutDeriv  OutDeriv;
    typedef typename Inherit::InDeriv  InDeriv;
    typedef typename Inherit::MappingData2D MappingData;

    enum { NIn = Inherit::NIn };
    enum { NOut = Inherit::NOut };
    typedef typename Inherit::MBloc MBloc;

protected:
    topology::PointData< sofa::helper::vector<MappingData> >  map;
    topology::QuadSetTopologyContainer*			_fromContainer;
    topology::QuadSetGeometryAlgorithms<In>*	_fromGeomAlgo;

    BarycentricMapperQuadSetTopology(topology::QuadSetTopologyContainer* fromTopology,
            topology::PointSetTopologyContainer* _toTopology)
        : TopologyBarycentricMapper<In,Out>(fromTopology, _toTopology),
          map(initData(&map,"map", "mapper data")),
          _fromContainer(fromTopology),
          _fromGeomAlgo(NULL)
    {}

    virtual ~BarycentricMapperQuadSetTopology() {}

public:
    void clear(int reserve=0);

    int addPointInQuad(const int index, const SReal* baryCoords);
    int createPointInQuad(const typename Out::Coord& p, int index, const typename In::VecCoord* points);

    void init(const typename Out::VecCoord& out, const typename In::VecCoord& in);
    /// Called if the mapper should setup handling of topological changes
    void initTopologyChange();

    void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );
    void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );
    void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );
    void applyJT( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in );

    void draw(const core::visual::VisualParams*,const typename Out::VecCoord& out, const typename In::VecCoord& in);

    inline friend std::istream& operator >> ( std::istream& in, BarycentricMapperQuadSetTopology<In, Out> &b )
    {
        unsigned int size_vec;

        in >> size_vec;
        sofa::helper::vector<MappingData>& m = *(b.map.beginEdit());
        m.clear();
        MappingData value;
        for (unsigned int i=0; i<size_vec; i++)
        {
            in >> value;
            m.push_back(value);
        }
        b.map.endEdit();
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const BarycentricMapperQuadSetTopology<In, Out> & b )
    {

        out << b.map.getValue().size();
        out << " " ;
        out << b.map;

        return out;
    }

};

/// Class allowing barycentric mapping computation on a TetrahedronSetTopology
template<class In, class Out>
class BarycentricMapperTetrahedronSetTopology : public TopologyBarycentricMapper<In,Out>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapperTetrahedronSetTopology,In,Out),SOFA_TEMPLATE2(TopologyBarycentricMapper,In,Out));
    typedef TopologyBarycentricMapper<In,Out> Inherit;
    typedef typename Inherit::Real Real;
    typedef typename Inherit::OutReal OutReal;
    typedef typename Inherit::OutDeriv  OutDeriv;
    typedef typename Inherit::InDeriv  InDeriv;
    typedef typename Inherit::MappingData3D MappingData;
    typedef BMBaryElementInfo<Real> BaryElementInfo;

    typedef core::topology::Topology::PointID PointID;
    typedef typename In::VecCoord VecCoord;

    enum { NIn = Inherit::NIn };
    enum { NOut = Inherit::NOut };
    typedef typename Inherit::MBloc MBloc;

    typedef typename sofa::helper::vector<BaryElementInfo> VecBaryTetraInfo;
    topology::TetrahedronData<VecBaryTetraInfo> d_vBaryTetraInfo;

protected:
    topology::PointData<sofa::helper::vector<MappingData>> map;

    VecCoord actualTetraPosition;

    topology::TetrahedronSetTopologyContainer* m_fromContainer;
    topology::TetrahedronSetGeometryAlgorithms<In>*	m_fromGeomAlgo = nullptr;
    core::State< In >* m_stateFrom;
    core::State< Out >* m_stateTo;


    bool m_useRestPosition;

    BarycentricMapperTetrahedronSetTopology(topology::TetrahedronSetTopologyContainer* fromTopology, topology::PointSetTopologyContainer* _toTopology,
                                            core::State< In >* stateFrom, core::State< Out >* stateTo,
                                            bool useRestPosition = false)
        : TopologyBarycentricMapper<In,Out>(fromTopology, _toTopology),
          d_vBaryTetraInfo(initData(&d_vBaryTetraInfo, "vBaryTetraInfo", "Vector of tetra information dedicated to topological changes")),
          map(initData(&map,"map", "mapper data")),
          m_fromContainer(fromTopology),
          m_stateFrom(stateFrom),
          m_stateTo(stateTo),
          m_useRestPosition(useRestPosition),
          m_vertexInfoHandler(this, &map),
          m_tetraInfoHandler(this, &d_vBaryTetraInfo)
    {
    }

    virtual ~BarycentricMapperTetrahedronSetTopology() {}

    // topologyData mechanism to handle topology changes
    class TetraInfoHandler : public sofa::component::topology::TopologyDataHandler<core::topology::Topology::Tetra, VecBaryTetraInfo>
    {
    public:
        using Tetra = core::topology::Topology::Tetra;

        typedef typename sofa::component::topology::TopologyDataHandler<Tetra, VecBaryTetraInfo> TopologyDataHandler;

        TetraInfoHandler(BarycentricMapperTetrahedronSetTopology* mapper, topology::TetrahedronData<VecBaryTetraInfo>* data)
            : TopologyDataHandler(data), obj(mapper)
        {}

        virtual void applyCreateFunction(unsigned int t,
            BaryElementInfo& baryElementInfo,
            const Tetra & tetra,
            const sofa::helper::vector< unsigned int >& ancestors,
            const sofa::helper::vector< double >& coeffs) override;

        virtual void applyDestroyFunction(unsigned int t, BaryElementInfo& baryTetraInfo) override;
        virtual void swap(unsigned int i1, unsigned int i2) override;

        //TODO
        //virtual void move(const sofa::helper::vector<unsigned int> &indexList,
        //    const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
        //    const sofa::helper::vector< sofa::helper::vector< double > >& coefs);

    protected:
        BarycentricMapperTetrahedronSetTopology* obj;
    };

    class VertexInfoHandler : public sofa::component::topology::TopologyDataHandler<core::topology::Topology::Point, sofa::helper::vector<MappingData>>
    {
    public:
        using Point = core::topology::Topology::Point;

        using TopologyDataHandler = sofa::component::topology::TopologyDataHandler<Point, sofa::helper::vector<MappingData>>;

        VertexInfoHandler(BarycentricMapperTetrahedronSetTopology* mapper, topology::PointData<sofa::helper::vector<MappingData>>* data)
            : TopologyDataHandler(data), obj(mapper)
        {}

        virtual void applyCreateFunction(unsigned int t,
            MappingData& vertexInfo,
            const Point& vertex,
            const sofa::helper::vector<unsigned int>& ancestors,
            const sofa::helper::vector<double>& coeffs) override;

        virtual void applyDestroyFunction(unsigned int t, MappingData& vertexInfo) override;
        virtual void swap(unsigned int i1, unsigned int i2) override;
    protected:
        BarycentricMapperTetrahedronSetTopology* obj;
    };

    VertexInfoHandler m_vertexInfoHandler;
    TetraInfoHandler m_tetraInfoHandler;
    std::vector<PointID> m_dirtyPoints;
    // END topologyData mechanism

public:
    void clear(int reserve=0);

    void init(const typename Out::VecCoord& out, const typename In::VecCoord& in);
    void projectDirtyPoints(const typename Out::VecCoord& out, const typename In::VecCoord& in);
    /// Called if the mapper should setup handling of topological changes
    void initTopologyChange();

    void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );
    void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );
    void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );
    void applyJT( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in );

    void draw(const core::visual::VisualParams*,const typename Out::VecCoord& out, const typename In::VecCoord& in);

    const sofa::helper::vector<MappingData>& getMap() const {return this->map.getValue() ;}

    helper::ReadAccessor< Data< sofa::helper::vector<MappingData> > > readPoint2TetraMap(){ return helper::ReadAccessor< Data< sofa::helper::vector<MappingData> > >(map);}
    helper::WriteAccessor< Data< sofa::helper::vector<MappingData> > > writePoint2TetraMap() { return helper::WriteAccessor< Data< sofa::helper::vector<MappingData> > >(map);}

};



/// Class allowing barycentric mapping computation on a HexahedronSetTopology
template<class In, class Out>
class BarycentricMapperHexahedronSetTopology : public TopologyBarycentricMapper<In,Out>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapperHexahedronSetTopology,In,Out),SOFA_TEMPLATE2(TopologyBarycentricMapper,In,Out));
    typedef TopologyBarycentricMapper<In,Out> Inherit;
    typedef typename Inherit::Real Real;
    typedef typename Inherit::OutReal OutReal;
    typedef typename Inherit::OutDeriv  OutDeriv;
    typedef typename Inherit::InDeriv  InDeriv;
    typedef typename Inherit::MappingData3D MappingData;

    enum { NIn = Inherit::NIn };
    enum { NOut = Inherit::NOut };
    typedef typename Inherit::MBloc MBloc;

protected:
    topology::PointData< sofa::helper::vector<MappingData> >  map;
    topology::HexahedronSetTopologyContainer*		_fromContainer;
    topology::HexahedronSetGeometryAlgorithms<In>*	_fromGeomAlgo;

    std::set<int>	_invalidIndex;

    BarycentricMapperHexahedronSetTopology()
        : TopologyBarycentricMapper<In,Out>(NULL, NULL),
          map(initData(&map,"map", "mapper data")),
          _fromContainer(NULL),_fromGeomAlgo(NULL)
    {}

    BarycentricMapperHexahedronSetTopology(topology::HexahedronSetTopologyContainer* fromTopology,
            topology::PointSetTopologyContainer* _toTopology)
        : TopologyBarycentricMapper<In,Out>(fromTopology, _toTopology),
          map(initData(&map,"map", "mapper data")),
          _fromContainer(fromTopology),
          _fromGeomAlgo(NULL)
    {}

    virtual ~BarycentricMapperHexahedronSetTopology() {}

public:
    void clear(int reserve=0);

    int addPointInCube(const int index, const SReal* baryCoords);

    int setPointInCube(const int pointIndex, const int cubeIndex, const SReal* baryCoords);

    void init(const typename Out::VecCoord& out, const typename In::VecCoord& in);
    /// Called if the mapper should setup handling of topological changes
    void initTopologyChange();

    void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );
    void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );
    void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );
    void applyJT( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in );

    void draw(const core::visual::VisualParams*,const typename Out::VecCoord& out, const typename In::VecCoord& in);

    //-- test mapping partiel
    void applyOnePoint( const unsigned int& hexaId, typename Out::VecCoord& out, const typename In::VecCoord& in);
    //--

    // handle topology changes in the From topology
    virtual void handleTopologyChange(core::topology::Topology* t);

    bool isEmpty() {return this->map.getValue().empty();}
    void setTopology(topology::HexahedronSetTopologyContainer* _topology) {this->fromTopology = _topology; _fromContainer=_topology;}
    inline friend std::istream& operator >> ( std::istream& in, BarycentricMapperHexahedronSetTopology<In, Out> &b )
    {
        unsigned int size_vec;

        in >> size_vec;
        sofa::helper::vector<MappingData>& m = *(b.map.beginEdit());
        m.clear();
        MappingData value;
        for (unsigned int i=0; i<size_vec; i++)
        {
            in >> value;
            m.push_back(value);
        }
        b.map.endEdit();
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const BarycentricMapperHexahedronSetTopology<In, Out> & b )
    {

        out << b.map.getValue().size();
        out << " " ;
        out << b.map;

        return out;
    }


};



template <class TIn, class TOut>
class BarycentricMapping : public core::Mapping<TIn, TOut>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(BarycentricMapping,TIn,TOut), SOFA_TEMPLATE2(core::Mapping,TIn,TOut));

    typedef core::Mapping<TIn, TOut> Inherit;
    typedef TIn In;
    typedef TOut Out;
    typedef In InDataTypes;
    typedef typename InDataTypes::VecCoord InVecCoord;
    typedef typename InDataTypes::VecDeriv InVecDeriv;
    typedef typename InDataTypes::Coord InCoord;
    typedef typename InDataTypes::Deriv InDeriv;
    typedef typename InDataTypes::Real Real;
    typedef Out OutDataTypes;
    typedef typename OutDataTypes::VecCoord OutVecCoord;
    typedef typename OutDataTypes::VecDeriv OutVecDeriv;
    typedef typename OutDataTypes::Coord OutCoord;
    typedef typename OutDataTypes::Deriv OutDeriv;
    typedef typename OutDataTypes::Real OutReal;

    typedef core::topology::BaseMeshTopology BaseMeshTopology;

    typedef TopologyBarycentricMapper<InDataTypes,OutDataTypes> Mapper;
    //typedef BarycentricMapperRegularGridTopology<InDataTypes, OutDataTypes> RegularGridMapper;
    //typedef BarycentricMapperHexahedronSetTopology<InDataTypes, OutDataTypes> HexaMapper;

protected:

    SingleLink<BarycentricMapping<In,Out>,Mapper,BaseLink::FLAG_STRONGLINK> mapper;

public:

    Data< bool > useRestPosition;
    Data < bool > d_handleTopologyChange;

#ifdef SOFA_DEV
    //--- partial mapping test
    Data< bool > sleeping;
#endif
protected:
    BarycentricMapping();

    BarycentricMapping(core::State<In>* from, core::State<Out>* to, typename Mapper::SPtr mapper);

    BarycentricMapping(core::State<In>* from, core::State<Out>* to, BaseMeshTopology * topology=NULL );

    virtual ~BarycentricMapping();

public:
    void init();

    void reinit();

    void apply(const core::MechanicalParams *mparams /* PARAMS FIRST */, Data< typename Out::VecCoord >& out, const Data< typename In::VecCoord >& in);

    void applyJ(const core::MechanicalParams *mparams /* PARAMS FIRST */, Data< typename Out::VecDeriv >& out, const Data< typename In::VecDeriv >& in);

    void applyJT(const core::MechanicalParams *mparams /* PARAMS FIRST */, Data< typename In::VecDeriv >& out, const Data< typename Out::VecDeriv >& in);

    void applyJT(const core::ConstraintParams *cparams /* PARAMS FIRST */, Data< typename In::MatrixDeriv >& out, const Data< typename Out::MatrixDeriv >& in);

    void draw(const core::visual::VisualParams* vparams);

    // handle topology changes depending on the topology
    virtual void handleTopologyChange(core::topology::Topology* t);

    // interface for continuous friction contact



    TopologyBarycentricMapper<InDataTypes,OutDataTypes> *getMapper()
    {
        return mapper.get();
    }

    sofa::core::topology::BaseMeshTopology* getFromTopo() { return topology_from; }
    sofa::core::topology::BaseMeshTopology* getToTopo() { return topology_to; }

protected:
    sofa::core::topology::BaseMeshTopology* topology_from;
    sofa::core::topology::BaseMeshTopology* topology_to;

private:
    void createMapperFromTopology(BaseMeshTopology * topology);
};


template<class Real, int NC,  int NP>
class BMMappingData
{
public:
    int in_index;
    //unsigned int points[NP];
    sofa::helper::fixed_array<Real, NC> baryCoords;

    inline friend std::istream& operator >> ( std::istream& in, BMMappingData<Real, NC, NP> &m )
    {
        in>>m.in_index;
        for (int i=0; i<NC; i++) in >> m.baryCoords[i];
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const BMMappingData<Real, NC , NP > & m )
    {
        out << m.in_index;
        for (int i=0; i<NC; i++)
            out << " " << m.baryCoords[i];
        out << "\n";
        return out;
    }
    SOFA_STRUCT_DECL(BMMappingData, in_index, baryCoords);
//    SOFA_STRUCT_STREAM_METHODS(BMMappingData);
//    SOFA_STRUCT_COMPARE_METHOD(BMMappingData);
};


template<class Real>
struct BMBaryElementInfo
{
    using PointID = core::topology::Topology::PointID;

    bool dirty = true; // dirty for newly created triangles since last apply call
    sofa::helper::unordered_set<PointID> vPointsIncluded; // ID of the points that are projected on the element
    sofa::defaulttype::Mat3x3d restBase;           // Base matrix used in the projection computation (in restPositions)
    sofa::defaulttype::Vec<3, Real> restCenter;         // Center of the element (in restPositions)

    inline friend std::ostream& operator<< (std::ostream& os, const BMBaryElementInfo<Real>& baryElementInfo)
    {
        os << " vPointsIncluded = " << baryElementInfo.vPointsIncluded
           << " restBase = " << baryElementInfo.restBase
           << " restCenter = " << baryElementInfo.restCenter
           << "\n";
        return os;
    }
    inline friend std::istream& operator >> (std::istream& in, BMBaryElementInfo<Real>&)
    {
        return in;
    }

    SOFA_STRUCT_DECL(BMBaryElementInfo, dirty, vPointsIncluded, restBase, restCenter);
//    SOFA_STRUCT_STREAM_METHODS(BMBaryElementInfo);
//    SOFA_STRUCT_COMPARE_METHOD(BMBaryElementInfo);
};

} // namespace mapping

} // namespace component

} // namespace sofa


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_MAPPING_BARYCENTRICMAPPING_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapping< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapping< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapper< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapper< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::TopologyBarycentricMapper< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::TopologyBarycentricMapper< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperRegularGridTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperRegularGridTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperSparseGridTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperSparseGridTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperMeshTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperMeshTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperEdgeSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperEdgeSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTriangleSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTriangleSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperQuadSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperQuadSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTetrahedronSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTetrahedronSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperHexahedronSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperHexahedronSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes >;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapping< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapping< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapper< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapper< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::TopologyBarycentricMapper< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::TopologyBarycentricMapper< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperRegularGridTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperRegularGridTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperSparseGridTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperSparseGridTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperMeshTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperMeshTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperEdgeSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperEdgeSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTriangleSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTriangleSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperQuadSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperQuadSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTetrahedronSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTetrahedronSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperHexahedronSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperHexahedronSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::ExtVec3fTypes >;
#endif
#ifndef SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapping< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapping< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapper< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapper< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::TopologyBarycentricMapper< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::TopologyBarycentricMapper< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperRegularGridTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperRegularGridTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperSparseGridTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperSparseGridTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperMeshTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperMeshTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperEdgeSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperEdgeSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTriangleSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTriangleSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperQuadSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperQuadSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTetrahedronSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperTetrahedronSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperHexahedronSetTopology< sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3fTypes >;
extern template class SOFA_BASE_MECHANICS_API sofa::component::mapping::BarycentricMapperHexahedronSetTopology< sofa::defaulttype::Vec3fTypes, sofa::defaulttype::Vec3dTypes >;
#endif
#endif
#endif



#endif
