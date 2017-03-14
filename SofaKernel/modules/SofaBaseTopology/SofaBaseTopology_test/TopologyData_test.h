#ifndef TopologyData_TEST_H
#define TopologyData_TEST_H

#include <gtest/gtest.h>

#include <SofaBaseTopology/TriangleSetTopologyModifier.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>

#include <sofa/core/topology/BaseTopologyData.h>
#include <SofaBaseTopology/TopologyData.inl>
#include <SofaBaseTopology/TopologyDataHandler.inl>

#include <SofaSimulationTree/GNode.h>
#include <SofaSimulationTree/TreeSimulation.h>
#include <sofa/simulation/Node.h>

#include <sofa/helper/vector.h>
#include <sofa/helper/map.h>


struct SubdivideTriangleInfo
{
    sofa::component::topology::TriangleSetTopologyContainer::TriangleID triangleId;
    sofa::defaulttype::Vec<2, sofa::defaulttype::Vec3Types::Real> bary;
};

class TopologyData_test : public virtual sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(TopologyData_test, sofa::core::objectmodel::BaseObject);
    typedef typename sofa::component::topology::TriangleSetTopologyContainer TriangleSetTopologyContainer;
    typedef typename sofa::core::topology::Triangle Triangle;
    typedef typename sofa::core::topology::Edge Edge;
    typedef typename sofa::core::topology::Point Point;

    TopologyData_test();
    ~TopologyData_test();

    TriangleSetTopologyContainer* m_topology;

    void init();
    inline void setTopology(TriangleSetTopologyContainer* topo) { m_topology = topo; }
    inline TriangleSetTopologyContainer* getTopology() { return m_topology; }
    
    struct TestStruct
    {
        unsigned int id;
        double baryCoord[2];
        inline friend std::ostream& operator<< (std::ostream& os, const TestStruct& ts)
        {
            os << " id = " << ts.id
                << " baryCoord = " << ts.baryCoord[0] << ts.baryCoord[1];
            return os;
        }
        inline friend std::istream& operator >> (std::istream& in, TestStruct& ts)
        {
            return in;
        }

    };

    typedef sofa::helper::vector<TestStruct> TestStructVec;
    typedef typename std::map<unsigned int, TestStruct> MapTestStruct;

    sofa::component::topology::TriangleData< MapTestStruct > triangleInfo;
    sofa::component::topology::EdgeData< MapTestStruct > edgeInfo;
    sofa::component::topology::PointData< MapTestStruct > pointInfo;

    

protected:
    class TriangleInfoHandler : public sofa::component::topology::TopologyDataHandler<Triangle, MapTestStruct >
    {
    public:
        typedef typename sofa::component::topology::TopologyDataHandler<Triangle, MapTestStruct > TopologyDataHandler;
        TriangleInfoHandler(TopologyData_test* o, sofa::component::topology::TriangleData<MapTestStruct>* d)
            : TopologyDataHandler(d), obj(o)
        {}

        virtual void applyCreateFunction(unsigned int triangleIndex, TestStruct& ts,
        const Triangle & t,
        const sofa::helper::vector< unsigned int >& ancestors,
        const sofa::helper::vector< double >& coeffs);

    protected:
        TopologyData_test* obj;
    };

    class EdgeInfoHandler : public sofa::component::topology::TopologyDataHandler<Edge, MapTestStruct >
    {
    public:
        typedef typename sofa::component::topology::TopologyDataHandler<Edge, MapTestStruct > TopologyDataHandler;
        EdgeInfoHandler(TopologyData_test* o, sofa::component::topology::EdgeData<MapTestStruct >* d)
            : TopologyDataHandler(d), obj(o)
        {}

        virtual void applyCreateFunction(unsigned int triangleIndex, TestStruct& ts,
            const Edge & t,
            const sofa::helper::vector< unsigned int >& ancestors,
            const sofa::helper::vector< double >& coeffs);

    protected:
        TopologyData_test* obj;
    };

    class PointInfoHandler : public sofa::component::topology::TopologyDataHandler<Point, MapTestStruct >
    {
    public:
        typedef typename sofa::component::topology::TopologyDataHandler<Point, MapTestStruct > TopologyDataHandler;
        PointInfoHandler(TopologyData_test* o, sofa::component::topology::PointData<MapTestStruct >* d)
            : TopologyDataHandler(d), obj(o)
        {}

        virtual void applyCreateFunction(unsigned int triangleIndex, TestStruct& ts,
            const Point & t,
            const sofa::helper::vector< unsigned int >& ancestors,
            const sofa::helper::vector< double >& coeffs);

    protected:
        TopologyData_test* obj;
    };

    TriangleInfoHandler* tih;
    EdgeInfoHandler* eih;
    PointInfoHandler* pih;
};

#endif //TopologyData_TEST_H


class TopologyData_fixture : public ::testing::Test
{
public:
    typedef typename sofa::component::topology::TriangleSetTopologyModifier TriangleSetTopologyModifier;
    typedef typename sofa::component::topology::TriangleSetTopologyContainer TriangleSetTopologyContainer;
    typedef typename sofa::core::topology::BaseMeshTopology::PointID PointID;
    typedef typename sofa::core::topology::BaseMeshTopology::TriangleID TriangleID;
    typedef typename sofa::core::topology::Triangle Triangle;
    typedef typename sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex TrianglesAroundVertex;
         
    virtual void SetUp();
    virtual void TearDown();

    sofa::simulation::Node::SPtr    rootNode;
    TriangleSetTopologyModifier::SPtr modifier;
    TriangleSetTopologyContainer::SPtr container;
    TopologyData_test::SPtr topoDataTest;

    void subdivideTriangles(const std::vector<SubdivideTriangleInfo>& pointsToAdd);

    void removePoints(const std::vector<PointID>& pointsToRemove);
};
