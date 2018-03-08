#include <gtest/gtest.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/EdgeSetTopologyModifier.h>
#include <sofa/simulation/tree/GNode.h>
#include <type_traits>

using namespace sofa::component::topology;
using namespace sofa::simulation;
using namespace sofa::simulation::tree;

namespace
{

TEST(EdgeSetTopology_test, checkEdgeSetTopologyIsEmptyWhenConstructed)
{
    EdgeSetTopologyContainer::SPtr edgeContainer = sofa::core::objectmodel::New< EdgeSetTopologyContainer >();
    EXPECT_EQ(0, edgeContainer->getNbPoints());
    EXPECT_EQ(0u, edgeContainer->getPoints().size());
    EXPECT_EQ(0, edgeContainer->getNbEdges());
    EXPECT_EQ(0u, edgeContainer->getEdges().size());
}


TEST(EdgeSetTopology_test, checkEdgeSetTopologyInitializationOfNeighborhoodInformation)
{
    EdgeSetTopologyContainer::SPtr edgeContainer = sofa::core::objectmodel::New< EdgeSetTopologyContainer >();

    sofa::helper::WriteAccessor< sofa::Data< EdgeSetTopologyContainer::SeqEdges > > edges = edgeContainer->d_edge;

    edges.push_back(EdgeSetTopologyContainer::Edge(0, 1));
    edges.push_back(EdgeSetTopologyContainer::Edge(3, 4));

    edgeContainer->init();

    ASSERT_EQ(5, edgeContainer->getNbPoints());
    EXPECT_EQ(2, edgeContainer->getNbEdges());

    //check adjacency information
    {
        const EdgeSetTopologyContainer* constEdgeContainer = edgeContainer.get();
        const auto eavArray = constEdgeContainer->getEdgesAroundVertexArray();
        ASSERT_EQ(5u, eavArray.size());
        
        
        const auto& eav0 = constEdgeContainer->getEdgesAroundVertex(PointID(0));
        ASSERT_EQ(1u, eav0.size());
        EXPECT_EQ(EdgeID(0), eav0[0]);

        const auto& eav1 = constEdgeContainer->getEdgesAroundVertex(PointID(1));
        ASSERT_EQ(1u, eav1.size());
        EXPECT_EQ(EdgeID(0), eav1[0]);

        const auto& eav2 = constEdgeContainer->getEdgesAroundVertex(PointID(2));
        EXPECT_EQ(0u, eav2.size());

        const auto& eav3 = constEdgeContainer->getEdgesAroundVertex(PointID(3));
        ASSERT_EQ(1u, eav3.size());
        EXPECT_EQ(EdgeID(1), eav3[0]);

        const auto& eav4 = constEdgeContainer->getEdgesAroundVertex(PointID(4));
        ASSERT_EQ(1u, eav4.size());
        EXPECT_EQ(EdgeID(1), eav4[0]);

        const auto eavArrayExpected = edgeContainer->getEdgesAroundVertexArray();
        EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
        EXPECT_TRUE( std::equal(eavArray.begin(),eavArray.end(), eavArrayExpected.begin() ) );
    }
}

struct EdgeSetTopology
{
    Node::SPtr node;
    EdgeSetTopologyContainer::SPtr edgeContainer;
    EdgeSetTopologyModifier::SPtr  edgeModifier;

    void init();

    void addOnePoint();

    bool removeLastPoint();

    void addOneEdge();

    bool removeLastEdge(bool removePoints);
};

void EdgeSetTopology::init()
{
    node          = sofa::core::objectmodel::New< GNode >(std::string(""));
    edgeContainer = sofa::core::objectmodel::New< EdgeSetTopologyContainer >();
    edgeModifier  = sofa::core::objectmodel::New< EdgeSetTopologyModifier >();

    node->addObject(edgeContainer);
    node->addObject(edgeModifier);
    node->init(sofa::core::ExecParams::defaultInstance());

}

void EdgeSetTopology::addOnePoint()
{
    edgeModifier->addPoints(1, false);
}

bool EdgeSetTopology::removeLastPoint()
{
    if (edgeContainer->getNbPoints() == 0)
    {
        return false;
    }
    sofa::helper::vector< PointID > indices;
    indices.push_back(edgeContainer->getNbPoints() - 1);

    edgeModifier->removePointsWarning(indices, false);
    edgeModifier->removePointsProcess(indices, false);

    return true;
}

void EdgeSetTopology::addOneEdge()
{
    EdgeSetTopologyContainer::SeqEdges edgesToAdd;
    edgesToAdd.push_back(Edge(edgeContainer->getNbPoints(), edgeContainer->getNbPoints() + 1));

    edgeModifier->addPoints(2, false);
    edgeModifier->addEdges(edgesToAdd);
}

bool EdgeSetTopology::removeLastEdge(bool removePoints)
{
    if (edgeContainer->getNbEdges() == 0)
    {
        return false;
    }

    sofa::helper::vector< EdgeID > lastEdgeIndex;
    lastEdgeIndex.push_back(edgeContainer->getNbEdges() - 1);
    edgeModifier->removeEdges(lastEdgeIndex, removePoints, true);

    return true;
}

TEST(EdgeSetTopology_test, checkEdgeSetTopologyAddPoint)
{
    EdgeSetTopology topology;
    topology.init();
    topology.addOnePoint();
    EXPECT_EQ(1, topology.edgeContainer->getNbPoints());
    //check adjacency information
    //check adjacency information
    const EdgeSetTopologyContainer* constEdgeContainer = topology.edgeContainer.get();
    const auto eavArray = constEdgeContainer->getEdgesAroundVertexArray();
    ASSERT_EQ(1u, eavArray.size());
    EXPECT_EQ(0u, eavArray[0].size());

    const auto eavArrayExpected = topology.edgeContainer->getEdgesAroundVertexArray();
    EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
    EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));
}

TEST(EdgeSetTopology_test, checkEdgeSetTopologyRemovePoint)
{
    EdgeSetTopology topology;
    topology.init();
    topology.addOnePoint();
    topology.removeLastPoint();

    EXPECT_EQ(0, topology.edgeContainer->getNbPoints());
    //check adjacency information
    const EdgeSetTopologyContainer* constEdgeContainer = topology.edgeContainer.get();
    const auto eavArray = constEdgeContainer->getEdgesAroundVertexArray();
    EXPECT_EQ(0u, eavArray.size());

    const auto eavArrayExpected = topology.edgeContainer->getEdgesAroundVertexArray();
    EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
    EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));
}

TEST(EdgeSetTopology_test, checkEdgeSetTopologyAddEdge)
{
    EdgeSetTopology topology;
    topology.init();
    topology.addOneEdge();

    EXPECT_EQ(2, topology.edgeContainer->getNbPoints());
    EXPECT_EQ(1, topology.edgeContainer->getNbEdges());

    //check adjacency information
    {
        const EdgeSetTopologyContainer* constEdgeContainer = topology.edgeContainer.get();
        const auto eavArray = constEdgeContainer->getEdgesAroundVertexArray();
        ASSERT_EQ(2u, eavArray.size());
        
        const auto& eav0 = constEdgeContainer->getEdgesAroundVertex(PointID(0));
        ASSERT_EQ(1u, eav0.size());
        EXPECT_EQ(EdgeID(0), eav0[0]);

        const auto& eav1 = constEdgeContainer->getEdgesAroundVertex(PointID(1));
        ASSERT_EQ(1u, eav1.size());
        EXPECT_EQ(EdgeID(0), eav1[0]);

        const auto eavArrayExpected = topology.edgeContainer->getEdgesAroundVertexArray();
        EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
        EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));
    }
}

TEST(EdgeSetTopology_test, checkEdgeSetTopologyRemoveEdgeKeepPoints)
{
    EdgeSetTopology topology;
    topology.init();
    topology.addOneEdge();
    topology.addOneEdge();

    topology.removeLastEdge(false);

    EXPECT_EQ(4, topology.edgeContainer->getNbPoints());
    EXPECT_EQ(1, topology.edgeContainer->getNbEdges());

    //check adjacency information
    {
        const EdgeSetTopologyContainer* constEdgeContainer = topology.edgeContainer.get();
        const auto eavArray = constEdgeContainer->getEdgesAroundVertexArray();
        ASSERT_EQ(4u, eavArray.size());

        const auto& eav0 = constEdgeContainer->getEdgesAroundVertex(PointID(0));
        ASSERT_EQ(1u, eav0.size());
        EXPECT_EQ(EdgeID(0), eav0[0]);

        const auto& eav1 = constEdgeContainer->getEdgesAroundVertex(PointID(1));
        ASSERT_EQ(1u, eav1.size());
        EXPECT_EQ(EdgeID(0), eav1[0]);

        const auto& eav2 = constEdgeContainer->getEdgesAroundVertex(PointID(2));
        EXPECT_EQ(0u, eav2.size());

        const auto& eav3 = constEdgeContainer->getEdgesAroundVertex(PointID(3));
        EXPECT_EQ(0u, eav3.size());

        const auto eavArrayExpected = topology.edgeContainer->getEdgesAroundVertexArray();
        EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
        EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));
    }
}


TEST(EdgeSetTopology_test, checkEdgeSetTopologyRemoveEdgeRemovePoints)
{
    EdgeSetTopology topology;
    topology.init();
    topology.addOneEdge();
    topology.addOneEdge();

    topology.removeLastEdge(true);

    EXPECT_EQ(2, topology.edgeContainer->getNbPoints());
    EXPECT_EQ(1, topology.edgeContainer->getNbEdges());

    //check adjacency information
    {
        const EdgeSetTopologyContainer* constEdgeContainer = topology.edgeContainer.get();
        const auto eavArray = constEdgeContainer->getEdgesAroundVertexArray();
        ASSERT_EQ(2u, eavArray.size());

        const auto& eav0 = constEdgeContainer->getEdgesAroundVertex(PointID(0));
        ASSERT_EQ(1u, eav0.size());
        EXPECT_EQ(EdgeID(0), eav0[0]);

        const auto& eav1 = constEdgeContainer->getEdgesAroundVertex(PointID(1));
        ASSERT_EQ(1u, eav1.size());
        EXPECT_EQ(EdgeID(0), eav1[0]);

        const auto eavArrayExpected = topology.edgeContainer->getEdgesAroundVertexArray();
        EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
        EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));
    }
}


}
