#include <sofa/simulation/Node.h>
#include <gtest/gtest.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyModifier.h>
#include <SofaSimulationTree/GNode.h>

using namespace sofa::component::topology;
using namespace sofa::simulation;


namespace
{

bool compareEdgesInTriangle(const TriangleSetTopologyContainer::EdgesInTriangle& lhs, const TriangleSetTopologyContainer::EdgesInTriangle& rhs)
{
    return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

TEST(TriangleSetTopology_test, checkTriangleSetTopologyIsEmptyWhenConstructed)
{
    TriangleSetTopologyContainer::SPtr triangleContainer = sofa::core::objectmodel::New< TriangleSetTopologyContainer >();
    EXPECT_EQ(0, triangleContainer->getNbPoints());
    EXPECT_EQ(0, triangleContainer->getPoints().size());
    EXPECT_EQ(0, triangleContainer->getNbEdges());
    EXPECT_EQ(0, triangleContainer->getEdges().size());
    EXPECT_EQ(0, triangleContainer->getNbTriangles());
    EXPECT_EQ(0, triangleContainer->getTriangles().size());
}


TEST(TriangleSetTopology_test, checkTriangleSetTopologyInitializationOfNeighborhoodInformation)
{
    TriangleSetTopologyContainer::SPtr triangleContainer = sofa::core::objectmodel::New< TriangleSetTopologyContainer >();

    triangleContainer->setNbPoints(3); // EdgeSetTopologyContainer automagically adds point base at int based on the list of edges that has been declared.
                                       // TriangleSetTopologyContainer does not. No matter the type of decision made, it should be consistent accross the various containers...
    sofa::helper::WriteAccessor< sofa::Data< TriangleSetTopologyContainer::SeqTriangles > > triangles = triangleContainer->d_triangle;
    triangles.push_back(TriangleSetTopologyContainer::Triangle(0, 1, 2));

    triangleContainer->init();

    ASSERT_EQ(3, triangleContainer->getNbPoints());
    EXPECT_EQ(3, triangleContainer->getNbEdges());
    EXPECT_EQ(1, triangleContainer->getNbTriangles());

    //check adjacency information is computed
    {
        const TriangleSetTopologyContainer* constTriangleContainer = triangleContainer.get();
        
        const auto eitArray = constTriangleContainer->getEdgesInTriangleArray();
        ASSERT_EQ(1, eitArray.size());
        ASSERT_EQ(3, eitArray[0].size());
        EXPECT_EQ(TriangleSetTopologyContainer::EdgeID(0), eitArray[0][0]);
        EXPECT_EQ(TriangleSetTopologyContainer::EdgeID(1), eitArray[0][1]);
        EXPECT_EQ(TriangleSetTopologyContainer::EdgeID(2), eitArray[0][2]);
        
        const auto eavArray = constTriangleContainer->getEdgesAroundVertexArray();
        ASSERT_EQ(3, eavArray.size());

        for (std::size_t i = 0; i < 3; ++i)
        {
            const auto& eavi = constTriangleContainer->getEdgesAroundVertex(TriangleSetTopologyContainer::PointID(i));
            ASSERT_EQ(2, eavi.size());

            sofa::helper::vector< TriangleSetTopologyContainer::EdgeID > eav_expected;

            std::cout << "Point: " << i << " eav: " << eavi << std::endl;
            // point 0 : edges 0,2 
            // point 1 : egdes 1,2
            // point 2 : edges 0,1

            for (std::size_t j = 0; j < 2; ++j)
            {
                eav_expected.push_back(TriangleSetTopologyContainer::EdgeID((i + (j + 1)) % 3));
            }
            std::sort(eav_expected.begin(), eav_expected.end());

            for (std::size_t j = 0; j < 2; ++j)
            {
                EXPECT_EQ(eav_expected[j], eavi[j]);
            }
        }

        const auto tavArray = constTriangleContainer->getTrianglesAroundVertexArray();
        ASSERT_EQ(3, tavArray.size());

        for (std::size_t i = 0; i < 3; ++i)
        {
            const auto& tavi = constTriangleContainer->getTrianglesAroundVertex(TriangleSetTopologyContainer::PointID(i));
            ASSERT_EQ(1, tavi.size());
            EXPECT_EQ(TriangleSetTopologyContainer::TriangleID(0), tavi[0]);
        }

        const auto eatArray = constTriangleContainer->getTrianglesAroundEdgeArray();
        ASSERT_EQ(3, eatArray.size());

        for (std::size_t i = 0; i < 3; ++i)
        {
            const auto& eati = constTriangleContainer->getTrianglesAroundEdge(TriangleSetTopologyContainer::EdgeID(i));
            ASSERT_EQ(1, eati.size());
            EXPECT_EQ(TriangleSetTopologyContainer::TriangleID(0), eati[0]);
        }

        const auto eitArrayExpected = triangleContainer->getEdgesInTriangleArray();
        EXPECT_EQ(eitArrayExpected.size(), eitArray.size());
        EXPECT_TRUE(std::equal(eitArray.begin(), eitArray.end(), eitArrayExpected.begin(), compareEdgesInTriangle ));

        const auto tavArrayExpected = triangleContainer->getTrianglesAroundVertexArray();
        EXPECT_EQ(tavArrayExpected.size(), tavArray.size());
        EXPECT_TRUE(std::equal(tavArray.begin(), tavArray.end(), tavArrayExpected.begin()));

        const auto eatArrayExpected = triangleContainer->getTrianglesAroundEdgeArray();
        EXPECT_EQ(eatArrayExpected.size(), eatArray.size());
        EXPECT_TRUE(std::equal(eatArray.begin(), eatArray.end(), eatArrayExpected.begin() ) );

    }
}

struct TriangleSetTopology
{
    sofa::simulation::Node::SPtr node;
    TriangleSetTopologyContainer::SPtr triangleContainer;
    TriangleSetTopologyModifier::SPtr  triangleModifier;

    void init();

    void addOnePoint();

    void addOneEdge();

    void addOneTriangle();

    bool removeLastPoint();

    bool removeLastEdge(bool removePoints);

    bool removeLastTriangle(bool removeEdges, bool removePoints);

};

void TriangleSetTopology::init()
{
    node = sofa::core::objectmodel::New< sofa::simulation::tree::GNode >(std::string(""));
    triangleContainer = sofa::core::objectmodel::New< TriangleSetTopologyContainer >();
    triangleModifier  = sofa::core::objectmodel::New< TriangleSetTopologyModifier >();

    node->addObject(triangleContainer);
    node->addObject(triangleModifier);
    node->init(sofa::core::ExecParams::defaultInstance());
}

void TriangleSetTopology::addOnePoint()
{
    triangleModifier->addPoints(1, false);
}

void TriangleSetTopology::addOneEdge()
{
    TriangleSetTopologyContainer::SeqEdges edgesToAdd;
    edgesToAdd.push_back(EdgeSetTopologyContainer::Edge(triangleContainer->getNbPoints(),
                              triangleContainer->getNbPoints()+1));
    triangleModifier->addPoints(2, false);
    triangleModifier->addEdges(edgesToAdd);
}

void TriangleSetTopology::addOneTriangle()
{
    TriangleSetTopologyContainer::SeqTriangles trianglesToAdd;
    trianglesToAdd.push_back(EdgeSetTopologyContainer::Triangle(triangleContainer->getNbPoints(),
                                      triangleContainer->getNbPoints() + 1, 
                                      triangleContainer->getNbPoints() + 2));

    triangleModifier->addPoints(3, false);
    triangleModifier->addTriangles(trianglesToAdd);
}

bool TriangleSetTopology::removeLastPoint()
{
    if (triangleContainer->getNbPoints() == 0)
    {
        return false;
    }

    sofa::helper::vector< EdgeSetTopologyContainer::PointID > indices;
    indices.push_back(triangleContainer->getNbPoints() - 1);

    triangleModifier->removePointsWarning(indices, false);
    triangleModifier->removePointsProcess(indices, false);

    return true;
}

bool TriangleSetTopology::removeLastEdge(bool removePoints)
{
    if (triangleContainer->getNbEdges() == 0)
    {
        return false;
    }

    sofa::helper::vector<EdgeSetTopologyContainer::EdgeID>  edgesToRemove;
    edgesToRemove.push_back(triangleContainer->getNbEdges() - 1);
    triangleModifier->removeEdges(edgesToRemove, removePoints);

    return true;
}

bool TriangleSetTopology::removeLastTriangle(bool removeEdges, bool removePoints)
{
    if (triangleContainer->getNbTriangles() == 0)
    {
        return false;
    }

    sofa::helper::vector< EdgeSetTopologyContainer::TriangleID > lastTriangleIndex;
    lastTriangleIndex.push_back(triangleContainer->getNbTriangles() - 1);
    triangleModifier->removeTriangles(lastTriangleIndex, removeEdges, removePoints);

    return true;
}

TEST(TriangleSetTopology_test, checkTriangleSetTopologyAddPoint)
{
    TriangleSetTopology topology;
    topology.init();
    topology.addOnePoint();

    const TriangleSetTopologyContainer* constTriangleContainer = topology.triangleContainer.get();

    EXPECT_EQ(1, constTriangleContainer->getNbPoints());
    EXPECT_EQ(0, constTriangleContainer->getTrianglesAroundEdgeArray().size());
    
    ASSERT_EQ(1, constTriangleContainer->getEdgesAroundVertexArray().size());
    EXPECT_EQ(0, constTriangleContainer->getEdgesAroundVertex(0).size());

    ASSERT_EQ(1, constTriangleContainer->getTrianglesAroundVertexArray().size() );
    EXPECT_EQ(0, constTriangleContainer->getTrianglesAroundVertex(0).size() );
}


TEST(TriangleSetTopology_test, checkTriangleSetTopologyRemovePoint)
{
    TriangleSetTopology topology;
    topology.init();
    topology.addOnePoint();
    topology.removeLastPoint();

    const TriangleSetTopologyContainer* constTriangleContainer = topology.triangleContainer.get();

    EXPECT_EQ(0, constTriangleContainer->getNbPoints());
    EXPECT_EQ(0, constTriangleContainer->getTrianglesAroundEdgeArray().size());
    EXPECT_EQ(0, constTriangleContainer->getEdgesAroundVertexArray().size());
    EXPECT_EQ(0, constTriangleContainer->getTrianglesAroundVertexArray().size());
}

TEST(TriangleSetTopology_test, checkTriangleSetTopologyAddEdge)
{
    TriangleSetTopology topology;
    topology.init();
    topology.addOneEdge();
    const TriangleSetTopologyContainer* constTriangleContainer = topology.triangleContainer.get();

    EXPECT_EQ(2, topology.triangleContainer->getNbPoints());
    EXPECT_EQ(1, topology.triangleContainer->getNbEdges());

    const auto taeArray         = constTriangleContainer->getTrianglesAroundEdgeArray();
    ASSERT_EQ(1, taeArray.size());
    EXPECT_EQ(0, taeArray[0].size());

    const auto eavArray = constTriangleContainer->getEdgesAroundVertexArray();
    ASSERT_EQ(2, eavArray.size());
    ASSERT_EQ(1, constTriangleContainer->getEdgesAroundVertex(0).size());
    EXPECT_EQ(0, constTriangleContainer->getEdgesAroundVertex(0)[0]);
    ASSERT_EQ(1, constTriangleContainer->getEdgesAroundVertex(1).size());
    EXPECT_EQ(0, constTriangleContainer->getEdgesAroundVertex(1)[0]);

    const auto tavArray = constTriangleContainer->getTrianglesAroundVertexArray();
    ASSERT_EQ(2, tavArray.size());
    EXPECT_EQ(0, constTriangleContainer->getTrianglesAroundVertex(0).size());
    EXPECT_EQ(0, constTriangleContainer->getTrianglesAroundVertex(1).size());

    const auto taeArrayExpected = topology.triangleContainer->getTrianglesAroundEdgeArray();
    EXPECT_EQ(taeArrayExpected.size(), taeArray.size());
    EXPECT_TRUE( std::equal(taeArray.begin(), taeArray.end(), taeArrayExpected.begin()) );

    const auto eavArrayExpected = topology.triangleContainer->getEdgesAroundVertexArray();
    EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
    EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));

    const auto tavArrayExpected = topology.triangleContainer->getTrianglesAroundVertexArray();
    EXPECT_EQ(tavArrayExpected.size(), tavArray.size());
    EXPECT_TRUE(std::equal(tavArray.begin(), tavArray.end(), tavArrayExpected.begin()));

}

TEST(TriangleSetTopology_test, checkTriangleSetTopologyRemoveEdgeRemovePoints)
{
    TriangleSetTopology topology;
    topology.init();
    topology.addOneEdge();
    topology.removeLastEdge(true);

    const TriangleSetTopologyContainer* constTriangleContainer = topology.triangleContainer.get();

    EXPECT_EQ(0, topology.triangleContainer->getNbPoints() );
    EXPECT_EQ(0, topology.triangleContainer->getNbEdges() );

    EXPECT_EQ(0, constTriangleContainer->getTrianglesAroundEdgeArray().size());
    EXPECT_EQ(0, constTriangleContainer->getEdgesAroundVertexArray().size());
    EXPECT_EQ(0, constTriangleContainer->getTrianglesAroundVertexArray().size());
}

TEST(TriangleSetTopology_test, checkTriangleSetTopologyAddTriangle)
{
    TriangleSetTopology topology;
    topology.init();
    topology.addOneTriangle();

    EXPECT_EQ(3, topology.triangleContainer->getNbPoints());
    EXPECT_EQ(3, topology.triangleContainer->getNbEdges());
    EXPECT_EQ(1, topology.triangleContainer->getNbTriangles());

    //check adjacency information is computed
    {
        const TriangleSetTopologyContainer* constTriangleContainer = topology.triangleContainer.get();

        const auto eitArray = constTriangleContainer->getEdgesInTriangleArray();
        ASSERT_EQ(1, eitArray.size() );
        EXPECT_EQ(EdgeSetTopologyContainer::EdgeID(0), eitArray[0][0]);
        EXPECT_EQ(EdgeSetTopologyContainer::EdgeID(1), eitArray[0][1]);
        EXPECT_EQ(EdgeSetTopologyContainer::EdgeID(2), eitArray[0][2]);

        const auto eavArray = constTriangleContainer->getEdgesAroundVertexArray();
        ASSERT_EQ(3, eavArray.size());

        for (std::size_t i = 0; i < 3; ++i)
        {
            const auto& eavi = constTriangleContainer->getEdgesAroundVertex(EdgeSetTopologyContainer::PointID(i));
            ASSERT_EQ(2, eavi.size());

            sofa::helper::vector< EdgeSetTopologyContainer::EdgeID > eav_expected;

            std::cout << "Point: " << i << " eav: " << eavi << std::endl;
            // point 0 : edges 0,2 
            // point 1 : egdes 1,2
            // point 2 : edges 0,1

            for (std::size_t j = 0; j < 2; ++j)
            {
                eav_expected.push_back(EdgeSetTopologyContainer::EdgeID((i + (j + 1)) % 3));
            }
            std::sort(eav_expected.begin(), eav_expected.end());

            for (std::size_t j = 0; j < 2; ++j)
            {
                EXPECT_EQ(eav_expected[j], eavi[j]);
            }
        }

        const auto tavArray = constTriangleContainer->getTrianglesAroundVertexArray();
        ASSERT_EQ(3, tavArray.size());

        for (std::size_t i = 0; i < 3; ++i)
        {
            const auto& tavi = constTriangleContainer->getTrianglesAroundVertex(EdgeSetTopologyContainer::PointID(i));
            ASSERT_EQ(1, tavi.size());
            EXPECT_EQ(EdgeSetTopologyContainer::TriangleID(0), tavi[0]);
        }

        const auto taeArray = constTriangleContainer->getTrianglesAroundEdgeArray();
        ASSERT_EQ(3, taeArray.size());

        for (std::size_t i = 0; i < 3; ++i)
        {
            const auto& taei = constTriangleContainer->getTrianglesAroundEdge(EdgeSetTopologyContainer::EdgeID(i));
            ASSERT_EQ(1, taei.size());
            EXPECT_EQ(EdgeSetTopologyContainer::TriangleID(0), taei[0]);
        }


        const auto eitArrayExpected = topology.triangleContainer->getEdgesInTriangleArray();
        EXPECT_EQ(eitArrayExpected.size(), eitArray.size());
        EXPECT_TRUE(std::equal(eitArray.begin(), eitArray.end(), eitArrayExpected.begin(), compareEdgesInTriangle));

        const auto taeArrayExpected = topology.triangleContainer->getTrianglesAroundEdgeArray();
        EXPECT_EQ(taeArrayExpected.size(), taeArray.size());
        EXPECT_TRUE(std::equal(taeArray.begin(), taeArray.end(), taeArrayExpected.begin()));

        const auto eavArrayExpected = topology.triangleContainer->getEdgesAroundVertexArray();
        EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
        EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));

        const auto tavArrayExpected = topology.triangleContainer->getTrianglesAroundVertexArray();
        EXPECT_EQ(tavArrayExpected.size(), tavArray.size());
        EXPECT_TRUE(std::equal(tavArray.begin(), tavArray.end(), tavArrayExpected.begin()));

    }
}

TEST(TriangleSetTopology_test, checkTriangleSetTopologyRemoveTriangleKeepPointsKeepEdges)
{
    TriangleSetTopology topology;
    topology.init();
    topology.addOneTriangle();
    topology.removeLastTriangle(false, false);

    EXPECT_EQ(3, topology.triangleContainer->getNbPoints());
    EXPECT_EQ(3, topology.triangleContainer->getNbEdges());
    EXPECT_EQ(0, topology.triangleContainer->getNbTriangles());

    const TriangleSetTopologyContainer* constTriangleContainer = topology.triangleContainer.get();

    const auto eitArray = constTriangleContainer->getEdgesInTriangleArray();
    EXPECT_EQ(0, eitArray.size());

    const auto eavArray = constTriangleContainer->getEdgesAroundVertexArray();
    ASSERT_EQ(3, eavArray.size());

    for (std::size_t i = 0; i < 3; ++i)
    {
        const auto& eavi = constTriangleContainer->getEdgesAroundVertex(EdgeSetTopologyContainer::PointID(i));
        ASSERT_EQ(2, eavi.size());

        sofa::helper::vector< EdgeSetTopologyContainer::EdgeID > eav_expected;

        std::cout << "Point: " << i << " eav: " << eavi << std::endl;
        // point 0 : edges 0,2 
        // point 1 : egdes 1,2
        // point 2 : edges 0,1

        for (std::size_t j = 0; j < 2; ++j)
        {
            eav_expected.push_back(EdgeSetTopologyContainer::EdgeID((i + (j + 1)) % 3));
        }
        std::sort(eav_expected.begin(), eav_expected.end());

        for (std::size_t j = 0; j < 2; ++j)
        {
            EXPECT_EQ(eav_expected[j], eavi[j]);
        }
    }

    const auto tavArray = constTriangleContainer->getTrianglesAroundVertexArray();
    ASSERT_EQ(3, tavArray.size());

    for (std::size_t i = 0; i < 3; ++i)
    {
        const auto& tavi = constTriangleContainer->getTrianglesAroundVertex(EdgeSetTopologyContainer::PointID(i));
        EXPECT_EQ(0, tavi.size());
    }

    const auto taeArray = constTriangleContainer->getTrianglesAroundEdgeArray();
    ASSERT_EQ(3, taeArray.size());

    for (std::size_t i = 0; i < 3; ++i)
    {
        const auto& taei = constTriangleContainer->getTrianglesAroundEdge(EdgeSetTopologyContainer::EdgeID(i));
        EXPECT_EQ(0, taei.size());
    }

    const auto eitArrayExpected = topology.triangleContainer->getEdgesInTriangleArray();
    EXPECT_EQ(eitArrayExpected.size(), eitArray.size());
    EXPECT_TRUE(std::equal(eitArray.begin(), eitArray.end(), eitArrayExpected.begin(), compareEdgesInTriangle));

    const auto taeArrayExpected = topology.triangleContainer->getTrianglesAroundEdgeArray();
    EXPECT_EQ(taeArrayExpected.size(), taeArray.size());
    EXPECT_TRUE(std::equal(taeArray.begin(), taeArray.end(), taeArrayExpected.begin()));

    const auto eavArrayExpected = topology.triangleContainer->getEdgesAroundVertexArray();
    EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
    EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));

    const auto tavArrayExpected = topology.triangleContainer->getTrianglesAroundVertexArray();
    EXPECT_EQ(tavArrayExpected.size(), tavArray.size());
    EXPECT_TRUE(std::equal(tavArray.begin(), tavArray.end(), tavArrayExpected.begin()));

}

TEST(TriangleSetTopology_test, checkTriangleSetTopologyRemoveTriangleKeepPointsRemoveEdges)
{
    TriangleSetTopology topology;
    topology.init();
    topology.addOneTriangle();
    topology.removeLastTriangle(true, false);

    EXPECT_EQ(3, topology.triangleContainer->getNbPoints());
    EXPECT_EQ(0, topology.triangleContainer->getNbEdges());
    EXPECT_EQ(0, topology.triangleContainer->getNbTriangles());

    const TriangleSetTopologyContainer* constTriangleContainer = topology.triangleContainer.get();

    const auto eitArray = constTriangleContainer->getEdgesInTriangleArray();
    EXPECT_EQ(0, eitArray.size());
    
    const auto taeArray = constTriangleContainer->getTrianglesAroundEdgeArray();
    EXPECT_EQ(0, taeArray.size());

    const auto eavArray = constTriangleContainer->getEdgesAroundVertexArray();
    ASSERT_EQ(3, eavArray.size());

    for (std::size_t i = 0; i < 3; ++i)
    {
        const auto& eavi = constTriangleContainer->getEdgesAroundVertex(EdgeSetTopologyContainer::PointID(i));
        EXPECT_EQ(0, eavi.size());
    }

    const auto& tavArray = constTriangleContainer->getTrianglesAroundVertexArray();
    ASSERT_EQ(3, tavArray.size());

    for (std::size_t i = 0; i < 3; ++i)
    {
        const auto& tavi = constTriangleContainer->getTrianglesAroundVertex(EdgeSetTopologyContainer::PointID(i));
        EXPECT_EQ(0, tavi.size());
    }

    const auto eitArrayExpected = topology.triangleContainer->getEdgesInTriangleArray();
    EXPECT_EQ(eitArrayExpected.size(), eitArray.size());
    EXPECT_TRUE(std::equal(eitArray.begin(), eitArray.end(), eitArrayExpected.begin(), compareEdgesInTriangle ));

    const auto taeArrayExpected = topology.triangleContainer->getTrianglesAroundEdgeArray();
    EXPECT_EQ(taeArrayExpected.size(), taeArray.size());
    EXPECT_TRUE(std::equal(taeArray.begin(), taeArray.end(), taeArrayExpected.begin()));

    const auto eavArrayExpected = topology.triangleContainer->getEdgesAroundVertexArray();
    EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
    EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));

    const auto tavArrayExpected = topology.triangleContainer->getTrianglesAroundVertexArray();
    EXPECT_EQ(tavArrayExpected.size(), tavArray.size());
    EXPECT_TRUE(std::equal(tavArray.begin(), tavArray.end(), tavArrayExpected.begin()));
}

TEST(TriangleSetTopology_test, checkTriangleSetTopologyRemoveTriangleRemovePointsRemoveEdges)
{
    TriangleSetTopology topology;
    topology.init();
    topology.addOneTriangle();
    topology.removeLastTriangle(true, true);

    EXPECT_EQ(0, topology.triangleContainer->getNbPoints());
    EXPECT_EQ(0, topology.triangleContainer->getNbEdges());
    EXPECT_EQ(0, topology.triangleContainer->getNbTriangles());

    const TriangleSetTopologyContainer* constTriangleContainer = topology.triangleContainer.get();

    const auto& eitArray = constTriangleContainer->getEdgesInTriangleArray();
    EXPECT_EQ(0, eitArray.size());

    const auto taeArray = constTriangleContainer->getTrianglesAroundEdgeArray();
    EXPECT_EQ(0, taeArray.size());

    const auto eavArray = constTriangleContainer->getEdgesAroundVertexArray();
    EXPECT_EQ(0, eavArray.size());

    const auto tavArray = constTriangleContainer->getTrianglesAroundVertexArray();
    EXPECT_EQ(0, tavArray.size());

    const auto eitArrayExpected = topology.triangleContainer->getEdgesInTriangleArray();
    EXPECT_EQ(eitArrayExpected.size(), eitArray.size());
    EXPECT_TRUE(std::equal(eitArray.begin(), eitArray.end(), eitArrayExpected.begin(), compareEdgesInTriangle));

    const auto taeArrayExpected = topology.triangleContainer->getTrianglesAroundEdgeArray();
    EXPECT_EQ(taeArrayExpected.size(), taeArray.size());
    EXPECT_TRUE(std::equal(taeArray.begin(), taeArray.end(), taeArrayExpected.begin()));

    const auto eavArrayExpected = topology.triangleContainer->getEdgesAroundVertexArray();
    EXPECT_EQ(eavArrayExpected.size(), eavArray.size());
    EXPECT_TRUE(std::equal(eavArray.begin(), eavArray.end(), eavArrayExpected.begin()));

    const auto tavArrayExpected = topology.triangleContainer->getTrianglesAroundVertexArray();
    EXPECT_EQ(tavArrayExpected.size(), tavArray.size());
    EXPECT_TRUE(std::equal(tavArray.begin(), tavArray.end(), tavArrayExpected.begin()));

}


}
