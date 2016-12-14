#include <gtest/gtest.h>
#include <SofaTopologyMapping/Mesh2PointTopologicalMapping.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyModifier.h>
#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaSimulationTree/GNode.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/helper/vector.h>

using namespace sofa::component::topology;
using namespace sofa::simulation;
using namespace sofa::simulation::tree;
using namespace sofa::defaulttype;

namespace
{

template< class DataTypes >
void setTrianglePosition(const typename DataTypes::Coord& p0, const typename DataTypes::Coord& p1, const typename DataTypes::Coord& p2, 
                          sofa::component::container::MechanicalObject<DataTypes>* mecaObj)
{
    auto x = mecaObj->readPositions();
    if (x.size() >= 3)
    {
        x[0] = p0;
        x[1] = p1;
        x[2] = p2;
    }
}


struct TriangleMesh2PointTopologicalMapping_test : public ::testing::Test
{
    typedef sofa::component::container::MechanicalObject<sofa::defaulttype::Vec3Types> MechanicalObject3d;
    typedef sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types> TriangleSetGeometryAlgorithm3d;
    typedef sofa::defaulttype::Vec3d Vec3d;
protected:

    void createScene();

    void addOneTriangleFrom();

    void addPointInMiddleOfLastTriangle();

    void addRefinedTriangles();

    void removeCoarseTriangle();

    void duplicateLastPoint();

    void initScene();

    Node::SPtr m_nodeFrom;
    Node::SPtr m_nodeTo;
    TriangleSetTopologyContainer::SPtr   m_triangleContainerFrom;
    TriangleSetTopologyContainer::SPtr   m_triangleContainerTo;
    TriangleSetTopologyModifier::SPtr    m_triangleModifierFrom;
    TriangleSetTopologyModifier::SPtr    m_triangleModifierTo;
    Mesh2PointTopologicalMapping::SPtr   m_mesh2PointTopologicalMapping;
    MechanicalObject3d::SPtr             m_mechanicalObjectFrom;
    MechanicalObject3d::SPtr             m_mechanicalObjectTo;
    TriangleSetGeometryAlgorithm3d::SPtr m_triangleAlgoFrom;
    TriangleSetGeometryAlgorithm3d::SPtr m_triangleAlgoTo;
};

void TriangleMesh2PointTopologicalMapping_test::createScene()
{
    m_nodeFrom = sofa::core::objectmodel::New< GNode >(std::string(""));
    m_triangleContainerFrom = sofa::core::objectmodel::New< TriangleSetTopologyContainer >();
    m_triangleModifierFrom  = sofa::core::objectmodel::New< TriangleSetTopologyModifier >();
    m_mechanicalObjectFrom  = sofa::core::objectmodel::New< MechanicalObject3d >();
    m_mechanicalObjectFrom->resize(0);
    m_triangleAlgoFrom = sofa::core::objectmodel::New< TriangleSetGeometryAlgorithm3d >();

    m_nodeFrom->addObject(m_triangleContainerFrom);
    m_nodeFrom->addObject(m_triangleModifierFrom);
    m_nodeFrom->addObject(m_mechanicalObjectFrom);
    m_nodeFrom->addObject(m_triangleAlgoFrom);

    m_nodeTo = sofa::core::objectmodel::New< GNode >(std::string(""));
    m_triangleContainerTo = sofa::core::objectmodel::New< TriangleSetTopologyContainer >();
    m_triangleModifierTo = sofa::core::objectmodel::New< TriangleSetTopologyModifier >();
    m_mesh2PointTopologicalMapping = sofa::core::objectmodel::New< Mesh2PointTopologicalMapping >();
    m_mechanicalObjectTo = sofa::core::objectmodel::New< MechanicalObject3d >();
    m_mechanicalObjectTo->resize(0);
    m_triangleAlgoTo = sofa::core::objectmodel::New< TriangleSetGeometryAlgorithm3d >();

    m_nodeTo->addObject(m_triangleContainerTo);
    m_nodeTo->addObject(m_triangleModifierTo);
    m_nodeTo->addObject(m_mechanicalObjectTo);
    m_nodeTo->addObject(m_triangleAlgoTo);
    m_nodeTo->addObject(m_mesh2PointTopologicalMapping);

    sofa::helper::vector<Vec3d> pointBaryCoords  { Vec3d(0,0,0)   };
    sofa::helper::vector<Vec3d> edgeBaryCoords   { Vec3d(0.5,0,0) };
    
    m_mesh2PointTopologicalMapping->setTopologies(m_triangleContainerFrom.get(), m_triangleContainerTo.get());
    m_mesh2PointTopologicalMapping->setPointBaryCoords(pointBaryCoords);
    m_mesh2PointTopologicalMapping->setEdgeBaryCoords(edgeBaryCoords);
    m_mesh2PointTopologicalMapping->setCopyEdges(true);
    m_mesh2PointTopologicalMapping->setCopyTriangles(true);

    m_nodeFrom->addChild(m_nodeTo);
}

void TriangleMesh2PointTopologicalMapping_test::initScene()
{

    m_nodeFrom->init(sofa::core::ExecParams::defaultInstance());
    //m_nodeTo->init(sofa::core::ExecParams::defaultInstance());
}

void TriangleMesh2PointTopologicalMapping_test::addOneTriangleFrom()
{
    TriangleSetTopologyContainer::SeqTriangles trianglesToAdd;
    trianglesToAdd.push_back(TriangleSetTopologyContainer::Triangle(m_triangleContainerFrom->getNbPoints(),
        m_triangleContainerFrom->getNbPoints() + 1,
        m_triangleContainerFrom->getNbPoints() + 2));

    sofa::helper::vector<sofa::core::topology::PointAncestorElem> ancestors;
    ancestors.resize(3);
    ancestors[0].localCoords = Vec3d(0, 0, 0);
    ancestors[1].localCoords = Vec3d(1, 0, 0);
    ancestors[2].localCoords = Vec3d(0, 1, 0);

    m_triangleModifierFrom->addPointsProcess(3);
    m_triangleModifierFrom->addPointsWarning(3, ancestors);
    m_triangleModifierFrom->propagateTopologicalChanges();

    m_triangleModifierFrom->addTriangles(trianglesToAdd);
}

void TriangleMesh2PointTopologicalMapping_test::addPointInMiddleOfLastTriangle()
{
    sofa::helper::vector<sofa::core::topology::PointAncestorElem> ancestors;
    {
        sofa::core::topology::PointAncestorElem pointAncestor;
        pointAncestor.type  = sofa::core::topology::TRIANGLE;
        pointAncestor.index = m_triangleContainerFrom->getNbTriangles() - 1;
        pointAncestor.localCoords = sofa::core::topology::PointAncestorElem::LocalCoords(0.33, 0.33, 0.0);
        ancestors.push_back(pointAncestor);
    }

    m_triangleModifierFrom->addPointsProcess(1);
    m_triangleModifierFrom->addPointsWarning(1, ancestors);
    m_triangleModifierFrom->propagateTopologicalChanges();
}

void TriangleMesh2PointTopologicalMapping_test::addRefinedTriangles()
{
    TriangleSetTopologyContainer::SeqTriangles trianglesToAdd;
    trianglesToAdd.push_back(TriangleSetTopologyContainer::Triangle(0, 3, 1));
    trianglesToAdd.push_back(TriangleSetTopologyContainer::Triangle(1, 3, 2));
    trianglesToAdd.push_back(TriangleSetTopologyContainer::Triangle(2, 3, 0));

    m_triangleModifierFrom->addTriangles(trianglesToAdd);
}

void TriangleMesh2PointTopologicalMapping_test::removeCoarseTriangle()
{
    sofa::helper::vector< sofa::core::topology::Topology::TriangleID > triangleToRemove;
    triangleToRemove.push_back(0);
    m_triangleModifierFrom->removeTriangles(triangleToRemove,true,true);
}

void TriangleMesh2PointTopologicalMapping_test::duplicateLastPoint()
{
    sofa::helper::vector<sofa::core::topology::PointAncestorElem> ancestors;
    {
        sofa::core::topology::PointAncestorElem pointAncestor;
        pointAncestor.type = sofa::core::topology::POINT;
        pointAncestor.index = m_triangleContainerFrom->getNbPoints() - 1;
        ancestors.push_back(pointAncestor);
    }

    m_triangleModifierFrom->addPointsProcess(1);
    m_triangleModifierFrom->addPointsWarning(1, ancestors, false);
    m_triangleModifierFrom->propagateTopologicalChanges();
}

TEST_F(TriangleMesh2PointTopologicalMapping_test, checkInitializationIsOK)
{
    createScene();
    initScene();
    addOneTriangleFrom();

    EXPECT_EQ(3, m_triangleContainerFrom->getNbPoints());
    EXPECT_EQ(3, m_triangleContainerFrom->getNbEdges());
    EXPECT_EQ(1, m_triangleContainerFrom->getNbTriangles());

    auto readPosFrom = m_mechanicalObjectFrom->readPositions();
    ASSERT_EQ(3, m_mechanicalObjectFrom->getSize());
    EXPECT_EQ(Vec3d(0, 0, 0), readPosFrom[0]);
    EXPECT_EQ(Vec3d(1, 0, 0), readPosFrom[1]);
    EXPECT_EQ(Vec3d(0, 1, 0), readPosFrom[2]);

    EXPECT_EQ(3+3, m_triangleContainerTo->getNbPoints());
    ASSERT_EQ(3, m_triangleContainerTo->getNbEdges());
    EXPECT_EQ(1, m_triangleContainerTo->getNbTriangles());

    ASSERT_EQ(3 + 3, m_mechanicalObjectTo->getSize());
    auto readPosTo = m_mechanicalObjectTo->readPositions();
    EXPECT_EQ(Vec3d(0, 0, 0), readPosTo[0]);
    EXPECT_EQ(Vec3d(1, 0, 0), readPosTo[1]);
    EXPECT_EQ(Vec3d(0, 1, 0), readPosTo[2]);

    for (int eid = 0; eid < m_triangleContainerTo->getNbEdges(); ++eid)
    {
        sofa::core::topology::Edge e = m_triangleContainerTo->getEdge(eid);
        const Vec3d edgeCenterPos = (readPosTo[e[0]] + readPosTo[e[1]])*0.5;
        EXPECT_EQ(edgeCenterPos, readPosTo[3 + eid]);
    }
}


TEST_F(TriangleMesh2PointTopologicalMapping_test, checkAddPointInMiddleOfTriangle)
{
    createScene();
    initScene();
    addOneTriangleFrom();
    addPointInMiddleOfLastTriangle();

    ASSERT_EQ(4, m_triangleContainerFrom->getNbPoints());
    EXPECT_EQ(3, m_triangleContainerFrom->getNbEdges());
    ASSERT_EQ(1, m_triangleContainerFrom->getNbTriangles());

    ASSERT_EQ(4 + 3, m_triangleContainerTo->getNbPoints());
    EXPECT_EQ(3, m_triangleContainerTo->getNbEdges());
    ASSERT_EQ(1, m_triangleContainerTo->getNbTriangles());

    
    auto readPosFrom = m_mechanicalObjectFrom->readPositions();
    auto readPosTo   = m_mechanicalObjectTo->readPositions();
    sofa::core::topology::Triangle t = m_triangleContainerFrom->getTriangle(0);
    
    const Vec3d triangleCenterPos = ((1 - 0.33)*readPosFrom[t[0]] + (0.33)*readPosFrom[t[1]] + (0.33)*readPosFrom[t[2]]);

    EXPECT_EQ(triangleCenterPos, readPosFrom.ref().back());
    EXPECT_EQ(triangleCenterPos, readPosTo.ref().back());
}


TEST_F(TriangleMesh2PointTopologicalMapping_test, checkTriangleRefinement)
{
    createScene();
    initScene();
    addOneTriangleFrom();
    addPointInMiddleOfLastTriangle();
    addRefinedTriangles();

    EXPECT_EQ(4, m_triangleContainerFrom->getNbPoints());
    EXPECT_EQ(6, m_triangleContainerFrom->getNbEdges());
    EXPECT_EQ(4, m_triangleContainerFrom->getNbTriangles());

    EXPECT_EQ(4 + 6, m_triangleContainerTo->getNbPoints());
    EXPECT_EQ(6, m_triangleContainerTo->getNbEdges());
    EXPECT_EQ(4, m_triangleContainerTo->getNbTriangles());
}


TEST_F(TriangleMesh2PointTopologicalMapping_test, checkAddPointInMiddleOfLastRefinedTriangle)
{
    // createScene();
    // initScene();
    // addOneTriangleFrom();
    // addPointInMiddleOfLastTriangle();
    // addRefinedTriangles();
    // addPointInMiddleOfLastTriangle();

    // ASSERT_EQ(5, m_triangleContainerFrom->getNbPoints());
    // ASSERT_EQ(5 + 6, m_triangleContainerTo->getNbPoints());

    // auto readPosFrom = m_mechanicalObjectFrom->readPositions();
    // auto readPosTo   = m_mechanicalObjectTo->readPositions();

    // EXPECT_EQ(readPosFrom.ref().back(), readPosTo.ref().back());
}


TEST_F(TriangleMesh2PointTopologicalMapping_test, checkCoarseTriangleRemoval)
{
    createScene();
    initScene();
    addOneTriangleFrom();
    addPointInMiddleOfLastTriangle();
    addRefinedTriangles();
    removeCoarseTriangle();

    EXPECT_EQ(4, m_triangleContainerFrom->getNbPoints());
    EXPECT_EQ(6, m_triangleContainerFrom->getNbEdges());
    EXPECT_EQ(3, m_triangleContainerFrom->getNbTriangles());

    EXPECT_EQ(4 + 6, m_triangleContainerTo->getNbPoints());
    EXPECT_EQ(6, m_triangleContainerTo->getNbEdges());
    EXPECT_EQ(3, m_triangleContainerTo->getNbTriangles());
}

TEST_F(TriangleMesh2PointTopologicalMapping_test, checkDuplicateLastPoint)
{
    createScene();
    initScene();
    addOneTriangleFrom();
    addPointInMiddleOfLastTriangle();
    duplicateLastPoint();

    ASSERT_EQ(5, m_triangleContainerFrom->getNbPoints());
    ASSERT_EQ(5 + 3, m_triangleContainerTo->getNbPoints());

    auto readPosFrom = m_mechanicalObjectFrom->readPositions();
    auto readPosTo   = m_mechanicalObjectTo->readPositions();
    EXPECT_EQ(readPosFrom.ref().back(), readPosTo.ref().back());
}

}


