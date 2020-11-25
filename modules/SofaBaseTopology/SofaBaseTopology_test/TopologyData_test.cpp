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
#ifndef TopologyData_TEST_CPP
#define TopologyData_TEST_CPP

#include "TopologyData_test.h"
#include <sofa/core/ObjectFactory.h>



SOFA_DECL_CLASS(TopologyData_test)
int TopologyData_testClass = sofa::core::RegisterObject("Test Map Data ")
.add< TopologyData_test >()
;


TopologyData_test::TopologyData_test():
triangleInfo(initData(&triangleInfo, "triangleInfo", "Example of TriangleMapData"))
, edgeInfo(initData(&edgeInfo, "edgeInfo", "Example of EdgeMapData"))
, pointInfo(initData(&pointInfo, "pointInfo", "Example of PointMapData"))
{
    this->f_listening.setValue(true);
    tih = new TriangleInfoHandler(this, &triangleInfo);
    eih = new EdgeInfoHandler(this, &edgeInfo);
    pih = new PointInfoHandler(this, &pointInfo);
}

TopologyData_test::~TopologyData_test()
{
    if (tih) delete tih;
    if (eih) delete eih;
    if (pih) delete pih;
}

void TopologyData_test::init()
{
    if (m_topology)
    {
        triangleInfo.createTopologicalEngine(m_topology, tih);
        triangleInfo.registerTopologicalData();

        edgeInfo.createTopologicalEngine(m_topology, eih);
        edgeInfo.registerTopologicalData();

        pointInfo.createTopologicalEngine(m_topology, pih);
        pointInfo.registerTopologicalData();

    }
    else
    {
        std::cout << "Topology not found !! " << std::endl;
    }
}

///////////////////////////////// Handler Methods //////////////////////////////////////

void TopologyData_test::TriangleInfoHandler::applyCreateFunction(unsigned int /*triangleIndex*/, TestStruct& ts,
    const Triangle & /*t*/,
    const sofa::helper::vector< unsigned int >& ancestors,
    const sofa::helper::vector< double >& /*coeffs*/)
{
    if (!ancestors.empty())
    {
        const MapTestStruct& data = obj->triangleInfo.getValue();
        MapTestStruct::const_iterator it = data.find(ancestors[0]);
        if (it != data.end())
        {
            const TestStruct& dataAncestor = (*it).second;
            ts.id = dataAncestor.id;
            ts.baryCoord[0] = dataAncestor.baryCoord[0];
            ts.baryCoord[1] = dataAncestor.baryCoord[1];
        }
    }
}

void TopologyData_test::EdgeInfoHandler::applyCreateFunction(unsigned int /*triangleIndex*/, TestStruct& ts,
    const Edge & /*e*/,
    const sofa::helper::vector< unsigned int >& ancestors,
    const sofa::helper::vector< double >& /*coeffs*/)
{
    if (!ancestors.empty())
    {
        const MapTestStruct& data = obj->triangleInfo.getValue();
        MapTestStruct::const_iterator it = data.find(ancestors[0]);
        if (it != data.end())
        {
            const TestStruct& dataAncestor = (*it).second;
            ts.id = dataAncestor.id;
            ts.baryCoord[0] = dataAncestor.baryCoord[0];
            ts.baryCoord[1] = dataAncestor.baryCoord[1];
        }
    }
}

void TopologyData_test::PointInfoHandler::applyCreateFunction(unsigned int /*triangleIndex*/, TestStruct& ts,
    const Point & /*p*/,
    const sofa::helper::vector< unsigned int >& ancestors,
    const sofa::helper::vector< double >& /*coeffs*/)
{
    if (!ancestors.empty())
    {
        const MapTestStruct& data = obj->triangleInfo.getValue();
        MapTestStruct::const_iterator it = data.find(ancestors[0]);
        if (it != data.end())
        {
            const TestStruct& dataAncestor = (*it).second;
            ts.id = dataAncestor.id;
            ts.baryCoord[0] = dataAncestor.baryCoord[0];
            ts.baryCoord[1] = dataAncestor.baryCoord[1];
        }
    }
}

#endif //TopologyData_TEST_CPP


///////////////////////////////// Setup the scene ///////////////////////////////////////////////

void TopologyData_fixture::TearDown()
{
    sofa::simulation::getSimulation()->unload(rootNode);
}

void TopologyData_fixture::SetUp()
{
    sofa::simulation::tree::getSimulation();

    std::string position = "-2.927155 - 2.984353 0 - 2.927155 3.075896 0  2.979814 - 2.984353 0  2.979814 3.075896 0 - 2.927155 0.045772 0  2.979814 0.045772 0  0.026330 - 2.984353 0  0.026330 3.075896 0  0.026330 0.045772 0 - 2.927155 1.560834 0  2.979814 1.560834 0  1.503072 - 2.984353 0 - 1.450413 3.075896 0 - 2.927155 - 1.469290 0  2.979814 - 1.469290 0 - 1.450413 - 2.984353 0  1.503072 3.075896 0  1.503072 0.045772 0 - 1.450413 0.045772 0  0.026330 1.560834 0  0.026330 - 1.469290 0  1.503072 - 1.469290 0  1.503072 1.560834 0 - 1.450413 1.560834 0 - 1.450413 - 1.469290 0";
    unsigned int nbPoints = 25;
    //std::string points = "0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24";
    std::string triangles = "21 11 2  22 17 5  23 18 8  24 15 6  17 21 14  8 20 21  20 6 11  16 22 10  7 19 22  19 8 17  12 23 19  1 9 23  9 4 18  18 24 20  4 13 24  13 0 15  14 21 2  10 22 5  19 23 8  20 24 6  5 17 14  17 8 21  21 20 11  3 16 10  16 7 22  22 19 17  7 12 19  12 1 23  23 9 18  8 18 20  18 4 24  24 13 15";

    rootNode = sofa::core::objectmodel::New<sofa::simulation::tree::GNode>("root");
    container = sofa::core::objectmodel::New<TriangleSetTopologyContainer>(); 
    modifier  = sofa::core::objectmodel::New<TriangleSetTopologyModifier>(); 
    topoDataTest = sofa::core::objectmodel::New<TopologyData_test>(); 

    container->d_initPoints.read(position);
    container->setNbPoints(nbPoints);
    container->d_triangle.read(triangles);

    //Link TopoMapData with the topology
    topoDataTest->setTopology(container.get());

    rootNode->addObject(container);
    rootNode->addObject(modifier);
    rootNode->addObject(topoDataTest);

    sofa::simulation::getSimulation()->init(rootNode.get());
}


//////////////////////////////// TOPOLOGICAL CHANGES /////////////////////////////////////

void TopologyData_fixture::subdivideTriangles(const std::vector<SubdivideTriangleInfo>& pointsToAdd)
{
    if (pointsToAdd.empty())
        return;

    std::size_t nbPointsToAdd = pointsToAdd.size();
    unsigned int nbAddedPoints = 0;

    typedef sofa::core::topology::PointAncestorElem PointAncestorElem;

    sofa::helper::vector<Triangle> newTriangles; newTriangles.reserve(3 * nbPointsToAdd);
    sofa::helper::vector<PointAncestorElem> pointAncestors; pointAncestors.reserve(nbPointsToAdd);
    sofa::helper::vector< sofa::helper::vector< TriangleID > > ancestors; ancestors.reserve(3 * nbPointsToAdd);
    sofa::helper::vector< sofa::helper::vector< double > > coefs; coefs.reserve(3 * nbPointsToAdd);
    sofa::helper::vector< TriangleID > removeTriangleList; removeTriangleList.reserve(nbPointsToAdd);
    sofa::helper::vector< TriangleID > newTriangleID; newTriangleID.reserve(3 * nbPointsToAdd);

    for (const auto& pointToAdd : pointsToAdd)
    {
        pointAncestors.emplace_back(sofa::core::topology::TRIANGLE, pointToAdd.triangleId, PointAncestorElem::LocalCoords(pointToAdd.bary, 1 - pointToAdd.bary.sum()));

        const Triangle triangleSource = container->getTriangle(pointToAdd.triangleId);
        unsigned int newPointId = container->getNbPoints() + nbAddedPoints;
        newTriangles.emplace_back(triangleSource[0], triangleSource[1], newPointId);
        newTriangles.emplace_back(triangleSource[1], triangleSource[2], newPointId);
        newTriangles.emplace_back(triangleSource[2], triangleSource[0], newPointId);

        for (unsigned int i = 0; i < 3; i++)
        {
            ancestors.emplace_back(1, pointToAdd.triangleId);
            coefs.emplace_back(1, 1.0);
            newTriangleID.push_back(container->getNbTriangles() + newTriangleID.size());
        }
        removeTriangleList.push_back(pointToAdd.triangleId);
        nbAddedPoints++;
    }
    modifier->addPointsProcess(pointAncestors.size());
    modifier->addPointsWarning(pointAncestors.size(), pointAncestors);
    modifier->propagateTopologicalChanges();

    modifier->addRemoveTriangles(newTriangles.size(), newTriangles, newTriangleID, ancestors, coefs, removeTriangleList);
    modifier->notifyEndingEvent();
    modifier->propagateTopologicalChanges();

    //std::cout << "New triangles : " << newTriangleID << std::endl;
}

void TopologyData_fixture::removePoints(const std::vector<PointID>& pointsToRemove)
{
    if (pointsToRemove.empty())
        return;

    std::size_t nbPointsToRemove = pointsToRemove.size();
    sofa::helper::vector<Triangle> newTriangles; newTriangles.reserve(nbPointsToRemove);
    sofa::helper::vector< sofa::helper::vector< TriangleID > > ancestors; ancestors.reserve(nbPointsToRemove);
    sofa::helper::vector< sofa::helper::vector< double > > coefs; coefs.reserve(nbPointsToRemove);
    sofa::helper::vector< TriangleID > removeTriangleList; removeTriangleList.reserve(3 * nbPointsToRemove);
    sofa::helper::vector< TriangleID > newTriangleID; newTriangleID.reserve(nbPointsToRemove);

    // All triangles around each given point will be removed
    for (const PointID pointIdToRemove : pointsToRemove)
    {
        if (pointIdToRemove >= unsigned(container->getNbPoints())) return;
        const TrianglesAroundVertex& trianglesAroundVertex = container->getTrianglesAroundVertex(pointIdToRemove);
        ASSERT_TRUE(trianglesAroundVertex.size() == 3); // we are only handling the case of points which were created by the subdivideTriangles() method above

        sofa::helper::vector<PointID> verticesAroundVertex;
        verticesAroundVertex.reserve(trianglesAroundVertex.size());
        for (unsigned int t = 0; t < trianglesAroundVertex.size(); ++t)
        {
            removeTriangleList.push_back(trianglesAroundVertex[t]);
            const Triangle triangleSource = container->getTriangle(trianglesAroundVertex[t]);
            for (unsigned int p = 0; p < triangleSource.size(); ++p)
            {
                PointID pointSourceId = triangleSource[p];
                if (pointSourceId != pointIdToRemove && std::find(verticesAroundVertex.begin(), verticesAroundVertex.end(), pointSourceId) == verticesAroundVertex.end())
                {
                    verticesAroundVertex.push_back(pointSourceId);
                }
            }
        }
        ASSERT_TRUE(verticesAroundVertex.size() == 3); // we are only handling the case of points which were created by the subdivideTriangles() method above
        newTriangles.emplace_back(verticesAroundVertex[0], verticesAroundVertex[1], verticesAroundVertex[2]);
        ancestors.push_back(trianglesAroundVertex);

        {
            sofa::helper::vector< double > coefTriangle(trianglesAroundVertex.size());

            for (unsigned int t = 0; t < trianglesAroundVertex.size(); ++t)
            {
                coefTriangle[t] = 1.0/ container->getNbTriangles();
            }
            coefs.push_back(coefTriangle);
        }
        newTriangleID.push_back(container->getNbTriangles() + newTriangleID.size());
    }

    modifier->addRemoveTriangles(newTriangles.size(), newTriangles, newTriangleID, ancestors, coefs, removeTriangleList);
    modifier->notifyEndingEvent();
    modifier->propagateTopologicalChanges();

}



///////////////////////////////////// TESTS  //////////////////////////////////////////////

TEST_F(TopologyData_fixture, checkThatAllComponentsAreExisting)
{
    ASSERT_TRUE(this->container.get() != NULL);
    ASSERT_TRUE(this->modifier.get() != NULL);
    ASSERT_TRUE(this->topoDataTest.get() != NULL);
    ASSERT_TRUE(this->topoDataTest->triangleInfo.getTopologyHandler() != NULL);
    ASSERT_TRUE(this->topoDataTest->edgeInfo.getTopologyHandler() != NULL);
    ASSERT_TRUE(this->topoDataTest->pointInfo.getTopologyHandler() != NULL);
}

TEST_F(TopologyData_fixture, checkNumberOfTopologyElements)
{
    TriangleSetTopologyContainer* topology = container.get();
    EXPECT_EQ(topology->getNbPoints(), 25);
    EXPECT_EQ(topology->getNbEdges(), 56);
    EXPECT_EQ(topology->getNbTriangles(), 32);
}

TEST_F(TopologyData_fixture, makeSureTheDataAreWellSet)
{
    TopologyData_test* topoData = topoDataTest.get();

    unsigned int idOfElement = 3;
    unsigned int idToSet = 124502;
    double baryCoordToSet[2] = { 0.33, 0.33 };
    TopologyData_test::TestStruct ts;
    ts.id = idToSet;
    ts.baryCoord[0] = baryCoordToSet[0];
    ts.baryCoord[1] = baryCoordToSet[1];

    TopologyData_test::MapTestStruct& ti = *(topoData->triangleInfo.beginEdit());
    TopologyData_test::MapTestStruct& ei = *(topoData->edgeInfo.beginEdit());
    TopologyData_test::MapTestStruct& pi = *(topoData->pointInfo.beginEdit());

    ti[idOfElement] = ts;
    ei[idOfElement] = ts;
    pi[idOfElement] = ts;

    topoData->triangleInfo.endEdit();
    topoData->edgeInfo.endEdit();
    topoData->pointInfo.endEdit();

    const TopologyData_test::MapTestStruct& triangleInfoMap = topoData->triangleInfo.getValue();
    const TopologyData_test::MapTestStruct& edgeInfoMap = topoData->edgeInfo.getValue();
    const TopologyData_test::MapTestStruct& pointInfoMap = topoData->pointInfo.getValue();

    TopologyData_test::MapTestStruct::const_iterator newElemtTriangleIt = triangleInfoMap.find(idOfElement);
    TopologyData_test::MapTestStruct::const_iterator newElemtEdgeIt = edgeInfoMap.find(idOfElement);
    TopologyData_test::MapTestStruct::const_iterator newElemtPointIt = pointInfoMap.find(idOfElement);

    EXPECT_TRUE(newElemtTriangleIt != triangleInfoMap.end());
    EXPECT_TRUE(newElemtEdgeIt != edgeInfoMap.end());
    EXPECT_TRUE(newElemtPointIt != pointInfoMap.end());

    EXPECT_EQ(newElemtTriangleIt->second.id, idToSet);
    EXPECT_EQ(newElemtEdgeIt->second.id, idToSet);
    EXPECT_EQ(newElemtPointIt->second.id, idToSet);

}

TEST_F(TopologyData_fixture, makeSureAfterTopologyChangeTheInfosAreTheSame)
{
    TriangleSetTopologyContainer* topology = container.get();
    TopologyData_test* topoData = topoDataTest.get();

    unsigned int nbTriangles = topology->getNbTriangles();
    unsigned int nbPoints = topology->getNbPoints();
    unsigned int nbEdges = topology->getNbEdges();
    unsigned int triangleIdToSubdivide = std::rand() % nbTriangles;
    unsigned int idToKeepAfterTC = std::rand();
    double baryCoordToKeepAfterTC[2] = { 0.33, 0.33 };
    TopologyData_test::TestStruct ts;
    ts.id = idToKeepAfterTC;
    ts.baryCoord[0] = baryCoordToKeepAfterTC[0];
    ts.baryCoord[1] = baryCoordToKeepAfterTC[1];


    TopologyData_test::MapTestStruct& ti = *(topoData->triangleInfo.beginEdit());
    ti[triangleIdToSubdivide] = ts;
    topoData->triangleInfo.endEdit();

    //std::cout << " triangle data before TC : " << topoData->triangleInfo.getValue() << std::endl;

    SubdivideTriangleInfo sti;
    sti.triangleId = triangleIdToSubdivide;
    sti.bary = sofa::defaulttype::Vec<2, sofa::defaulttype::Vec3Types::Real>(0.33, 0.33);

    std::vector<SubdivideTriangleInfo> stiv;
    stiv.push_back(sti);

    subdivideTriangles(stiv);

    EXPECT_EQ(u_int(topology->getNbTriangles()), nbTriangles + 2);
    EXPECT_EQ(u_int(topology->getNbPoints()), nbPoints + 1);
    EXPECT_EQ(u_int(topology->getNbEdges()), nbEdges + 3);

    //std::cout << " triangle data after TC : " << topoData->triangleInfo.getValue() << std::endl;

    const TopologyData_test::MapTestStruct& triangleInfoMap = topoData->triangleInfo.getValue();
    TopologyData_test::MapTestStruct::const_iterator dataIt = triangleInfoMap.find(triangleIdToSubdivide);
    
    EXPECT_EQ(dataIt->second.id, idToKeepAfterTC);
    EXPECT_EQ(dataIt->second.baryCoord[0], baryCoordToKeepAfterTC[0]);
    EXPECT_EQ(dataIt->second.baryCoord[1], baryCoordToKeepAfterTC[1]);

    TopologyData_test::MapTestStruct::const_iterator dataIt0 = triangleInfoMap.find(nbTriangles);
    TopologyData_test::MapTestStruct::const_iterator dataIt1 = triangleInfoMap.find(nbTriangles+1);

    EXPECT_EQ(dataIt0->second.id, idToKeepAfterTC);
    EXPECT_EQ(dataIt0->second.baryCoord[0], baryCoordToKeepAfterTC[0]);
    EXPECT_EQ(dataIt0->second.baryCoord[1], baryCoordToKeepAfterTC[1]);
    EXPECT_EQ(dataIt1->second.id, idToKeepAfterTC);
    EXPECT_EQ(dataIt1->second.baryCoord[0], baryCoordToKeepAfterTC[0]);
    EXPECT_EQ(dataIt1->second.baryCoord[1], baryCoordToKeepAfterTC[1]);
}


TEST_F(TopologyData_fixture, makeSureAfterAddingAndDeletingPointTheTopologyStaysTheSame)
{
    TriangleSetTopologyContainer* topology = container.get();
    TopologyData_test* topoData = topoDataTest.get();

    unsigned int nbTriangles = topology->getNbTriangles();
    unsigned int nbPoints = topology->getNbPoints();
    unsigned int nbEdges = topology->getNbEdges();
    unsigned int triangleIdToSubdivide = std::rand() % nbTriangles;
    unsigned int idToKeepAfterTC = std::rand();
    double baryCoordToKeepAfterTC[2] = { 0.33, 0.33 };
    TopologyData_test::TestStruct ts;
    ts.id = idToKeepAfterTC;
    ts.baryCoord[0] = baryCoordToKeepAfterTC[0];
    ts.baryCoord[1] = baryCoordToKeepAfterTC[1];


    TopologyData_test::MapTestStruct& ti = *(topoData->triangleInfo.beginEdit());
    TopologyData_test::TestStruct& newElemtTriangleIt = ti[triangleIdToSubdivide];
    newElemtTriangleIt = ts;
    topoData->triangleInfo.endEdit();

    //std::cout << " triangle data before TC : " << topoData->triangleInfo.getValue() << std::endl;

    SubdivideTriangleInfo sti;
    sti.triangleId = triangleIdToSubdivide;
    sti.bary = sofa::defaulttype::Vec<2, sofa::defaulttype::Vec3Types::Real>(0.33, 0.33);

    std::vector<SubdivideTriangleInfo> stiv;
    stiv.push_back(sti);

    subdivideTriangles(stiv);

    EXPECT_EQ((unsigned int)topology->getNbTriangles(), nbTriangles + 2);
    EXPECT_EQ((unsigned int)topology->getNbPoints(), nbPoints + 1);
    EXPECT_EQ((unsigned int)topology->getNbEdges(), nbEdges + 3);

    std::vector<unsigned int> pointToRemove;
    pointToRemove.emplace_back(topology->getNbPoints()-1);
    removePoints(pointToRemove);

    EXPECT_EQ((unsigned int)topology->getNbTriangles(), nbTriangles);
    EXPECT_EQ((unsigned int)topology->getNbPoints(), nbPoints);
    EXPECT_EQ((unsigned int)topology->getNbEdges(), nbEdges);

    //std::cout << " triangle data after TC : " << topoData->triangleInfo.getValue() << std::endl;

    const TopologyData_test::MapTestStruct& triangleInfoMap = topoData->triangleInfo.getValue();
    TopologyData_test::MapTestStruct::const_iterator dataIt = triangleInfoMap.find(triangleIdToSubdivide);

    EXPECT_EQ(dataIt->second.id, idToKeepAfterTC);
    EXPECT_EQ(dataIt->second.baryCoord[0], baryCoordToKeepAfterTC[0]);
    EXPECT_EQ(dataIt->second.baryCoord[1], baryCoordToKeepAfterTC[1]);
}
