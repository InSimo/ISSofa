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
#include <SofaTopologyMapping/Mesh2PointTopologicalMapping.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/ObjectFactory.h>

#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyModifier.h>
#include <SofaBaseTopology/PointSetTopologyModifier.h>

#include <sofa/defaulttype/Vec.h>
#include <map>
#include <sofa/defaulttype/VecTypes.h>

#include <numeric>

namespace sofa
{
namespace component
{
namespace topology
{

SOFA_CLASS_IMPL((Mesh2PointTopologicalMapping));

using namespace sofa::defaulttype;
using namespace sofa::component::topology;
using namespace sofa::core::topology;

SOFA_DECL_CLASS ( Mesh2PointTopologicalMapping )

// Register in the Factory
int Mesh2PointTopologicalMappingClass = core::RegisterObject ( "This class maps any mesh primitive (point, edge, triangle...) into a point using a relative position from the primitive" )
        .add< Mesh2PointTopologicalMapping >()
        ;

// Implementation
Mesh2PointTopologicalMapping::Mesh2PointTopologicalMapping ()
    : pointBaryCoords ( initData ( &pointBaryCoords, "pointBaryCoords", "Coordinates for the points of the output topology created from the points of the input topology" ) ),
      edgeBaryCoords ( initData ( &edgeBaryCoords, "edgeBaryCoords", "Coordinates for the points of the output topology created from the edges of the input topology" ) ),
      triangleBaryCoords ( initData ( &triangleBaryCoords, "triangleBaryCoords", "Coordinates for the points of the output topology created from the triangles of the input topology" ) ),
      quadBaryCoords ( initData ( &quadBaryCoords, "quadBaryCoords", "Coordinates for the points of the output topology created from the quads of the input topology" ) ),
      tetraBaryCoords ( initData ( &tetraBaryCoords, "tetraBaryCoords", "Coordinates for the points of the output topology created from the tetra of the input topology" ) ),
      hexaBaryCoords ( initData ( &hexaBaryCoords, "hexaBaryCoords", "Coordinates for the points of the output topology created from the hexa of the input topology" ) ),
      copyEdges ( initData ( &copyEdges, false, "copyEdges", "Activate mapping of input edges into the output topology (requires at least one item in pointBaryCoords)" ) ),
      copyTriangles ( initData ( &copyTriangles, false, "copyTriangles", "Activate mapping of input triangles into the output topology (requires at least one item in pointBaryCoords)" ) ),
      copyTetrahedra ( initData ( &copyTetrahedra, false, "copyTetrahedra", "Activate mapping of input tetrahedra into the output topology (requires at least one item in pointBaryCoords)" ) ),
       initDone(false)
{
    pointBaryCoords.setGroup("BaryCoords");
    edgeBaryCoords.setGroup("BaryCoords");
    triangleBaryCoords.setGroup("BaryCoords");
    quadBaryCoords.setGroup("BaryCoords");
    tetraBaryCoords.setGroup("BaryCoords");
    hexaBaryCoords.setGroup("BaryCoords");
}

void Mesh2PointTopologicalMapping::init()
{
    initDone = true;
    if(fromModel)
    {
        if(toModel)
        {
            int toModelLastPointIndex = 0;
            toModel->clear();

            PointSetTopologyModifier *toPointMod = NULL;
            toModel->getContext()->get(toPointMod, sofa::core::objectmodel::BaseContext::Local);
            EdgeSetTopologyModifier *toEdgeMod = NULL;
            toModel->getContext()->get(toEdgeMod, sofa::core::objectmodel::BaseContext::Local);
            TriangleSetTopologyModifier *toTriangleMod = NULL;
            toModel->getContext()->get(toTriangleMod, sofa::core::objectmodel::BaseContext::Local);
			TetrahedronSetTopologyModifier *toTetrahedronMod = NULL;
            toModel->getContext()->get(toTetrahedronMod, sofa::core::objectmodel::BaseContext::Local);
            //QuadSetTopologyModifier *toQuadMod = NULL;
            //TetrahedronSetTopologyModifier *toTetrahedronMod = NULL;
            //HexahedronSetTopologyModifier *toHexahedronMod = NULL;


            if (copyEdges.getValue() && pointBaryCoords.getValue().empty())
            {
                serr << "copyEdges requires at least one item in pointBaryCoords" << sendl;
                copyEdges.setValue(false);
            }

            if (copyTriangles.getValue() && pointBaryCoords.getValue().empty())
            {
                serr << "copyTriangles requires at least one item in pointBaryCoords" << sendl;
                copyTriangles.setValue(false);
            }
           if (copyTetrahedra.getValue() && pointBaryCoords.getValue().empty())
            {
                serr << "copyTetrahedra requires at least one item in pointBaryCoords" << sendl;
                copyTetrahedra.setValue(false);
            }

            // point to point mapping
            if (!pointBaryCoords.getValue().empty())
            {
                int nbPointsToAdd = fromModel->getNbPoints();
                pointsMappedFrom[POINT].resize(nbPointsToAdd);

                sofa::helper::vector< unsigned int > indices(nbPointsToAdd);
                std::iota(std::begin(indices), std::end(indices), 0); // Fill with 0, 1, ..., nbPointsToAdd
                PointsAdded pointsToAdd(nbPointsToAdd, indices);

                addInputPoints(&pointsToAdd);
                toModelLastPointIndex += nbPointsToAdd * pointBaryCoords.getValue().size();
            }

            // edge to point mapping
            if (!edgeBaryCoords.getValue().empty())
            {
                int nbEdgesToAdd = fromModel->getNbEdges();
                pointsMappedFrom[EDGE].resize(fromModel->getNbEdges());

                sofa::helper::vector< unsigned int > indices(nbEdgesToAdd);
                std::iota(std::begin(indices), std::end(indices), 0); // Fill with 0, 1, ..., nbEdgesToAdd
                EdgesAdded edgesToAdd(nbEdgesToAdd, {}, indices);

                addInputEdges(&edgesToAdd);
            }

            // edge to edge identity mapping
            if (copyEdges.getValue())
            {
                sout << "Copying " << fromModel->getNbEdges() << " edges" << sendl;
                sofa::helper::vector<Edge> edgesToAdd;
                edgesToAdd.reserve(fromModel->getNbEdges());
                for (int i=0; i<fromModel->getNbEdges(); i++)
                {
                    Edge e = fromModel->getEdge(i);
                    for (unsigned int j=0; j<e.size(); ++j)
                        e[j] = pointsMappedFrom[POINT][e[j]][0];
                    if (toEdgeMod)
                    {
                        edgesToAdd.push_back(e);
                    }
                    else
                        toModel->addEdge(e[0],e[1]);
                }

                if (toEdgeMod)
                {
                    toEdgeMod->addEdgesProcess(edgesToAdd);
                }
            }


            // triangle to point mapping
            if (!triangleBaryCoords.getValue().empty())
            {
                int nbTrianglesToAdd = fromModel->getNbTriangles();
                pointsMappedFrom[TRIANGLE].resize(nbTrianglesToAdd);

                sofa::helper::vector< unsigned int > indices(nbTrianglesToAdd);
                std::iota(std::begin(indices), std::end(indices), 0); // Fill with 0, 1, ..., nbTrianglesToAdd
                TrianglesAdded trianglesToAdd(nbTrianglesToAdd, {}, indices);

                addInputTriangles(&trianglesToAdd);
            }

            // triangle to triangle identity mapping
            if (copyTriangles.getValue())
            {
                sout << "Copying " << fromModel->getNbTriangles() << " triangles" << sendl;
                sofa::helper::vector< Triangle > trianglesToAdd;
                trianglesToAdd.reserve(fromModel->getNbTriangles());
                for (int i=0; i<fromModel->getNbTriangles(); i++)
                {
                    Triangle t = fromModel->getTriangle(i);
                    for (unsigned int j=0; j<t.size(); ++j)
                        t[j] = pointsMappedFrom[POINT][t[j]][0];
                    if (toTriangleMod)
                    {
                        trianglesToAdd.push_back(t);
                    }
                    else
                        toModel->addTriangle(t[0],t[1],t[2]);
                }
                if (toTriangleMod)
                {
                    toTriangleMod->addTrianglesProcess(trianglesToAdd);
                }
            }

            // quad to point mapping
            if (!quadBaryCoords.getValue().empty())
            {
                pointsMappedFrom[QUAD].resize(fromModel->getNbQuads());
                for (int i=0; i<fromModel->getNbQuads(); i++)
                {
                    for (unsigned int j=0; j<quadBaryCoords.getValue().size(); j++)
                    {
                        Quad q = fromModel->getQuad(i);

                        Vec3d p0(fromModel->getPX(q[0]), fromModel->getPY(q[0]), fromModel->getPZ(q[0]));
                        Vec3d p1(fromModel->getPX(q[1]), fromModel->getPY(q[1]), fromModel->getPZ(q[1]));
                        Vec3d p2(fromModel->getPX(q[2]), fromModel->getPY(q[2]), fromModel->getPZ(q[2]));
                        Vec3d p3(fromModel->getPX(q[3]), fromModel->getPY(q[3]), fromModel->getPZ(q[3]));

                        double fx = quadBaryCoords.getValue()[j][0];
                        double fy = quadBaryCoords.getValue()[j][1];

                        Vec3d result =  p0 * ((1-fx) * (1-fy))
                                + p1 * ((  fx) * (1-fy))
                                + p2 * ((1-fx) * (  fy))
                                + p3 * ((  fx) * (  fy));

                        toModel->addPoint(result[0], result[1], result[2]);

                        pointsMappedFrom[QUAD][i].push_back(toModelLastPointIndex);
                        pointSource.push_back(std::make_pair(QUAD,i));
                        toModelLastPointIndex++;
                    }
                }
            }

            // tetrahedron to point mapping
            if (!tetraBaryCoords.getValue().empty())
            {
                int nbTetrahedraToAdd = fromModel->getNbTetrahedra();
                pointsMappedFrom[TETRA].resize(nbTetrahedraToAdd);

                sofa::helper::vector< unsigned int > indices(nbTetrahedraToAdd);
                std::iota(std::begin(indices), std::end(indices), 0); // Fill with 0, 1, ..., nbTetrahedrasToAdd
                TetrahedraAdded tetrahedraToAdd(nbTetrahedraToAdd, {}, indices);

                addInputTetrahedra(&tetrahedraToAdd);
            }
			// triangle to triangle identity mapping
            if (copyTetrahedra.getValue())
            {

                sout << "Copying " << fromModel->getNbTetrahedra() << " tetrahedra" << sendl;
                for (int i=0; i<fromModel->getNbTetrahedra(); i++)
                {
                    Tetrahedron t = fromModel->getTetrahedron(i);
                    for (unsigned int j=0; j<t.size(); ++j)
                        t[j] = pointsMappedFrom[POINT][t[j]][0];
                    if (toTetrahedronMod)
                        toTetrahedronMod->addTetrahedronProcess(t);
                    else
                        toModel->addTetra(t[0],t[1],t[2],t[3]);
                }
            }
            // hexahedron to point mapping
            if (!hexaBaryCoords.getValue().empty())
            {
                pointsMappedFrom[HEXA].resize(fromModel->getNbHexahedra());
                for (int i=0; i<fromModel->getNbHexahedra(); i++)
                {
                    for (unsigned int j=0; j<hexaBaryCoords.getValue().size(); j++)
                    {
                        Hexahedron h = fromModel->getHexahedron(i);

                        Vec3d p0(fromModel->getPX(h[0]), fromModel->getPY(h[0]), fromModel->getPZ(h[0]));
                        Vec3d p1(fromModel->getPX(h[1]), fromModel->getPY(h[1]), fromModel->getPZ(h[1]));
						Vec3d p2(fromModel->getPX(h[3]), fromModel->getPY(h[3]), fromModel->getPZ(h[3]));
						Vec3d p3(fromModel->getPX(h[2]), fromModel->getPY(h[2]), fromModel->getPZ(h[2]));
                        Vec3d p4(fromModel->getPX(h[4]), fromModel->getPY(h[4]), fromModel->getPZ(h[4]));
                        Vec3d p5(fromModel->getPX(h[5]), fromModel->getPY(h[5]), fromModel->getPZ(h[5]));
						Vec3d p6(fromModel->getPX(h[7]), fromModel->getPY(h[7]), fromModel->getPZ(h[7]));
						Vec3d p7(fromModel->getPX(h[6]), fromModel->getPY(h[6]), fromModel->getPZ(h[6]));

                        double fx = hexaBaryCoords.getValue()[j][0];
                        double fy = hexaBaryCoords.getValue()[j][1];
                        double fz = hexaBaryCoords.getValue()[j][2];

                        Vec3d result =  p0 * ((1-fx) * (1-fy) * (1-fz))
                                + p1 * ((  fx) * (1-fy) * (1-fz))
                                + p2 * ((1-fx) * (  fy) * (1-fz))
                                + p3 * ((  fx) * (  fy) * (1-fz))
                                + p4 * ((1-fx) * (1-fy) * (  fz))
                                + p5 * ((  fx) * (1-fy) * (  fz))
								+ p6 * ((1-fx) * (  fy) * (  fz))
								+ p7 * ((  fx) * (  fy) * (  fz));

                        toModel->addPoint(result[0], result[1], result[2]);

                        pointsMappedFrom[HEXA][i].push_back(toModelLastPointIndex);
                        pointSource.push_back(std::make_pair(HEXA,i));
                        toModelLastPointIndex++;
                    }
                }
            }
            internalCheck("init");
        }
    }
}

/// Check consistency of internal maps and output topology
bool Mesh2PointTopologicalMapping::internalCheck(const char* step, const helper::fixed_array <int, NB_ELEMENTS >& nbInputRemoved)
{
    bool ok = true;
    unsigned int nbPOut = (unsigned int)toModel->getNbPoints();
    if (nbPOut != pointSource.size())
    {
        serr << "Internal Error after " << step << ": pointSource size " << pointSource.size() << " != output topology size " << nbPOut << sendl;
        ok = false;
    }
    unsigned int nbPMapped = 0;
    for (int type=0; type<NB_ELEMENTS; ++type)
    {
        const vector< vector<int> >& pointsMapped = pointsMappedFrom[type];
        std::string typestr;
        unsigned int nbEIn = 0;
        unsigned int nbEPOut = 0;
        switch (type)
        {
        case POINT :    typestr="Point";    nbEIn = fromModel->getNbPoints();     nbEPOut = pointBaryCoords.getValue().size(); break;
        case EDGE :     typestr="Edge";     nbEIn = fromModel->getNbEdges();      nbEPOut = edgeBaryCoords.getValue().size(); break;
        case TRIANGLE : typestr="Triangle"; nbEIn = fromModel->getNbTriangles();  nbEPOut = triangleBaryCoords.getValue().size(); break;
        case QUAD :     typestr="Quad";     nbEIn = fromModel->getNbQuads();      nbEPOut = quadBaryCoords.getValue().size(); break;
        case TETRA :    typestr="Tetra";    nbEIn = fromModel->getNbTetrahedra(); nbEPOut = tetraBaryCoords.getValue().size(); break;
        case HEXA :     typestr="Hexa";     nbEIn = fromModel->getNbHexahedra();  nbEPOut = hexaBaryCoords.getValue().size(); break;
        default :       typestr="Unknown";  break;
        }
        nbEIn -= nbInputRemoved[type];
        if (pointsMapped.empty())
        {
            if (nbEIn && nbEPOut)
            {
                serr << "Internal Error after " << step << ": pointsMappedFrom" << typestr << " is empty while there should be " << nbEPOut << " generated points per input " << typestr << sendl;
                ok = false;
            }
            continue;
        }

        if (nbEIn != pointsMapped.size())
        {
            serr << "Internal Error after " << step << ": pointsMappedFrom" << typestr << " size " << pointsMapped.size() << " != input topology size " << nbEIn;
            if (nbInputRemoved[type]) serr << " (including " << nbInputRemoved[type] << " removed input elements)";
            serr << sendl;
            ok = false;
        }
        for (unsigned int es = 0; es < pointsMapped.size(); ++es)
        {
            if (pointsMapped[es].size() != nbEPOut)
            {
                serr << "Internal Error after " << step << ":     pointsMappedFrom" << typestr << "[" << es << "] size " << pointsMapped[es].size() << " != barycoords size " << nbEPOut << sendl;
                ok = false;
            }
            for (unsigned int j = 0; j < pointsMapped[es].size(); ++j)
            {
                if ((unsigned)pointsMapped[es][j] >= nbPOut)
                {
                    serr << "Internal Error after " << step << ":     pointsMappedFrom" << typestr << "[" << es << "][" << j << "] = " << pointsMapped[es][j] << " >= " << nbPOut << sendl;
                    ok = false;
                }
            }
        }
        nbPMapped += nbEIn * nbEPOut;
    }
    if (nbPOut != nbPMapped + pointsToRemove.size())
    {
        serr << "Internal Error after " << step << ": " << nbPOut << " mapped points + " << pointsToRemove.size() << " removed points != output topology size " << nbPOut << sendl;
        ok = false;
    }
    if (copyEdges.getValue())
    {
        if (fromModel->getNbEdges() - nbInputRemoved[EDGE] != toModel->getNbEdges())
        {
            serr << "Internal Error after " << step << ": edges were copied, yet output edges size " << toModel->getNbEdges() << " - " << nbInputRemoved[EDGE] << " != input edges size " << fromModel->getNbEdges();
            if (nbInputRemoved[EDGE]) serr << " - " << nbInputRemoved[EDGE];
            serr << sendl;
            ok = false;
        }
    }
    if (copyTriangles.getValue())
    {
        if (fromModel->getNbTriangles() - nbInputRemoved[TRIANGLE] != toModel->getNbTriangles())
        {
            serr << "Internal Error after " << step << ": triangles were copied, yet output triangles size " << toModel->getNbTriangles() << " != input triangles size " << fromModel->getNbTriangles();
            if (nbInputRemoved[TRIANGLE]) serr << " - " << nbInputRemoved[TRIANGLE];
            serr << sendl;
            ok = false;
        }
    }
	if (copyTetrahedra.getValue())
    {
        if (fromModel->getNbTetrahedra() - nbInputRemoved[TETRA] != toModel->getNbTetrahedra())
        {
            serr << "Internal Error after " << step << ": tetrahedra were copied, yet output tetrahedra size " << toModel->getNbTetrahedra() << " != input tetrahedra size " << fromModel->getNbTetrahedra();
            if (nbInputRemoved[TETRA]) serr << " - " << nbInputRemoved[TETRA];
            serr << sendl;
            ok = false;
        }
    }
    sout << "Internal check done after " << step << ", " << fromModel->getNbPoints();
    if (nbInputRemoved[POINT]) sout << " - " << nbInputRemoved[POINT];
    sout << " input points, " << nbPOut;
    if (pointsToRemove.size()) sout << " - " << pointsToRemove.size();
    sout << " generated points";
    if (copyEdges.getValue()) sout << ", " << toModel->getNbEdges() << " generated edges";
    if (copyTriangles.getValue()) sout << ", " << toModel->getNbTriangles() << " generated triangles";
    if (copyTetrahedra.getValue()) sout << ", " << toModel->getNbTetrahedra() << " generated tetrahedra";
    sout << "." << sendl;
    return ok;
}


void Mesh2PointTopologicalMapping::addInputPoints(const sofa::core::topology::PointsAdded * pAdd, PointSetTopologyModifier* toPointMod)
{
    const sofa::helper::vector<unsigned int>& pIdArray = pAdd->pointIndexArray;    
    const sofa::helper::vector< PointAncestorElem >& ancestorsElem = pAdd->ancestorElems;
    const vector< Vec3d > &pBaryCoords = pointBaryCoords.getValue();

    for (unsigned int pId : pIdArray)
    {
        sout << "INPUT ADD POINTS " << pId << sendl;
        if (pointsMappedFrom[POINT].size() < pId + 1)
        {
            pointsMappedFrom[POINT].resize(pId +1);
        }
        else
        {
            pointsMappedFrom[POINT][pId].clear();
        }
        
        for (unsigned int j = 0; j < pBaryCoords.size(); j++)
        {
            pointsMappedFrom[POINT][pId].push_back(pointSource.size());
            pointSource.push_back(std::make_pair(POINT, pId));

            if (!toPointMod)
            {
                toModel->addPoint(fromModel->getPX(pId) + pBaryCoords[j][0], fromModel->getPY(pId) + pBaryCoords[j][1], fromModel->getPZ(pId) + pBaryCoords[j][2]);
            }
		}
    }
    
    if (toPointMod)
    {
        sofa::helper::vector< PointAncestorElem > ancestors;
        ancestors.reserve(pIdArray.size()*pBaryCoords.size());

        for (unsigned int i = 0; i < pIdArray.size(); ++i)
        {
            for (unsigned int j = 0; j < pBaryCoords.size(); ++j)
            {
                PointAncestorElem pAncestor;
                if (i < ancestorsElem.size())
                {
                    if (ancestorsElem[i].type == sofa::core::topology::POINT)
                    {
                        pAncestor.type  = sofa::core::topology::POINT;
                        if (ancestorsElem[i].index != sofa::core::topology::Topology::InvalidID)
                        {
                            pAncestor.index = pointsMappedFrom[POINT][ancestorsElem[i].index][j];
                        }
                        pAncestor.localCoords = ( ancestorsElem[i].localCoords + pBaryCoords[j] );
                    }
                    else if (ancestorsElem[i].type == sofa::core::topology::EDGE &&
                             copyEdges.getValue())
                    {
                        pAncestor = ancestorsElem[i];
                    }
                    else if (ancestorsElem[i].type == sofa::core::topology::TRIANGLE &&
                             copyTriangles.getValue())
                    {
                        pAncestor = ancestorsElem[i];
                    }
                    else if (ancestorsElem[i].type == sofa::core::topology::TETRAHEDRON &&
                             copyTetrahedra.getValue())
                    {
                        pAncestor = ancestorsElem[i];
                    }
                }
                ancestors.push_back(pAncestor);
            }
        }

        toPointMod->addPointsProcess(pIdArray.size() * pBaryCoords.size());
        toPointMod->addPointsWarning(pIdArray.size() * pBaryCoords.size(), ancestors);
    }
}

void Mesh2PointTopologicalMapping::addInputEdges(const sofa::core::topology::EdgesAdded* eAdd, PointSetTopologyModifier* toPointMod)
{
    const vector< Vec3d > &eBaryCoords = edgeBaryCoords.getValue();

    const sofa::helper::vector<unsigned int>& eIdArray = eAdd->edgeIndexArray;
    for (unsigned int i : eIdArray)
    {
        if (pointsMappedFrom[EDGE].size() < i + 1)
            pointsMappedFrom[EDGE].resize(i + 1);
        else
            pointsMappedFrom[EDGE][i].clear();

        Edge e = fromModel->getEdge(i);

        Vec3d p0(fromModel->getPX(e[0]), fromModel->getPY(e[0]), fromModel->getPZ(e[0]));
        Vec3d p1(fromModel->getPX(e[1]), fromModel->getPY(e[1]), fromModel->getPZ(e[1]));

        for (unsigned int j = 0; j < eBaryCoords.size(); j++)
        {
            pointsMappedFrom[EDGE][i].push_back(pointSource.size());
            pointSource.push_back(std::make_pair(EDGE, i));
        }

        if (!toPointMod)
        {
            for (unsigned int j = 0; j < eBaryCoords.size(); j++)
            {
                double fx = eBaryCoords[j][0];

                Vec3d result = p0 * (1 - fx) + p1 * fx;

                toModel->addPoint(result[0], result[1], result[2]);
            }
        }
    }

    if (toPointMod)
    {
        helper::vector< helper::vector< unsigned int > > ancestors;
        helper::vector< helper::vector< double       > > coefs;

        ancestors.resize(eIdArray.size() * eBaryCoords.size());
        coefs.resize(eIdArray.size() * eBaryCoords.size());

        std::size_t offset = 0u;
        for (unsigned int i : eIdArray)
        {
            Edge e = fromModel->getEdge(i);
            for (unsigned int j = 0; j < eBaryCoords.size(); j++)
            {
                ancestors[offset + j].push_back(pointsMappedFrom[POINT][e[0]][0]);
                ancestors[offset + j].push_back(pointsMappedFrom[POINT][e[1]][0]);

                coefs[offset + j].push_back(eBaryCoords[j][0]);
                coefs[offset + j].push_back(1 - eBaryCoords[j][0]);
            }
            offset += eBaryCoords.size();
        }

        toPointMod->addPointsProcess(eIdArray.size() * eBaryCoords.size());
        toPointMod->addPointsWarning(eIdArray.size() * eBaryCoords.size(), ancestors, coefs, true);
    }
}

void Mesh2PointTopologicalMapping::addInputTriangles(const sofa::core::topology::TrianglesAdded* tAdd, PointSetTopologyModifier* toPointMod)
{
    const vector< Vec3d > &tBaryCoords = triangleBaryCoords.getValue();

    const sofa::helper::vector<unsigned int>& tIdArray = tAdd->triangleIndexArray;
    for (unsigned int i : tIdArray)
    {
        if (pointsMappedFrom[TRIANGLE].size() < i+1)
            pointsMappedFrom[TRIANGLE].resize(i+1);
        else
            pointsMappedFrom[TRIANGLE][i].clear();

        Triangle t = fromModel->getTriangle(i);

        Vec3d p0(fromModel->getPX(t[0]), fromModel->getPY(t[0]), fromModel->getPZ(t[0]));
        Vec3d p1(fromModel->getPX(t[1]), fromModel->getPY(t[1]), fromModel->getPZ(t[1]));
        Vec3d p2(fromModel->getPX(t[2]), fromModel->getPY(t[2]), fromModel->getPZ(t[2]));

        for (unsigned int j = 0; j < tBaryCoords.size(); j++)
        {
            pointsMappedFrom[TRIANGLE][i].push_back(pointSource.size());
            pointSource.push_back(std::make_pair(TRIANGLE,i));
        }

        if (!toPointMod)
        {
            for (unsigned int j = 0; j < tBaryCoords.size(); j++)
            {
                double fx = tBaryCoords[j][0];
                double fy = tBaryCoords[j][1];

                Vec3d result =  p0 * (1-fx-fy) + p1 * fx + p2 * fy;

                toModel->addPoint(result[0], result[1], result[2]);
            }
        }

    }

    if (toPointMod)
    {
        toPointMod->addPointsProcess(tIdArray.size() * tBaryCoords.size());
        toPointMod->addPointsWarning(tIdArray.size() * tBaryCoords.size(), {}, {});
    }
}


void Mesh2PointTopologicalMapping::addInputTetrahedra(const sofa::core::topology::TetrahedraAdded* tAdd, PointSetTopologyModifier* toPointMod)
{
    const vector< Vec3d > &tBaryCoords = tetraBaryCoords.getValue();

    const sofa::helper::vector<unsigned int>& tIdArray = tAdd->tetrahedronIndexArray;
    for (unsigned int i : tIdArray)
    {
        if (pointsMappedFrom[TETRA].size() < i+1)
            pointsMappedFrom[TETRA].resize(i+1);
        else
            pointsMappedFrom[TETRA][i].clear();

        Tetrahedron t = fromModel->getTetrahedron(i);

        Vec3d p0(fromModel->getPX(t[0]), fromModel->getPY(t[0]), fromModel->getPZ(t[0]));
        Vec3d p1(fromModel->getPX(t[1]), fromModel->getPY(t[1]), fromModel->getPZ(t[1]));
        Vec3d p2(fromModel->getPX(t[2]), fromModel->getPY(t[2]), fromModel->getPZ(t[2]));
        Vec3d p3(fromModel->getPX(t[3]), fromModel->getPY(t[3]), fromModel->getPZ(t[3]));

        for (unsigned int j = 0; j < tBaryCoords.size(); j++)
        {
            pointsMappedFrom[TETRA][i].push_back(pointSource.size());
            pointSource.push_back(std::make_pair(TETRA,i));
        }

        if (!toPointMod)
        {
            for (unsigned int j = 0; j < tBaryCoords.size(); j++)
            {
                double fx = tBaryCoords[j][0];
                double fy = tBaryCoords[j][1];
                double fz = tBaryCoords[j][2];

                Vec3d result =  p0 * (1-fx-fy-fz) + p1 * fx + p2 * fy +p3*fz;

                toModel->addPoint(result[0], result[1], result[2]);
            }
        }
    }

    if (toPointMod)
    {
        toPointMod->addPointsProcess(tIdArray.size() * tBaryCoords.size());
        toPointMod->addPointsWarning(tIdArray.size() * tBaryCoords.size(), {}, {});
    }
}

void Mesh2PointTopologicalMapping::updateTopologicalMappingTopDown()
{
    if(fromModel && toModel && initDone)
    {
        std::list<const TopologyChange *>::const_iterator changeIt=fromModel->beginChange();
        std::list<const TopologyChange *>::const_iterator itEnd=fromModel->endChange();

        PointSetTopologyModifier *toPointMod = NULL;
        EdgeSetTopologyModifier *toEdgeMod = NULL;
        TriangleSetTopologyModifier *toTriangleMod = NULL;
        TetrahedronSetTopologyModifier *toTetrahedronMod = NULL;
        //QuadSetTopologyModifier *toQuadMod = NULL;
        //HexahedronSetTopologyModifier *toHexahedronMod = NULL;
        toModel->getContext()->get(toPointMod, sofa::core::objectmodel::BaseContext::Local);
        bool check = false;
        helper::fixed_array <int, NB_ELEMENTS > nbInputRemoved;
        nbInputRemoved.assign(0);
        std::string laststep = "";
        while( changeIt != itEnd )
        {
            TopologyChangeType changeType = (*changeIt)->getChangeType();
            laststep += " ";
            laststep += sofa::core::topology::parseTopologyChangeTypeToString(changeType);
            switch( changeType )
            {
            case core::topology::POINTSINDICESSWAP:
            {
                unsigned int i1 = ( static_cast< const PointsIndicesSwap * >( *changeIt ) )->index[0];
                unsigned int i2 = ( static_cast< const PointsIndicesSwap* >( *changeIt ) )->index[1];
				sout << "INPUT SWAP POINTS "<<i1 << " " << i2 << sendl;
                swapInput(POINT,i1,i2);
                check = true;
                break;
            }
            case core::topology::POINTSADDED:
            {
                const PointsAdded * pAdd = static_cast< const PointsAdded * >(*changeIt);
                addInputPoints(pAdd, toPointMod);
                check = true;
                break;
            }
            case core::topology::POINTSREMOVED:
            {
                const sofa::helper::vector<unsigned int>& tab = ( static_cast< const PointsRemoved * >( *changeIt ) )->getArray();
				 sout << "INPUT REMOVE POINTS "<<tab << sendl;
                removeInput(POINT, tab );
                check = true;
                nbInputRemoved[POINT] += tab.size();
                break;
            }
            case core::topology::POINTSRENUMBERING:
            {
                const sofa::helper::vector<unsigned int>& tab = ( static_cast< const PointsRenumbering * >( *changeIt ) )->getinv_IndexArray();
				 sout << "INPUT RENUMBER POINTS "<<tab << sendl;
                renumberInput(POINT, tab );
                check = true;
                break;
            }
            case core::topology::EDGESADDED:
            {
                const EdgesAdded *eAdd = static_cast< const EdgesAdded * >( *changeIt );
//				sout << "INPUT ADD EDGES " << tab << sendl;
                addInputEdges(eAdd, toPointMod);
                toPointMod->propagateTopologicalChanges();
                if (copyEdges.getValue())
                {
                    if (!toEdgeMod) toModel->getContext()->get(toEdgeMod, sofa::core::objectmodel::BaseContext::Local);
                    if (toEdgeMod)
                    {
                        sout << "EDGESADDED : " << eAdd->getNbAddedEdges() << sendl;
                        const sofa::helper::vector<Edge>& fromArray = eAdd->edgeArray;
                        sofa::helper::vector<Edge> toArray;
                        toArray.resize(fromArray.size());
                        for (unsigned int i=0; i<fromArray.size(); ++i)
                            for (unsigned int j=0; j<fromArray[i].size(); ++j)
                                toArray[i][j] = pointsMappedFrom[POINT][fromArray[i][j]][0];
                        toEdgeMod->addEdgesProcess(toArray);
                        toEdgeMod->addEdgesWarning(eAdd->getNbAddedEdges(), toArray, eAdd->edgeIndexArray, eAdd->ancestorsList, eAdd->coefs);
                        toEdgeMod->propagateTopologicalChanges();
                    }
                }
                check = true;
                break;
            }
            case core::topology::EDGESREMOVED:
            {
                const EdgesRemoved *eRem = static_cast< const EdgesRemoved * >( *changeIt );
                const sofa::helper::vector<unsigned int> &tab = eRem->getArray();
                if (copyEdges.getValue())
                {
                    if (!toEdgeMod) toModel->getContext()->get(toEdgeMod, sofa::core::objectmodel::BaseContext::Local);
                    if (toEdgeMod)
                    {
                        sout << "EDGESREMOVED : " << eRem->getNbRemovedEdges() << sendl;
                        sofa::helper::vector<unsigned int> toArray = tab;
                        toEdgeMod->removeEdgesWarning(toArray);
                        toEdgeMod->propagateTopologicalChanges();
                        toEdgeMod->removeEdgesProcess(tab, false);
                    }
                }
//				sout << "INPUT REMOVE EDGES "<<tab << sendl;
                removeInput(EDGE, tab );
                check = true;
                nbInputRemoved[EDGE] += tab.size();
                break;
            }
            case core::topology::TRIANGLESADDED:
            {
                const TrianglesAdded *tAdd = static_cast< const TrianglesAdded * >( *changeIt );
//				sout << "INPUT ADD TRIANGLES " << tab << sendl;
                addInputTriangles(tAdd, toPointMod);
                toPointMod->propagateTopologicalChanges();
                if (copyTriangles.getValue())
                {
                    if (!toTriangleMod) toModel->getContext()->get(toTriangleMod, sofa::core::objectmodel::BaseContext::Local);
                    if (toTriangleMod)
                    {
                        sout << "TRIANGLESADDED : " << tAdd->getNbAddedTriangles() << sendl;
                        const sofa::helper::vector<Triangle>& fromArray = tAdd->triangleArray;
                        sofa::helper::vector<Triangle> toArray;
                        toArray.resize(fromArray.size());
                        for (unsigned int i=0; i<fromArray.size(); ++i)
                            for (unsigned int j=0; j<fromArray[i].size(); ++j)
                                toArray[i][j] = pointsMappedFrom[POINT][fromArray[i][j]][0];
                        sout << "<IN: " << fromModel->getNbTriangles() << " OUT: " << toModel->getNbTriangles() << sendl;
                        sout << "     ToArray : " << toArray.size() << " : " << toArray << sendl;
                        toTriangleMod->addTrianglesProcess(toArray);
                        sout << "     triangleIndexArray : " << tAdd->triangleIndexArray.size() << " : " << tAdd->triangleIndexArray << sendl;
                        toTriangleMod->addTrianglesWarning(tAdd->getNbAddedTriangles(), toArray, tAdd->triangleIndexArray, tAdd->ancestorsList, tAdd->coefs);
                        toTriangleMod->propagateTopologicalChanges();
                        sout << ">IN: " << fromModel->getNbTriangles() << " OUT: " << toModel->getNbTriangles() << sendl;
                    }
                }
                check = true;
                break;
            }
            case core::topology::TRIANGLESREMOVED:
            {
                const TrianglesRemoved *tRem = static_cast< const TrianglesRemoved * >( *changeIt );
                const sofa::helper::vector<unsigned int> &tab = tRem->getArray();
                if (copyTriangles.getValue())
                {
                    if (!toTriangleMod) toModel->getContext()->get(toTriangleMod, sofa::core::objectmodel::BaseContext::Local);
                    if (toTriangleMod)
                    {
                        sout << "TRIANGLESREMOVED : " << tRem->getNbRemovedTriangles() << " : " << tab << sendl;
                        sofa::helper::vector<unsigned int> toArray = tab;
                        toTriangleMod->removeTrianglesWarning(toArray);
                        toTriangleMod->propagateTopologicalChanges();
                        toTriangleMod->removeTrianglesProcess(tab, false);
                    }
                }
//				sout << "INPUT REMOVE TRIANGLES "<<tab << sendl;
                removeInput(TRIANGLE, tab );
                check = true;
                nbInputRemoved[TRIANGLE] += tab.size();
                break;
            }
            case core::topology::QUADSADDED:
            {
                /// @todo
                break;
            }
            case core::topology::QUADSREMOVED:
            {
                const sofa::helper::vector<unsigned int> &tab = ( static_cast< const QuadsRemoved *>( *changeIt ) )->getArray();
//				sout << "INPUT REMOVE QUADS "<<tab << sendl;
                removeInput(QUAD, tab );
                check = true;
                nbInputRemoved[QUAD] += tab.size();
                break;
            }
            case core::topology::TETRAHEDRAADDED:
            {
				const TetrahedraAdded *tAdd = static_cast< const TetrahedraAdded * >( *changeIt );
//				sout << "INPUT ADD TETRAHEDRA " << tab << sendl;
                addInputTetrahedra(tAdd, toPointMod);
                toPointMod->propagateTopologicalChanges();
                if (copyTetrahedra.getValue())
                {
                    if (!toTetrahedronMod) toModel->getContext()->get(toTetrahedronMod, sofa::core::objectmodel::BaseContext::Local);
                    if (toTetrahedronMod)
                    {
                        sout << "TETRAHEDRAADDED : " << tAdd->getNbAddedTetrahedra() << sendl;
                        const sofa::helper::vector<Tetrahedron>& fromArray = tAdd->tetrahedronArray;
                        sofa::helper::vector<Tetrahedron> toArray;
                        toArray.resize(fromArray.size());
                        for (unsigned int i=0; i<fromArray.size(); ++i)
                            for (unsigned int j=0; j<fromArray[i].size(); ++j)
                                toArray[i][j] = pointsMappedFrom[POINT][fromArray[i][j]][0];
                        sout << "<IN: " << fromModel->getNbTetrahedra() << " OUT: " << toModel->getNbTetrahedra() << sendl;
                        sout << "     ToArray : " << toArray.size() << " : " << toArray << sendl;
                        toTetrahedronMod->addTetrahedraProcess(toArray);
                        sout << "     tetrahedronIndexArray : " << tAdd->tetrahedronIndexArray.size() << " : " << tAdd->tetrahedronIndexArray << sendl;
                        toTetrahedronMod->addTetrahedraWarning(tAdd->getNbAddedTetrahedra(), toArray, tAdd->tetrahedronIndexArray, tAdd->ancestorsList, tAdd->coefs);
                        toTetrahedronMod->propagateTopologicalChanges();
                        sout << ">IN: " << fromModel->getNbTetrahedra() << " OUT: " << toModel->getNbTetrahedra() << sendl;
                    }
                }
                check = true;
                break;
            }
            case core::topology::TETRAHEDRAREMOVED:
            {
				const TetrahedraRemoved *tRem = static_cast< const TetrahedraRemoved * >( *changeIt );
                const sofa::helper::vector<unsigned int> &tab = tRem->getArray();
                if (copyTetrahedra.getValue())
                {
                    if (!toTetrahedronMod) toModel->getContext()->get(toTetrahedronMod, sofa::core::objectmodel::BaseContext::Local);
                    if (toTetrahedronMod)
                    {
                        sout << "TETRAHEDRAREMOVED : " << tRem->getNbRemovedTetrahedra() << " : " << tab << sendl;
                        sofa::helper::vector<unsigned int> toArray = tab;
                        toTetrahedronMod->removeTetrahedraWarning(toArray);
                        toTetrahedronMod->propagateTopologicalChanges();
                        toTetrahedronMod->removeTetrahedraProcess(tab, false);
                    }
                }
//				sout << "INPUT REMOVE TETRAHEDRA "<<tab << sendl;
                removeInput(TETRA, tab );
                nbInputRemoved[TETRA] += tab.size();
                check = true;
                break;
            }
            case core::topology::HEXAHEDRAADDED:
            {
                /// @TODO
                break;
            }
            case core::topology::HEXAHEDRAREMOVED:
            {
                const sofa::helper::vector<unsigned int> &tab = ( static_cast< const HexahedraRemoved *>( *changeIt ) )->getArray();
//				sout << "INPUT REMOVE HEXAHEDRA "<<tab << sendl;
                removeInput(HEXA, tab );
                check = true;
                nbInputRemoved[TETRA] += tab.size();
                break;
            }
            case core::topology::ENDING_EVENT:
            {
//			    sout << "ENDING EVENT" << sendl;
                
                pointsToRemove.erase(BaseMeshTopology::InvalidID);
                if (toPointMod != NULL)
                {
                    if (!pointsToRemove.empty())
                    {
                        // TODO: This will fail to work if add and
                        // remove changes are combined and removes are
                        // signaled prior to adds! The indices will mix
                        // up.

                        sofa::helper::vector<unsigned int> vitems;
                        vitems.reserve(pointsToRemove.size());
                        vitems.insert(vitems.end(), pointsToRemove.rbegin(), pointsToRemove.rend());

                        toPointMod->removePointsWarning(vitems);
                        toPointMod->propagateTopologicalChanges();

                        removeOutputPoints(vitems);

                        toPointMod->removePointsProcess(vitems);
                        pointsToRemove.clear();
                    }

                    toPointMod->propagateTopologicalChanges();
                    toPointMod->notifyEndingEvent();
                    toPointMod->propagateTopologicalChanges();
                }

                check = true;
                break;
            }

            default:
                sout << "IGNORING " << sofa::core::topology::parseTopologyChangeTypeToString(changeType) << sendl;
                break;

            }
            ++changeIt;
        }
        if (check)
            internalCheck(laststep.c_str(), nbInputRemoved);
    }
}

void Mesh2PointTopologicalMapping::swapInput(Element elem, int i1, int i2)
{
    if (pointsMappedFrom[elem].empty()) return;
    vector<int> i1Map = pointsMappedFrom[elem][i1];
    vector<int> i2Map = pointsMappedFrom[elem][i2];

    pointsMappedFrom[elem][i1] = i2Map;
    for(unsigned int i = 0; i < i2Map.size(); ++i)
    {
        if (i2Map[i] != -1) pointSource[i2Map[i]].second = i1;
    }

    pointsMappedFrom[elem][i2] = i1Map;
    for(unsigned int i = 0; i < i1Map.size(); ++i)
    {
        if (i1Map[i] != -1) pointSource[i1Map[i]].second = i2;
    }
}

void Mesh2PointTopologicalMapping::removeInput(Element elem,  const sofa::helper::vector<unsigned int>& index )
{
    if (pointsMappedFrom[elem].empty()) return;
    unsigned int last = pointsMappedFrom[elem].size() -1;

    for (unsigned int i = 0; i < index.size(); ++i)
    {
        if (index[i] != last)
        {
            swapInput(elem, index[i], last );
        }
        for (unsigned int j = 0; j < pointsMappedFrom[elem][last].size(); ++j)
        {
            int map = pointsMappedFrom[elem][last][j];
            if (map != -1)
            {
                pointsToRemove.insert(map);
                pointSource[map].second = -1;
            }
        }
        --last;
    }

    pointsMappedFrom[elem].resize( last + 1 );
}

void Mesh2PointTopologicalMapping::renumberInput(Element elem, const sofa::helper::vector<unsigned int>& index )
{
    if (pointsMappedFrom[elem].empty()) return;
    helper::vector< vector<int> > copy = pointsMappedFrom[elem];
    for (unsigned int i = 0; i < index.size(); ++i)
    {
        const vector<int>& map = copy[index[i]];
        pointsMappedFrom[elem][i] = map;
        for (unsigned int j = 0; j < map.size(); ++j)
        {
            int m = map[j];
            if (m != -1)
                pointSource[m].second = i;
        }
    }
}

void Mesh2PointTopologicalMapping::swapOutputPoints(int i1, int i2, bool removeLast)
{
    std::pair<Element, int> i1Source = pointSource[i1];
    std::pair<Element, int> i2Source = pointSource[i2];
    pointSource[i1] = i2Source;
    pointSource[i2] = i1Source;
    if (i1Source.second != -1)
    {
        // replace i1 by i2 in pointsMappedFrom[i1Source.first][i1Source.second]
        vector<int> & pts = pointsMappedFrom[i1Source.first][i1Source.second];
        for (unsigned int j = 0; j < pts.size(); ++j)
        {
            if (pts[j] == i1)
            {
                if (removeLast)
                    pts[j] = -1;
                else
                    pts[j] = i2;
            }
        }
    }
    if (i2Source.second != -1)
    {
        // replace i2 by i1 in pointsMappedFrom[i2Source.first][i1Source.second]
        vector<int> & pts = pointsMappedFrom[i2Source.first][i2Source.second];
        for (unsigned int j = 0; j < pts.size(); ++j)
        {
            if (pts[j] == i2)
                pts[j] = i1;
        }
    }
}

void Mesh2PointTopologicalMapping::removeOutputPoints( const sofa::helper::vector<unsigned int>& index )
{
    unsigned int last = pointSource.size() - 1;

    for (unsigned int i = 0; i < index.size(); ++i)
    {
        swapOutputPoints( index[i], last, true );
        --last;
    }

    pointSource.resize(last + 1);
}

} // namespace topology
} // namespace component
} // namespace sofa

