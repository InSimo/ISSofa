/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <SofaTopologyMapping/IdentityTopologicalMapping.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/ObjectFactory.h>

#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/EdgeSetTopologyModifier.h>

#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyModifier.h>

#include <SofaBaseTopology/QuadSetTopologyContainer.h>
#include <SofaBaseTopology/QuadSetTopologyModifier.h>

#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyModifier.h>

#include <SofaBaseTopology/HexahedronSetTopologyContainer.h>
#include <SofaBaseTopology/HexahedronSetTopologyModifier.h>

#include <sofa/core/topology/TopologyChange.h>
#include <sofa/defaulttype/Vec.h>
#include <map>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa
{

namespace component
{

namespace topology
{

using namespace sofa::defaulttype;

using namespace sofa::component::topology;
using namespace sofa::core::topology;

SOFA_DECL_CLASS(IdentityTopologicalMapping)

// Register in the Factory
int IdentityTopologicalMappingClass = core::RegisterObject("This class is a specific implementation of TopologicalMapping where the destination topology should be kept identical to the source topology. The implementation currently assumes that both topology have been initialized identically.")
        .add< IdentityTopologicalMapping >()

        ;

IdentityTopologicalMapping::IdentityTopologicalMapping()
: d_createOutputTopologyAtInit(initData(&d_createOutputTopologyAtInit, true, "createOutputTopologyAtInit", "If true, the output topology is create from input topology"))
, d_initOnly(initData(&d_initOnly, false, "initOnly", "If true, only considere topo change at init"))
{
}


IdentityTopologicalMapping::~IdentityTopologicalMapping()
{
}

void IdentityTopologicalMapping::init()
{
    sofa::core::topology::TopologicalMapping::init();
    this->updateLinks();
    if (d_createOutputTopologyAtInit.getValue() && fromModel && toModel)
    {
        toModel->clear();

        PointSetTopologyModifier *toPointMod = nullptr;
        EdgeSetTopologyModifier *toEdgeMod   = nullptr;
        TriangleSetTopologyModifier *toTriangleMod = nullptr;

        toModel->getContext()->get(toPointMod);
        toModel->getContext()->get(toEdgeMod);
        toModel->getContext()->get(toTriangleMod);


        if (!toPointMod)
        {
            for (int i = 0; i < fromModel->getNbPoints(); i++)
            {
                toModel->addPoint(fromModel->getPX(i), fromModel->getPY(i), fromModel->getPZ(i));
            }
        }
        else
        {
            sofa::helper::vector<sofa::core::topology::PointAncestorElem> ancestors;
            ancestors.resize(fromModel->getNbPoints());
            for (int i = 0; i < fromModel->getNbPoints(); i++)
            {
                sofa::core::topology::PointAncestorElem anc;
                anc.type = sofa::core::topology::TopologyObjectType::POINT;
                anc.index = core::topology::Topology::InvalidID;
                anc.localCoords = sofa::defaulttype::Vec3d(fromModel->getPX(i), fromModel->getPY(i), fromModel->getPZ(i));
                ancestors[i] = anc;
            }
            toPointMod->addPoints(fromModel->getNbPoints(), ancestors, true);
        }


        const auto& edges = fromModel->getEdges();
        if (!toEdgeMod)
        {
            for (auto e : edges)
            {
                toModel->addEdge(e[0], e[1]);
            }
        }
        else
        {
            toEdgeMod->addEdges(edges);
        }

        const auto& triangles = fromModel->getTriangles();
        if (!toTriangleMod)
        {
            for (auto tri : triangles)
            {
                toModel->addTriangle(tri[0], tri[1], tri[2]);
            }
        }
        else
        {
            toTriangleMod->addTriangles(triangles);
        }

        const auto& tetrahedra = fromModel->getTetrahedra();
        for (auto tetra : tetrahedra)
        {
            toModel->addTetra(tetra[0], tetra[1], tetra[2], tetra[3]);
        }

        const auto& hexahedra = fromModel->getHexahedra();
        for (auto hexa : hexahedra)
        {
            toModel->addHexa(hexa[0], hexa[1], hexa[2], hexa[3], hexa[4], hexa[5], hexa[6], hexa[7]);
        }

        //Need to call container createDerivedData() method to update topology neighborhood information
        toModel->createDerivedData();
    }
}

unsigned int IdentityTopologicalMapping::getFromIndex(unsigned int ind)
{
    return ind;
}

void IdentityTopologicalMapping::updateTopologicalMappingTopDown()
{
    if (!fromModel || !toModel || d_initOnly.getValue()) return;

    std::list<const TopologyChange *>::const_iterator itBegin=fromModel->beginChange();
    std::list<const TopologyChange *>::const_iterator itEnd=fromModel->endChange();

    if (itBegin == itEnd) return;

    PointSetTopologyModifier *toPointMod = nullptr;
    EdgeSetTopologyModifier *toEdgeMod   = nullptr;
    TriangleSetTopologyModifier *toTriangleMod = nullptr;
    
    toModel->getContext()->get(toPointMod);
    toModel->getContext()->get(toEdgeMod);
    toModel->getContext()->get(toTriangleMod);

    TriangleSetTopologyContainer *fromTriangleCon = nullptr;
    fromModel->getContext()->get(fromTriangleCon);

    TriangleSetTopologyContainer *toTriangleCon = nullptr;
    toModel->getContext()->get(toTriangleCon);

    if (!toPointMod)
    {
        serr << "No PointSetTopologyModifier found for target topology." << sendl;
        return;
    }

    while( itBegin != itEnd )
    {
        const TopologyChange* topoChange = *itBegin;
        TopologyChangeType changeType = topoChange->getChangeType();

        switch( changeType )
        {

        case core::topology::ENDING_EVENT:
        {
//            std::cout << "ENDING_EVENT" << std::endl;
            toPointMod->propagateTopologicalChanges();
            toPointMod->notifyEndingEvent();
            toPointMod->propagateTopologicalChanges();
            break;
        }

        case core::topology::POINTSADDED:
        {
            const PointsAdded * pAdd = static_cast< const PointsAdded * >( topoChange );
//            std::cout << "POINTSADDED : " << pAdd->getNbAddedVertices() << std::endl;
            toPointMod->addPointsProcess(pAdd->getNbAddedVertices());
            toPointMod->addPointsWarning(pAdd->getNbAddedVertices(), pAdd->ancestorsList, pAdd->coefs, true);
            toPointMod->propagateTopologicalChanges();
            break;
        }
        case core::topology::POINTSREMOVED:
        {
            const PointsRemoved *pRem = static_cast< const PointsRemoved * >( topoChange );
            sofa::helper::vector<unsigned int> tab = pRem->getArray();
//            std::cout << "POINTSREMOVED : " << tab.size() << std::endl;
            toPointMod->removePointsWarning(tab, true);
            toPointMod->propagateTopologicalChanges();
            toPointMod->removePointsProcess(tab, true);
            break;
        }
        case core::topology::POINTSRENUMBERING:
        {
            const PointsRenumbering *pRenumber = static_cast< const PointsRenumbering * >( topoChange );
            const sofa::helper::vector<unsigned int> &tab = pRenumber->getIndexArray();
            const sofa::helper::vector<unsigned int> &inv_tab = pRenumber->getinv_IndexArray();
//            std::cout << "POINTSRENUMBERING : " << tab.size() <<std::endl;
            toPointMod->renumberPointsWarning(tab, inv_tab, true);
            toPointMod->propagateTopologicalChanges();
            toPointMod->renumberPointsProcess(tab, inv_tab, true);
            break;
        }

        case core::topology::EDGESADDED:
        {
            if (!toEdgeMod) toModel->getContext()->get(toEdgeMod);
            if (!toEdgeMod) break;
            const EdgesAdded *eAdd = static_cast< const EdgesAdded * >( topoChange );
//            std::cout << "EDGESADDED : " << eAdd->getNbAddedEdges() << std::endl;
            toEdgeMod->addEdgesProcess(eAdd->edgeArray);
            toEdgeMod->addEdgesWarning(eAdd->getNbAddedEdges(), eAdd->edgeArray, eAdd->edgeIndexArray, eAdd->ancestorsList, eAdd->coefs);
            toEdgeMod->propagateTopologicalChanges();
            break;
        }

        case core::topology::EDGESREMOVED:
        {
            if (!toEdgeMod) toModel->getContext()->get(toEdgeMod);
            if (!toEdgeMod) break;
            const EdgesRemoved *eRem = static_cast< const EdgesRemoved * >( topoChange );
            sofa::helper::vector<unsigned int> tab = eRem->getArray();
//            std::cout << "EDGESREMOVED : " << std::endl;
            toEdgeMod->removeEdgesWarning(tab);
            toEdgeMod->propagateTopologicalChanges();
            toEdgeMod->removeEdgesProcess(tab, false);
            break;
        }

        case core::topology::TRIANGLESADDED:
        {
            if (!toTriangleMod) toModel->getContext()->get(toTriangleMod);
            if (!toTriangleMod) break;
            const TrianglesAdded *tAdd = static_cast< const TrianglesAdded * >( topoChange );
//            std::cout << "TRIANGLESADDED : " << tAdd->getNbAddedTriangles() << std::endl;
            toTriangleMod->addTrianglesProcess(tAdd->triangleArray);
            toTriangleMod->addTrianglesWarning(tAdd->getNbAddedTriangles(), tAdd->triangleArray, tAdd->triangleIndexArray, tAdd->ancestorsList, tAdd->coefs);
            toTriangleMod->propagateTopologicalChanges();
            break;
        }

        case core::topology::TRIANGLESREMOVED:
        {
            if (!toTriangleMod) toModel->getContext()->get(toTriangleMod);
            if (!toTriangleMod) break;
            const TrianglesRemoved *tRem = static_cast< const TrianglesRemoved * >( topoChange );
            sofa::helper::vector<unsigned int> tab = tRem->getArray();
//            std::cout << "TRIANGLESREMOVED : " << tab.size() << std::endl;
            toTriangleMod->removeTrianglesWarning(tab);
            toTriangleMod->propagateTopologicalChanges();
            toTriangleMod->removeTrianglesProcess(tab, false);
            break;
        }

        default:
            break;
        };

        ++itBegin;
    }
    toPointMod->propagateTopologicalChanges();

//    std::cout << "End Nb of points of fromModel : " << fromTriangleCon->getNbPoints() << std::endl;
//    std::cout << "End Nb of points of fromState : " << fromTriangleCon->getContext()->getState()->getSize() << std::endl;
//    std::cout << "End Nb of edges of fromModel : " << fromTriangleCon->getNbEdges() << std::endl;
//    std::cout << "End Nb of triangles of fromModel : " << fromTriangleCon->getNbTriangles() << std::endl;

//    std::cout << "End Nb of points of toModel : " << toTriangleCon->getNbPoints() << std::endl;
//    std::cout << "End Nb of points of toState : " << toTriangleCon->getContext()->getState()->getSize() << std::endl;
//    std::cout << "End Nb of edges of toModel : " << toTriangleCon->getNbEdges() << std::endl;
//    std::cout << "End Nb of triangles of toModel : " << toTriangleCon->getNbTriangles() << std::endl;

}

} // namespace topology

} // namespace component

} // namespace sofa

