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
#ifndef SOFA_COMPONENT_TOPOLOGY_MESH2POINTTOPOLOGICALMAPPING_H
#define SOFA_COMPONENT_TOPOLOGY_MESH2POINTTOPOLOGICALMAPPING_H

#include <sofa/SofaGeneral.h>
#include <sofa/core/topology/TopologicalMapping.h>
#include <SofaBaseTopology/PointSetTopologyModifier.h>
#include <sofa/core/topology/TopologyChange.h>

#include <sofa/defaulttype/Vec.h>
#include <map>
#include <set>

#include <sofa/core/BaseMapping.h>
#include <SofaBaseTopology/TopologyData.h>


namespace sofa
{
namespace component
{
namespace topology
{
/**
 * \brief Creates a new topology containing additional points from an existing one
 *
 * For each primitive in the input topology a new point will be created in the output topology computed from a parameter vector 
 * (pointBaryCoords, edgeBaryCoords, triangleBaryCoords, quadBaryCoords, tetraBaryCoords, hexaBaryCoords) containing the barycentric coordinate of the point 
 *  with respect to the primitive
 *
 * Example: if the input topology is a triangle mesh using this mapping with pointBaryCoords="0 0 0" edgeBaryCoords="0.5 0 0" copyEdges="true" copyTriangles="true"
 * will produce the same triangle and edge topology for the output topology as the input, with additional isolated points created in the middle of each edge. 
 *
*/

class SOFA_TOPOLOGY_MAPPING_API Mesh2PointTopologicalMapping : public sofa::core::topology::TopologicalMapping
{
public:
    SOFA_CLASS_EXTERNAL((Mesh2PointTopologicalMapping),((sofa::core::topology::TopologicalMapping)));
    typedef sofa::defaulttype::Vec3d Vec3d;

protected:
    /** \brief Constructor.
     *
     */
    Mesh2PointTopologicalMapping ();

    /** \brief Destructor.
     *
         * Does nothing.
         */
    virtual ~Mesh2PointTopologicalMapping() {};
public:
    /** \brief Initializes the target BaseTopology from the source BaseTopology.
     */
    virtual void init();

    /// Method called at each topological changes propagation which comes from the INPUT topology to adapt the OUTPUT topology :
    virtual void updateTopologicalMappingTopDown();

    virtual unsigned int getGlobIndex(unsigned int ind)
    {
        if(ind<pointSource.size())
        {
            return pointSource[ind].second;
        }
        else
        {
            return 0;
        }
    }

    virtual unsigned int getFromIndex(unsigned int ind)
    {
        return ind;
    }

    enum Element
    {
        POINT = 0,
        EDGE,
        TRIANGLE,
        QUAD,
        TETRA,
        HEXA,
        NB_ELEMENTS
    };


    const helper::vector< helper::vector<int> >& getPointsMappedFromPoint() const { return pointsMappedFrom[POINT]; }
    const helper::vector< helper::vector<int> >& getPointsMappedFromEdge() const { return pointsMappedFrom[EDGE]; }
    const helper::vector< helper::vector<int> >& getPointsMappedFromTriangle() const { return pointsMappedFrom[TRIANGLE]; }
    const helper::vector< helper::vector<int> >& getPointsMappedFromQuad() const { return pointsMappedFrom[QUAD]; }
    const helper::vector< helper::vector<int> >& getPointsMappedFromTetra() const { return pointsMappedFrom[TETRA]; }
    const helper::vector< helper::vector<int> >& getPointsMappedFromHexa() const { return pointsMappedFrom[HEXA]; }

    template<class TopoType>
    const helper::vector< helper::vector<int> >& getPointsMappedFromElem() const;

    const helper::vector< Vec3d >& getPointBaryCoords() const { return pointBaryCoords.getValue(); }
    const helper::vector< Vec3d >& getEdgeBaryCoords() const { return edgeBaryCoords.getValue(); }
    const helper::vector< Vec3d >& getTriangleBaryCoords() const { return triangleBaryCoords.getValue(); }
    const helper::vector< Vec3d >& getQuadBaryCoords() const { return quadBaryCoords.getValue(); }
    const helper::vector< Vec3d >& getTetraBaryCoords() const { return tetraBaryCoords.getValue(); }
    const helper::vector< Vec3d >& getHexaBaryCoords() const { return hexaBaryCoords.getValue(); }

    const helper::vector< std::pair<Element,int> >& getPointSource() const { return pointSource;}

    void setPointBaryCoords(const sofa::helper::vector<Vec3d> pBaryCoords)    { pointBaryCoords.setValue(pBaryCoords);    }
    void setEdgeBaryCoords(const sofa::helper::vector<Vec3d> eBaryCoords)     { edgeBaryCoords.setValue(eBaryCoords);     }
    void setTriangleBaryCoords(const sofa::helper::vector<Vec3d> tBaryCoords) { triangleBaryCoords.setValue(tBaryCoords); }
    void setQuadBaryCoords(const sofa::helper::vector<Vec3d> qBaryCoords)     { quadBaryCoords.setValue(qBaryCoords);     }
    void setTetraBaryCoords(const sofa::helper::vector<Vec3d> tetBaryCoords)  { tetraBaryCoords.setValue(tetBaryCoords);  }
    void setHexaBaryCoords(const sofa::helper::vector<Vec3d> hBaryCoords)     { hexaBaryCoords.setValue(hBaryCoords);     }

    void setCopyEdges(bool copy)      { copyEdges.setValue(copy);      }
    void setCopyTriangles(bool copy)  { copyTriangles.setValue(copy);  }
    void setCopyTetrahedra(bool copy) { copyTetrahedra.setValue(copy); }

protected:

    Data< helper::vector< Vec3d > > pointBaryCoords; ///< Coordinates for the points of the output topology created from the points of the input topology
    Data< helper::vector< Vec3d > > edgeBaryCoords; ///< Coordinates for the points of the output topology created from the edges of the input topology
    Data< helper::vector< Vec3d > > triangleBaryCoords; ///< Coordinates for the points of the output topology created from the triangles of the input topology
    Data< helper::vector< Vec3d > > quadBaryCoords; ///< Coordinates for the points of the output topology created from the quads of the input topology
    Data< helper::vector< Vec3d > > tetraBaryCoords; ///< Coordinates for the points of the output topology created from the tetra of the input topology
    Data< helper::vector< Vec3d > > hexaBaryCoords; ///< Coordinates for the points of the output topology created from the hexa of the input topology

    Data< bool > copyEdges; ///< Activate mapping of input edges into the output topology (requires at least one item in pointBaryCoords)
    Data< bool > copyTriangles; ///< Activate mapping of input triangles into the output topology (requires at least one item in pointBaryCoords)
	Data< bool > copyTetrahedra; ///< Activate mapping of input tetrahedras into the output topology (requires at least one item in pointBaryCoords)

    helper::fixed_array< vector< vector<int> >, NB_ELEMENTS > pointsMappedFrom; ///< Points mapped from the differents elements (see the enum Element declared before)

    vector< std::pair<Element,int> > pointSource; ///< Correspondance between the points mapped and the elements from which are mapped

    std::set<unsigned int> pointsToRemove;

    void addInputPoints(const sofa::core::topology::PointsAdded *pAdd, PointSetTopologyModifier* toPointMod = nullptr);
    void addInputEdges(const sofa::core::topology::EdgesAdded* eAdd, PointSetTopologyModifier* toPointMod = nullptr);
    void addInputTriangles(const sofa::core::topology::TrianglesAdded* tAdd, PointSetTopologyModifier* toPointMod = nullptr);
    void addInputTetrahedra(const sofa::core::topology::TetrahedraAdded* tAdd, PointSetTopologyModifier* toPointMod = nullptr);

    void swapInput(Element elem, int i1, int i2);
    void removeInput(Element elem, const sofa::helper::vector<unsigned int>& tab );
    void renumberInput(Element elem, const sofa::helper::vector<unsigned int>& index );

    void swapOutputPoints(int i1, int i2, bool removeLast = false);
    void removeOutputPoints( const sofa::helper::vector<unsigned int>& tab );

protected:
    bool internalCheck(const char* step, const helper::fixed_array <int, NB_ELEMENTS >& nbInputRemoved);
    
    bool internalCheck(const char* step)
    {
        helper::fixed_array <int, NB_ELEMENTS > nbInputRemoved;
        nbInputRemoved.assign(0);
        return internalCheck(step, nbInputRemoved);
    }
    bool initDone;
};


template<> inline const helper::vector< helper::vector<int> >& Mesh2PointTopologicalMapping::getPointsMappedFromElem<sofa::core::topology::Topology::Point>() const { return getPointsMappedFromPoint(); }
template<> inline const helper::vector< helper::vector<int> >& Mesh2PointTopologicalMapping::getPointsMappedFromElem<sofa::core::topology::Topology::Edge>() const { return getPointsMappedFromEdge(); }
template<> inline const helper::vector< helper::vector<int> >& Mesh2PointTopologicalMapping::getPointsMappedFromElem<sofa::core::topology::Topology::Triangle>() const { return getPointsMappedFromTriangle(); }
template<> inline const helper::vector< helper::vector<int> >& Mesh2PointTopologicalMapping::getPointsMappedFromElem<sofa::core::topology::Topology::Quad>() const { return getPointsMappedFromQuad(); }
template<> inline const helper::vector< helper::vector<int> >& Mesh2PointTopologicalMapping::getPointsMappedFromElem<sofa::core::topology::Topology::Tetrahedron>() const { return getPointsMappedFromTetra(); }
template<> inline const helper::vector< helper::vector<int> >& Mesh2PointTopologicalMapping::getPointsMappedFromElem<sofa::core::topology::Topology::Hexahedron>() const { return getPointsMappedFromHexa(); }


} // namespace topology
} // namespace component
} // namespace sofa

#endif // SOFA_COMPONENT_TOPOLOGY_MESH2POINTTOPOLOGICALMAPPING_H
