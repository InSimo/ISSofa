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
#ifndef SOFA_COMPONENT_TOPOLOGY_TRIANGLESETTOPOLOGYCONTAINER_H
#define SOFA_COMPONENT_TOPOLOGY_TRIANGLESETTOPOLOGYCONTAINER_H

#include <SofaBaseTopology/EdgeSetTopologyContainer.h>

namespace sofa
{

namespace component
{

namespace topology
{
class TriangleSetTopologyModifier;

using core::topology::BaseMeshTopology;

typedef BaseMeshTopology::PointID		            	PointID;
typedef BaseMeshTopology::EdgeID		               	EdgeID;
typedef BaseMeshTopology::TriangleID	               TriangleID;
typedef BaseMeshTopology::Edge		        	         Edge;
typedef BaseMeshTopology::Triangle	        	         Triangle;
typedef BaseMeshTopology::SeqTriangles	        	      SeqTriangles;
typedef BaseMeshTopology::EdgesInTriangle	         	EdgesInTriangle;
typedef BaseMeshTopology::TrianglesAroundVertex    	TrianglesAroundVertex;
typedef BaseMeshTopology::TrianglesAroundEdge        	TrianglesAroundEdge;
typedef sofa::helper::vector<TriangleID>                  VecTriangleID;

/*! \class TriangleSetTopologyContainer
\brief: Object that stores a set of triangles and provides access
to each triangle and its edges and vertices */
class SOFA_BASE_TOPOLOGY_API TriangleSetTopologyContainer : public EdgeSetTopologyContainer
{
    friend class TriangleSetTopologyModifier;

public:
    SOFA_CLASS(TriangleSetTopologyContainer,EdgeSetTopologyContainer);
protected:
    TriangleSetTopologyContainer();

    virtual ~TriangleSetTopologyContainer() {}
public:
    virtual void init();

    virtual void reinit();


    /// Procedural creation methods
    /// @{
    virtual void clear();
    virtual void addEdge( int, int ) {}
    virtual void addTriangle( int a, int b, int c );
    /// @}



    /// BaseMeshTopology API
    /// @{

    /** \brief Returns the quad array. */
    virtual const SeqTriangles& getTriangles()
    {
        return getTriangleArray();
    }

    /** \brief Returns a reference to the Data of triangles array container. */
    Data< sofa::helper::vector<Triangle> >& getTriangleDataArray() {return d_triangle;}


    /** \brief Returns the triangle corresponding to the TriangleID i.
     *         Not thread safe can recompute adjacency information
     *
     * @param ID of a triangle.
     * @return The corresponding triangle.
     */
    virtual const Triangle getTriangle(TriangleID i);

    /** \brief Returns the triangle corresponding to the TriangleID i. 
    *          Thread safe
    *
    * @param ID of a triangle.
    * @return The corresponding triangle.
    */
    const Triangle& getTriangle(TriangleID i) const;


    /* Returns the indices of a triangle given three vertex indices.
    *  Thread safe : does not recompute any adjacency information.
    *
    * @param the three vertex indices.
    * @return the ID of the corresponding triangle.
    * @return -1 if none
    */
    int getTriangleIndex(PointID v1, PointID v2, PointID v3) const;


    /* Returns the indices of a triangle given three vertex indices.
     * Not thread safe if the adjacency information for the vertex indices is not up to date.
     * 
     * @param the three vertex indices.
     * @return the ID of the corresponding triangle.
     * @return -1 if none
     */
    virtual int getTriangleIndex(PointID v1, PointID v2, PointID v3);


    /** \brief Returns the 3 edges adjacent to a given triangle.
    *          Thread safe, does not recompute any adjacency information for the edges.
    *
    * @param ID of a triangle.
    * @return EdgesInTriangle list composing the input triangle.
    */
    const EdgesInTriangle& getEdgesInTriangle(TriangleID i) const;

    /** \brief Returns the 3 edges adjacent to a given triangle. 
     *         Not thread safe since it can recompute the adjacency informations for the edges.
     *
     * @param ID of a triangle.
     * @return EdgesInTriangle list composing the input triangle.
     */
    virtual const EdgesInTriangle& getEdgesInTriangle(TriangleID i);


    /** \brief Returns the set of triangles adjacent to a given vertex. 
     *         Thread safe, does not recompute any adjacency information.
     *
     * @param ID of a vertex
     * @return TrianglesAroundVertex list around the input vertex
     */
    const TrianglesAroundVertex& getTrianglesAroundVertex(PointID i) const;

    /** \brief Returns the set of triangles adjacent to a given vertex.
    *
    * @param ID of a vertex
    * @return TrianglesAroundVertex list around the input vertex
    */
    virtual const TrianglesAroundVertex& getTrianglesAroundVertex(PointID i);


    /** \brief Returns the set of triangles adjacent to a given edge.
    *          Thread safe, does not recompute any adjacency information.
    *
    * @param ID of an edge.
    * @return TrianglesAroundEdge list around the input edge.
    */
    const TrianglesAroundEdge& getTrianglesAroundEdge(EdgeID i) const;

    /** \brief Returns the set of triangles adjacent to a given edge.
     *         Not thread safe, can recompute adjacency information.
     *
     * @param ID of an edge.
     * @return TrianglesAroundEdge list around the input edge.
     */
    virtual const TrianglesAroundEdge& getTrianglesAroundEdge(EdgeID i);


    /** \brief Returns the index (either 0, 1 ,2) of the vertex whose global index is vertexIndex.
     *
     * @param Ref to a triangle.
     * @param Id of a vertex.
     * @return the position of this vertex in the triangle (i.e. either 0, 1, 2).
     * @return -1 if none.
     */
    virtual int getVertexIndexInTriangle(const Triangle &t, PointID vertexIndex) const;

    /** \brief Returns the index (either 0, 1 ,2) of the edge whose global index is edgeIndex.
     *
     * @param Ref to an EdgesInTriangle.
     * @param Id of an edge.
     * @return the position of this edge in the triangle (i.e. either 0, 1, 2).
     * @return -1 if none.
     */
    virtual int getEdgeIndexInTriangle(const EdgesInTriangle &t, EdgeID edgeIndex) const;

    /// @}



    /// Dynamic Topology API
    /// @{


    /** \brief Checks if the topology is coherent
     *
     * Check if the shell arrays are coherent
     * @see m_triangle
     * @see m_edgesInTriangle
     * @see m_trianglesAroundVertex
     * @see m_trianglesAroundEdge
     */
    virtual bool checkTopology() const;


    /** \brief Returns the number of triangles in this topology.
     *	The difference to getNbTriangles() is that this method does not generate the triangle array if it does not exist.
     */
    unsigned int getNumberOfTriangles() const;

    /** \brief Returns the number of topological element of the current topology.
     * This function avoids to know which topological container is in used.
     */
    virtual unsigned int getNumberOfElements() const;

    /** \brief Returns the Triangle array. Thread safe */
    const sofa::helper::vector<Triangle> &getTriangleArray() const;

    /** \brief Returns the Triangle array. */
    const sofa::helper::vector<Triangle> &getTriangleArray();

    /** \brief Returns the EdgesInTriangle array (i.e. provide the 3 edge indices for each triangle). Thread safe*/
    const sofa::helper::vector< EdgesInTriangle > &getEdgesInTriangleArray() const;

    /** \brief Returns the EdgesInTriangle array (i.e. provide the 3 edge indices for each triangle). */
    const sofa::helper::vector< EdgesInTriangle > &getEdgesInTriangleArray() ;


    /** \brief Returns the TrianglesAroundVertex array (i.e. provide the triangles indices adjacent to each vertex). Thread safe*/
    const sofa::helper::vector< TrianglesAroundVertex > &getTrianglesAroundVertexArray() const;

    /** \brief Returns the TrianglesAroundVertex array (i.e. provide the triangles indices adjacent to each vertex). */
    const sofa::helper::vector< TrianglesAroundVertex > &getTrianglesAroundVertexArray();

    /** \brief Returns the TrianglesAroundEdge array (i.e. provide the triangles indices adjacent to each edge). Thread safe*/
    const sofa::helper::vector< TrianglesAroundEdge > &getTrianglesAroundEdgeArray() const;

    /** \brief Returns the TrianglesAroundEdge array (i.e. provide the triangles indices adjacent to each edge). */
    const sofa::helper::vector< TrianglesAroundEdge > &getTrianglesAroundEdgeArray() ;

    /** \brief: Return a list of TriangleID which are on a border.
     * @see createElementsOnBorder()
     */
    const sofa::helper::vector <TriangleID>& getTrianglesOnBorder();


    /** \brief: Return a list of EdgeID which are on a border.
     * @see createElementsOnBorder()
     */
    const sofa::helper::vector <EdgeID>& getEdgesOnBorder();


    /** \brief: Return a vector of PointID which are on a border.
     * @see createElementsOnBorder()
     */
    const sofa::helper::vector <PointID>& getPointsOnBorder();


    /// Get information about connexity of the mesh
    /// @{
    /** \brief Checks if the topology has only one connected component
      *
      * @return true if only one connected component
      */
    virtual bool checkConnexity();

    /// Returns the number of connected component.
    virtual unsigned int getNumberOfConnectedComponent();

    /// Returns the set of element indices connected to an input one (i.e. which can be reached by topological links)
    virtual const VecTriangleID getConnectedElement(TriangleID elem);

    /// Returns the set of element indices adjacent to a given element (i.e. sharing a link)
    virtual const VecTriangleID getElementAroundElement(TriangleID elem);
    /// Returns the set of element indices adjacent to a given list of elements (i.e. sharing a link)
    virtual const VecTriangleID getElementAroundElements(VecTriangleID elems);
    /// @}

    bool hasTriangles() const;

    bool hasEdgesInTriangle() const;

    bool hasTrianglesAroundVertex() const;

    bool hasTrianglesAroundEdge() const;

    bool hasBorderElementLists() const;

    /** \brief Returns the type of the topology */
    virtual sofa::core::topology::TopologyObjectType getTopologyType() const {return sofa::core::topology::TRIANGLE;}


    /** \brief: Create element lists which are on topology border:
     *
     * - A vector of TriangleID @see m_trianglesOnBorder. ( I.e which have at least: one edge not adjacent
     to an other Triangle)
     * - A vector of EdgeID @see m_edgesOnBorder. (I.e which are adjacent to only one Triangle)
     * - A vector of PointID @see m_pointsOnBorder. (I.e which are part of only one Triangle)
     */
    void createElementsOnBorder();

    /// @}

    /// Will change order of vertices in triangle: t[1] <=> t[2]
    void reOrientateTriangle(TriangleID id);

    /// Create / update all topological arrays that are derived from the "master" elements
    /// (i.e. edges around triangles, edges in triangles, triangles around points, ...)
    void createDerivedData() override;

protected:

    /** \brief Creates the TriangleSet array.
     *
     * This function must be implemented by derived classes to create a list of triangles from a set of tetrahedra for instance
     */
    virtual void createTriangleSetArray();


    /** \brief Creates the EdgeSet array.
     *
     * Create the set of edges when needed.
     */
    virtual void createEdgeSetArray();


    /** \brief Creates the array of edge indices for each triangle.
     *
     * This function is only called if the EdgesInTriangle array is required.
     * m_edgesInTriangle[i] contains the 3 indices of the 3 edges composing the ith triangle.
     */
    virtual void createEdgesInTriangleArray();


    /** \brief Creates the TrianglesAroundVertex Array.
     *
     * This function is only called if the TrianglesAroundVertex array is required.
     * m_trianglesAroundVertex[i] contains the indices of all triangles adjacent to the ith DOF.
     */
    virtual void createTrianglesAroundVertexArray();


    /** \brief Creates the TrianglesAroundEdge Array.
     *
     * This function is only called if the TrianglesAroundVertex array is required.
     * m_trianglesAroundEdge[i] contains the indices of all triangles adjacent to the ith edge.
     */
    virtual void createTrianglesAroundEdgeArray();


    void clearTriangles();

    void clearEdgesInTriangle();

    void clearTrianglesAroundVertex();

    void clearTrianglesAroundEdge();

    void clearBorderElementLists();


protected:

    /** \brief Returns a non-const list of triangle indices around a given DOF for subsequent modification.
     *
     * @return TrianglesAroundVertex lists in non-const.
     * @see getTrianglesAroundVertex()
     */
    virtual TrianglesAroundVertex& getTrianglesAroundVertexForModification(const PointID vertexIndex);


    /** \brief Returns a non-const list of triangle indices around a given edge for subsequent modification.
     *
     * @return TrianglesAroundEdge lists in non-const.
     * @see getTrianglesAroundEdge()
     */
    virtual TrianglesAroundEdge& getTrianglesAroundEdgeForModification(const EdgeID edgeIndex);


    /// \brief Function creating the data graph linked to d_triangle
    virtual void updateTopologyEngineGraph();


    /// Use a specific boolean @see m_triangleTopologyDirty in order to know if topology Data is dirty or not.
    /// Set/Get function access to this boolean
    void setTriangleTopologyToDirty() {m_triangleTopologyDirty = true;}
    void cleanTriangleTopologyFromDirty() {m_triangleTopologyDirty = false;}
    const bool& isTriangleTopologyDirty() {return m_triangleTopologyDirty;}

public:
    /// provides the set of triangles.
    Data< sofa::helper::vector<Triangle> > d_triangle;

    /// If true, this will erase the loaded edge set and create another one based on the triangular topology
    Data< bool >                           d_createEdgeSetArray;

protected:
    /// provides the 3 edges in each triangle.
    sofa::helper::vector<EdgesInTriangle> m_edgesInTriangle;

    /// for each vertex provides the set of triangles adjacent to that vertex.
    sofa::helper::vector< TrianglesAroundVertex > m_trianglesAroundVertex;

    /// for each edge provides the set of triangles adjacent to that edge.
    sofa::helper::vector< TrianglesAroundEdge > m_trianglesAroundEdge;

    /// Set of triangle indices on topology border.
    sofa::helper::vector <TriangleID> m_trianglesOnBorder;

    /// Set of edge indices on topology border.
    sofa::helper::vector <EdgeID> m_edgesOnBorder;

    /// Set of point indices on topology border.
    sofa::helper::vector <PointID> m_pointsOnBorder;

    /// Boolean used to know if the topology Data of this container is dirty
    bool m_triangleTopologyDirty = false;

    /// List of engines related to this specific container
    sofa::helper::list <sofa::core::topology::TopologyEngine *> m_enginesList;

    /// \brief variables used to display the graph of Data/DataEngines linked to this Data array.
    sofa::helper::vector < sofa::helper::vector <std::string> > m_dataGraph;
    sofa::helper::vector < sofa::helper::vector <std::string> > m_enginesGraph;

};

} // namespace topology

} // namespace component

} // namespace sofa

#endif
