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
#ifndef SOFA_COMPONENT_LOADER_MESHOBJLOADER_H
#define SOFA_COMPONENT_LOADER_MESHOBJLOADER_H

#include <sofa/core/loader/MeshLoader.h>
#include <sofa/SofaCommon.h>
#include <sofa/helper/SVector.h>
namespace sofa
{

namespace component
{

namespace loader
{

class MapFaceTexCoord
{
    typedef unsigned int FaceId;
    typedef sofa::helper::pair<sofa::defaulttype::Vector2, sofa::helper::set<FaceId> > PairTexFace;

    typedef helper::vector< PairTexFace >::const_iterator ConstItPair;
    typedef helper::vector< PairTexFace >::iterator       ItPair;

public:
    MapFaceTexCoord()
    {
        m_mapTexFaces.clear();
        m_defaultTexCoord = sofa::defaulttype::Vector2();
    }
    helper::vector< PairTexFace > m_mapTexFaces;
    sofa::defaulttype::Vector2 m_defaultTexCoord = sofa::defaulttype::Vector2();

    const sofa::defaulttype::Vector2& getTexCoord(FaceId faceId) const
    {
        if (m_mapTexFaces.empty())
        {
            std::cerr << " TexCoord container empty " << std::endl;
            return m_defaultTexCoord;
        }

        // An optimized implementation (credit to Thomas Jund)
        // the trade-off is you don't check the sanity of the structure
        if (m_mapTexFaces.size() == 1)
        {
            return m_mapTexFaces[0].first;
        }
        // -------------------------------

        ConstItPair it = std::find_if(m_mapTexFaces.cbegin(), m_mapTexFaces.cend(), [&](const PairTexFace& pairTF)
        {
            return (pairTF.second.find(faceId) != pairTF.second.end());
        });

        if (it == m_mapTexFaces.cend())
        {
            std::cerr << " Non existent texCoord on face Id : " << faceId << " in map : "  << *this <<  std::endl;
            return m_defaultTexCoord;
        }
        else
        {
            return it->first;
        }
    }

    const sofa::defaulttype::Vector2& operator[] (FaceId faceId) const
    {
        return getTexCoord(faceId);
    }

    helper::vector<sofa::defaulttype::Vector2> getAllTexCoord() const
    {
        helper::vector<sofa::defaulttype::Vector2> result;
        for (const auto it : m_mapTexFaces)
        {
            result.emplace_back(it.first);
        }
        return result;
    }

    void setTexCoord(const FaceId& faceId, const sofa::defaulttype::Vector2& newTexCoord)
    {
        ItPair it = std::find_if(m_mapTexFaces.begin(), m_mapTexFaces.end(), [&](const PairTexFace& oldPairTF)
        {
            const sofa::defaulttype::Vector2& oldTexCoord = oldPairTF.first;
            return (newTexCoord[0] == oldTexCoord[0] && newTexCoord[1] == oldTexCoord[1]);
        });

        if (it == m_mapTexFaces.end())
        {
            m_mapTexFaces.resize(m_mapTexFaces.size() + 1);
            m_mapTexFaces.back().first = newTexCoord;
            m_mapTexFaces.back().second.insert(faceId);
        }
        else
        {
            it->second.insert(faceId);
        }
    }

    inline bool isDuplicated() const
    {
        return (m_mapTexFaces.size() != 1);
    }

    inline std::size_t size() const
    {
        return m_mapTexFaces.size();
    }


    SOFA_STRUCT_DECL(MapFaceTexCoord, m_mapTexFaces);
    SOFA_STRUCT_STREAM_METHODS(MapFaceTexCoord);
    SOFA_STRUCT_COMPARE_METHOD(MapFaceTexCoord);
};

} // namespace loader

} // namespace component

} // namespace sofa

SOFA_STRUCT_DEFINE_TYPEINFO(sofa::component::loader::MapFaceTexCoord);


namespace sofa
{

namespace component
{

namespace loader
{

class SOFA_LOADER_API MeshObjLoader : public sofa::core::loader::MeshLoader
{
public:
    enum FaceType { EDGE, TRIANGLE, QUAD, NBFACETYPE };

    typedef unsigned int FaceId;

    SOFA_CLASS(MeshObjLoader,sofa::core::loader::MeshLoader);
protected:
    MeshObjLoader();
    virtual ~MeshObjLoader();
public:
    virtual bool load();

    template <class T>
    static bool canCreate ( T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg )
    {
        return BaseLoader::canCreate (obj, context, arg);
    }

protected:
    bool readOBJ (std::istringstream& filestream, const char* filename);
    bool readMTL (const char* filename, helper::vector <sofa::core::loader::Material>& materials);
    void addGroup (const sofa::core::loader::PrimitiveGroup& g);

    sofa::core::loader::Material material;
    Data<bool> d_handleSeams;
    Data<bool> loadMaterial;
    std::string textureName;
    FaceType faceType;

public:
    Data <helper::vector <sofa::core::loader::Material> > materials;
    Data <helper::SVector <helper::SVector <int> > > faceList;
    Data <helper::SVector <helper::SVector <int> > > texIndexList;
    Data <helper::vector<sofa::defaulttype::Vector3> > positionsList;
    Data< helper::vector<sofa::defaulttype::Vector2> > texCoordsList;
    Data <helper::SVector<helper::SVector<int> > > normalsIndexList;
    Data <helper::vector<sofa::defaulttype::Vector3> > normalsList;
    Data< helper::vector<sofa::defaulttype::Vector2> > texCoords;
    Data< helper::vector<MapFaceTexCoord> > texCoordsInFace; // texCoordsInFace[PointID][FaceId] should give you the texCoord of the point indexed by PointID in the Face indexed by FaceID
    Data< bool > computeMaterialFaces;
    helper::vector< Data <helper::vector <unsigned int> >* > subsets_indices;

    /// If vertices have multiple normals/texcoords, then we need to separate them
    /// This vector store which input position is used for each vertex
    /// If it is empty then each vertex correspond to one position
    Data< helper::vector<int> > d_vertPosIdx;

    /// Similarly this vector store which input normal is used for each vertex
    /// If it is empty then each vertex correspond to one normal
    Data< helper::vector<int> > d_vertNormIdx;

    /// List of points on discontinuity
    Data< helper::vector<unsigned int> > d_pointsOnBorder;

    virtual std::string type() { return "The format of this mesh is OBJ."; }
};


} // namespace loader

} // namespace component

} // namespace sofa

#endif
