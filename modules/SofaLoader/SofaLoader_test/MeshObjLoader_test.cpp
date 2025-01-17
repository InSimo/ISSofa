/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include "Sofa_test.h"

#include <SofaLoader/MeshObjLoader.h>

using namespace sofa::component::loader;

namespace sofa {

class MeshObjLoader_test  : public ::testing::Test, public MeshObjLoader
{
public :

    /**
     * Constructor call for each test
     */
    void SetUp(){}

    /**
     * Helper function to check mesh loading.
     * Compare basic values from a mesh with given results.
     */
    void loadTest(std::string filename, int pointNb, int edgeNb, int triangleNb,  int quadNb, int polygonNb,
                  int tetraNb, int hexaNb, int normalPerVertexNb, int normalListNb, int texCoordListNb, int materialNb)
    {
        this->setFilename(sofa::helper::system::DataRepository.getFile(filename));
        this->load();
        EXPECT_EQ(pointNb, this->positions.getValue().size());
        EXPECT_EQ(edgeNb, this->edges.getValue().size());
        EXPECT_EQ(triangleNb, this->triangles.getValue().size());
        EXPECT_EQ(quadNb, this->quads.getValue().size());
        EXPECT_EQ(polygonNb, this->polygons.getValue().size());
        EXPECT_EQ(tetraNb, this->tetrahedra.getValue().size());
        EXPECT_EQ(hexaNb, this->hexahedra.getValue().size());
        EXPECT_EQ(normalPerVertexNb, this->normals.getValue().size());
        EXPECT_EQ(normalListNb, this->normalsList.getValue().size());
        EXPECT_EQ(texCoordListNb, this->texCoordsList.getValue().size());
        EXPECT_EQ(materialNb, this->materials.getValue().size());
    }

};

/** MeshObjLoader::load()
 * For a given meshes check that imported data are correct
 */
TEST_F(MeshObjLoader_test, LoadCall)
{
    //Check loader high level result
    EXPECT_FALSE(this->load());
    this->setFilename(sofa::helper::system::DataRepository.getFile("mesh/square.obj"));
    EXPECT_TRUE(this->load());

    //Use several meshes to test : edges, triangles, quads, normals, materials NUMBER
    loadTest("mesh/square.obj", 4, 4, 0,  0, 0, 0, 0, 0, 0, 0, 0);
    loadTest("mesh/dragon.obj", 1190, 0, 2564,  0, 0, 0, 0, 0, 0, 0, 0);
    loadTest("mesh/box.obj", 4, 0, 4,  0, 0, 0, 0, 4, 2, 0, 0);
    loadTest("mesh/cube.obj", 8, 0, 12,  0, 0, 0, 0, 8, 8, 0, 0);
    loadTest("mesh/caducee_base.obj", 3576, 0, 420,  3350, 0, 0, 0, 0, 0, 0, 8);
    loadTest("mesh/torus.obj", 800, 0, 1600,  0, 0, 0, 0, 0, 0, 861, 0);
}

} // namespace sofa
