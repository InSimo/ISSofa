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
#ifndef SOFA_COMPONENT_VISUALMODEL_OGLMODEL_H
#define SOFA_COMPONENT_VISUALMODEL_OGLMODEL_H

#include <vector>
#include <string>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/gl/Texture.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/SofaGeneral.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <SofaBaseVisual/VisualModelImpl.h>

#define   NB_MAX_TEXTURES 16

namespace sofa
{

namespace component
{

namespace visualmodel
{

/**
 *  \brief Main class for rendering 3D model in SOFA.
 *
 *  This class implements VisuelModelImpl with rendering functions
 *  using OpenGL.
 *
 */

class SOFA_OPENGL_VISUAL_API OglModel : public VisualModelImpl
{
public:
    SOFA_CLASS_EXTERNAL((OglModel), ((VisualModelImpl)));

protected:
    Data<bool> premultipliedAlpha, useVBO, writeZTransparent, alphaBlend, depthTest, d_stencilTest;
    Data<int> cullFace;
    Data<GLfloat> lineWidth;
    Data<GLfloat> pointSize;
    Data<bool> lineSmooth;
    Data<bool> pointSmooth;
    /// Suppress field for save as function
    Data < bool > isToPrint;

    using DataOptions = Data<sofa::helper::OptionsGroup>;

    // primitive types
    DataOptions primitiveType;

    //alpha blend function
    DataOptions blendEquation;
    DataOptions sourceFactor;
    DataOptions destFactor;
    GLenum blendEq, sfactor, dfactor;

    //stencil function
    DataOptions    d_stencilFuncFront;
    Data<unsigned> d_stencilFuncRefFront;
    Data<unsigned> d_stencilFuncMaskFront;
    DataOptions    d_stencilOpSFailFront;
    DataOptions    d_stencilOpDpFailFront;
    DataOptions    d_stencilOpDpPassFront;
    Data<unsigned> d_stencilOpMaskFront;
    DataOptions    d_stencilFuncBack;
    Data<unsigned> d_stencilFuncRefBack;
    Data<unsigned> d_stencilFuncMaskBack;
    DataOptions  d_stencilOpSFailBack;
    DataOptions  d_stencilOpDpFailBack;
    DataOptions  d_stencilOpDpPassBack;
    Data<unsigned> d_stencilOpMaskBack;
    static std::pair<helper::vector<GLenum>,helper::vector<std::string> > OptionsStencilFunc;
    static std::pair<helper::vector<GLenum>,helper::vector<std::string> > OptionsStencilOp;

    // support for enabling clip planes within shaders
    Data<int> d_activeClipDistances;

    // allow disabling writing to color buffer (i.e. only depth and/or stencil)
    Data<GLfloat> d_colorMask;

    helper::gl::Texture *tex; //this texture is used only if a texture name is specified in the scn
    GLuint vbo, iboEdges, iboTriangles, iboQuads;
    bool canUseVBO, VBOGenDone, initDone, useEdges, useTriangles, useQuads, canUsePatches;
    unsigned int oldVerticesSize, oldEdgesSize, oldTrianglesSize, oldQuadsSize;
    void internalDraw(const core::visual::VisualParams* vparams, bool transparent);

    void drawGroup(int ig, bool transparent);
    void drawGroups(bool transparent);

    virtual void pushTransformMatrix(float* matrix) { glPushMatrix(); glMultMatrixf(matrix); }
    virtual void popTransformMatrix() { glPopMatrix(); }

    std::vector<helper::gl::Texture*> textures;

    std::map<int, int> materialTextureIdMap; //link between a material and a texture

    GLenum getGLenum(const char* c ) const;


    OglModel();

    ~OglModel();
public:

    bool loadTexture(const std::string& filename);
    bool loadTextures() ;

    void initTextures();
    virtual void initVisual();

    virtual void init() { VisualModelImpl::init(); }

    virtual void updateBuffers();

    bool hasTransparent();

public:
    bool isUseEdges()	{ return useEdges; }
    bool isUseTriangles()	{ return useTriangles; }
    bool isUseQuads()	{ return useQuads; }
    bool isUseVbo()	{ return useVBO.getValue(); }

    helper::gl::Texture* getTex() const	{ return tex; }
    GLuint getVbo()	{ return vbo;	}
    GLuint getIboEdges() { return iboEdges; }
    GLuint getIboTriangles() { return iboTriangles; }
    GLuint getIboQuads()    { return iboQuads; }
    const std::vector<helper::gl::Texture*>& getTextures() const { return textures;	}

#ifdef SOFA_HAVE_GLEW
    void createVertexBuffer();
    void createEdgesIndicesBuffer();
    void createTrianglesIndicesBuffer();
    void createQuadsIndicesBuffer();
    void initVertexBuffer();
    void initEdgesIndicesBuffer();
    void initTrianglesIndicesBuffer();
    void initQuadsIndicesBuffer();
    void updateVertexBuffer();
    void updateEdgesIndicesBuffer();
    void updateTrianglesIndicesBuffer();
    void updateQuadsIndicesBuffer();
#endif
};

typedef sofa::defaulttype::Vec<3,GLfloat> GLVec3f;
typedef sofa::defaulttype::ExtVectorTypes<GLVec3f,GLVec3f> GLExtVec3fTypes;

} // namespace visualmodel

} // namespace component

} // namespace sofa

#endif
