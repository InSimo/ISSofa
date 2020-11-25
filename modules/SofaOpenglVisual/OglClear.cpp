/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <SofaOpenglVisual/OglClear.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa
{

namespace component
{

namespace visualmodel
{

SOFA_DECL_CLASS(OglClear)

int OglClearClass = core::RegisterObject("Call glClear with the specified flags.")
.add< OglClear >()
;

OglClear::OglClear()
: d_clear(initData(&d_clear, true, "clear", "Toggle the call to glClear()"))
, d_clearColor(initData(&d_clearColor, false, "clearColor", "Enable clearing of color buffer"))
, d_clearDepth(initData(&d_clearDepth, false, "clearDepth", "Enable clearing of depth buffer"))
, d_clearStencil(initData(&d_clearStencil, false, "clearStencil", "Enable clearing of stencil buffer"))
, d_passStd(initData(&d_passStd, true, "passStd", "Activate during Std rendering pass (default)"))
, d_passTransparent(initData(&d_passTransparent, false, "passTransparent", "Activate during Transparent rendering pass"))
, d_passShadow(initData(&d_passShadow, false, "passShadow", "Activate during Shadow rendering pass"))

{
}

OglClear::~OglClear()
{
}

void OglClear::fwdDraw(core::visual::VisualParams* vparams)
{
    if (!d_clear.getValue()) return;
    switch(vparams->pass())
    {
        case core::visual::VisualParams::Std:
            if (!d_passStd.getValue()) return;
            break;
        case core::visual::VisualParams::Transparent:
            if (!d_passTransparent.getValue()) return;
            break;
        case core::visual::VisualParams::Shadow:
            if (!d_passShadow.getValue()) return;
            break;
    }
    GLbitfield buffers = 0;
    if (d_clearColor.getValue()) buffers |= GL_COLOR_BUFFER_BIT;
    if (d_clearDepth.getValue()) buffers |= GL_DEPTH_BUFFER_BIT;
    if (d_clearStencil.getValue()) buffers |= GL_STENCIL_BUFFER_BIT;
    if (buffers)
    {
        glDisable(GL_DEPTH_TEST);
        glClear(buffers);
        glEnable(GL_DEPTH_TEST);
    }
}

} // namespace visualmodel

} // namespace component

} // namespace sofa
