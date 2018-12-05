/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_COMPONENT_VISUALMODEL_OGLCLEAR_H
#define SOFA_COMPONENT_VISUALMODEL_OGLCLEAR_H

#include <sofa/SofaGeneral.h>
#include <sofa/core/visual/VisualModel.h>

namespace sofa
{

namespace component
{

namespace visualmodel
{

class SOFA_OPENGL_VISUAL_API OglClear : public virtual sofa::core::visual::VisualModel
{
public:
    SOFA_CLASS(OglClear, sofa::core::visual::VisualModel);

protected:
    OglClear();
    virtual ~OglClear();

public:

    Data<bool> d_clear;
    Data<bool> d_clearColor;
    Data<bool> d_clearDepth;
    Data<bool> d_clearStencil;
    Data<bool> d_passStd;
    Data<bool> d_passTransparent;
    Data<bool> d_passShadow;

    void fwdDraw(core::visual::VisualParams* vparams) override;
};

} // namespace visualmodel

} // namespace component

} // namespace sofa

#endif
