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

#include <SofaOpenglVisual/OglLabel.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>

#include <string>
#include <iostream>

namespace sofa
{

namespace component
{

namespace visualmodel
{


SOFA_DECL_CLASS(OglLabel)

int OglLabelClass = core::RegisterObject("A simple visualization for 2D text.")
        .add< OglLabel >()
        ;

OglLabel::OglLabel(): stepCounter(0)
  ,prefix(initData(&prefix, std::string(""), "prefix", "The prefix of the text to display"))
  ,label(initData(&label, std::string(""), "label", "The text to display"))
  ,suffix(initData(&suffix, std::string(""), "suffix", "The suffix of the text to display"))
  ,x(initData(&x, (float)10, "x", "The x position of the text on the screen (<1 : fraction of view width, >1 : pixels)"))
  ,y(initData(&y, (float)10, "y", "The y position of the text on the screen (<1 : fraction of view height, >1 : pixels)"))
  ,fontsize(initData(&fontsize, (unsigned int)14, "fontsize", "The size of the font used to display the text on the screen "))
  ,color(initData(&color, std::string("contrast"), "color", "The color of the text to display"))
  ,updateLabelEveryNbSteps(initData(&updateLabelEveryNbSteps, (unsigned int)0, "updateLabelEveryNbSteps", "Update the display of the label every nb of time steps"))
  ,f_visible(initData(&f_visible,true,"visible","Is label displayed"))
  ,f_allViewports(initData(&f_allViewports, false, "allViewports", "Set to true to enable rendering in all viewports (use tags to select which subset)"))
{
    f_listening.setValue(true);
}

void OglLabel::init()
{
    this->getContext()->getRootContext()->get(backgroundSetting, sofa::core::objectmodel::BaseContext::SearchRoot);
    if (color.getValue() == "contrast")
    {
        if (!backgroundSetting)
        {
            color.setValue("white");
        }
        else
        {
            if (f_printLog.getValue()) sout << "Background color is " << backgroundSetting->color.getValue() << sendl;
        }
    }

    reinit();
}


void OglLabel::reinit()
{
    internalLabel = label.getValue();
    setColor(color.getValue());
}

void OglLabel::updateVisual()
{
    if (!updateLabelEveryNbSteps.getValue()) internalLabel = label.getValue();
}

void OglLabel::handleEvent(sofa::core::objectmodel::Event *event)
{
    if ( /*simulation::AnimateEndEvent* ev =*/  sofa::simulation::AnimateBeginEvent::DynamicCast(event))
    {
        if (updateLabelEveryNbSteps.getValue())
        {
            stepCounter++;
            if(stepCounter > updateLabelEveryNbSteps.getValue())
            {
                stepCounter = 0;
                internalLabel = label.getValue();
            }
        }
    }
}

void OglLabel::drawVisual(const core::visual::VisualParams* vparams)
{

    if (!f_visible.getValue() ) return;

    const core::visual::VisualParams::Viewport& viewport = vparams->viewport();
    if (!f_allViewports.getValue() && (viewport[0] != 0 || viewport[1] != 0))
    {
        return; // do not display in viewports other that the main view
    }

    // Save state and disable clipping plane
    glPushAttrib(GL_ENABLE_BIT);
    GLint maxClipPlanes;
    glGetIntegerv(GL_MAX_CLIP_PLANES, &maxClipPlanes);
    for (int i = 0; i < maxClipPlanes; ++i)
		glDisable(GL_CLIP_PLANE0+i);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_1D);
    glDisable(GL_BLEND);
    glDepthMask(1);

    vparams->drawTool()->setLightingEnabled(false);
	// vparams->drawTool()->setPolygonMode(1,true);

    // color of the text
    Color color(r, g, b, a);
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, &color[0]);
    static const float emissive[4] = { 0.0f, 0.0f, 0.0f, 0.0f};
    static const float specular[4] = { 1.0f, 1.0f, 1.0f, 1.0f};
    glMaterialfv (GL_FRONT_AND_BACK, GL_EMISSION, emissive);
    glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf  (GL_FRONT_AND_BACK, GL_SHININESS, 20);
    glColor4f(r, g, b, a);

    std::string text = prefix.getValue() + internalLabel.c_str() + suffix.getValue();


    int w = x.getValue() > 1 ? int (x.getValue()) : int (x.getValue()*viewport[2]);
    int h = y.getValue() > 1 ? int (y.getValue()) : int (y.getValue()*viewport[3]);

    vparams->drawTool()->writeOverlayText(
        w, h, fontsize.getValue(),  // x, y, size
        color,
        text.c_str());


    // Restore state
    glPopAttrib();
}


void OglLabel::setColor(float r, float g, float b, float a)
{
    this->r = r;
    this->g = g;
    this->b = b;
    this->a = a;

    if (f_printLog.getValue()) sout << "Set color to: " << r << ", " << g << ", " << b << ", " << a << sendl;
}

static int hexval(char c)
{
    if (c>='0' && c<='9') return c-'0';
    else if (c>='a' && c<='f') return (c-'a')+10;
    else if (c>='A' && c<='F') return (c-'A')+10;
    else return 0;
}

void OglLabel::setColor(std::string color)
{
    if (color.empty()) return;
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
    float a = 1.0f;
    if (color[0]>='0' && color[0]<='9')
    {
        sscanf(color.c_str(),"%f %f %f %f", &r, &g, &b, &a);
    }
    else if (color[0]=='#' && color.length()>=7)
    {
        r = (hexval(color[1])*16+hexval(color[2]))/255.0f;
        g = (hexval(color[3])*16+hexval(color[4]))/255.0f;
        b = (hexval(color[5])*16+hexval(color[6]))/255.0f;
        if (color.length()>=9)
            a = (hexval(color[7])*16+hexval(color[8]))/255.0f;
    }
    else if (color[0]=='#' && color.length()>=4)
    {
        r = (hexval(color[1])*17)/255.0f;
        g = (hexval(color[2])*17)/255.0f;
        b = (hexval(color[3])*17)/255.0f;
        if (color.length()>=5)
            a = (hexval(color[4])*17)/255.0f;
    }
    else if (color == "white")    { r = 1.0f; g = 1.0f; b = 1.0f; }
    else if (color == "black")    { r = 0.0f; g = 0.0f; b = 0.0f; }
    else if (color == "red")      { r = 1.0f; g = 0.0f; b = 0.0f; }
    else if (color == "green")    { r = 0.0f; g = 1.0f; b = 0.0f; }
    else if (color == "blue")     { r = 0.0f; g = 0.0f; b = 1.0f; }
    else if (color == "cyan")     { r = 0.0f; g = 1.0f; b = 1.0f; }
    else if (color == "magenta")  { r = 1.0f; g = 0.0f; b = 1.0f; }
    else if (color == "yellow")   { r = 1.0f; g = 1.0f; b = 0.0f; }
    else if (color == "gray")     { r = 0.5f; g = 0.5f; b = 0.5f; }
    else if (color == "contrast")
    {
        if (backgroundSetting)
        {
            //in contrast mode, the text color is selected between black or white depending on the background color
            defaulttype::Vector3 backgroundColor = backgroundSetting->color.getValue();
            backgroundColor *= 255;
            SReal yiq = backgroundColor[0]*299 + backgroundColor[1]*587 + backgroundColor[2]*114;
            yiq /= 1000;
            if (yiq >= 128)
            {
                if (f_printLog.getValue()) sout << "Black is selected to display text on this background" << sendl;
                r = 0.0f; g = 0.0f; b = 0.0f;
            }
            else
            {
                if (f_printLog.getValue()) sout << "White is selected to display text on this background" << sendl;
                r = 1.0f; g = 1.0f; b = 1.0f;
            }
        }
        else
        {
            serr << "Background setting not found, cannot use contrast on color data (set white instead)" << sendl;
            r = 1.0f; g = 1.0f; b = 1.0f;
        }
    }
    else
    {
        serr << "Unknown color "<<color<<sendl;
        return;
    }
    setColor(r,g,b,a);
}


} // namespace visualmodel

} // namespace component

} // namespace sofa
