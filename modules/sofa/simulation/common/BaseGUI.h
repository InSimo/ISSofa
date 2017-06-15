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
#ifndef SOFA_SIMULATION_COMMON_BASEGUI_H
#define SOFA_SIMULATION_COMMON_BASEGUI_H

#include <sofa/helper/system/config.h>
#include <sofa/simulation/common/Node.h>

#include <map>
#include <vector>
#include <string>
#include <memory>

namespace sofa
{

namespace simulation
{

namespace gui
{

class SOFA_SIMULATION_COMMON_API BaseGUI
{
public:
    struct CopyScreenInfo
    {
        void* ctx;
        unsigned int name;
        unsigned int target;
        int srcX;
        int srcY;
        int dstX;
        int dstY;
        int width;
        int height;
    };

    virtual void initialize(const char* programName) = 0;
    virtual void stepMainLoop() {}
    virtual int mainLoop() = 0;
    virtual void setViewerResolution(int  width, int  height) {};
    virtual void setScene(sofa::simulation::Node::SPtr groot, const char* filename = nullptr, bool temporaryFile = false) = 0;
    virtual void redraw() = 0;
    virtual void showFPS(double fps) {};
    virtual bool getCopyScreenRequest(CopyScreenInfo& copyScreenInfo) { return false; }
    virtual void useCopyScreen(CopyScreenInfo& copyScreenInfo) { }
    virtual sofa::simulation::Node* getCurrentSimulation() = 0;

    virtual bool saveScreenshot(const std::string& filename, int compression_level = -1) = 0;
    virtual void sendMessage(const std::string& msgType, const std::string& msgValue) {}
    virtual void setMaxFPS(double fpsMaxRate) {}
    virtual void setBackgroundColor(const defaulttype::Vector3& color) {}

    virtual void getViewerView(sofa::defaulttype::Vec3d& pos, sofa::defaulttype::Quat& ori) = 0;
    virtual void setViewerView(const sofa::defaulttype::Vec3d& pos, const sofa::defaulttype::Quat &ori) = 0;

    void addGUIOption(const char* option);

protected:
    std::vector<std::string> guiOptions;
};

} // namespace gui

} // namespace simulation

} // namespace sofa

#endif
