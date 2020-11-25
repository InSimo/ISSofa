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

#include <sofa/core/visual/DrawTool.h>

namespace sofa
{

namespace core
{

namespace visual
{

DrawTool::DrawTool()
{
    clear();
    setColors();
}

DrawTool::~DrawTool()
{
}

static std::vector<std::pair<std::string, defaulttype::Vec4f> > defaultColors =
{
    {"white", defaulttype::Vec4f(255, 255, 255, 255) / 255.f},

    {"green", defaulttype::Vec4f(0.f, 255.f, 0.f, 255) / 255.f},
    {"dark green", defaulttype::Vec4f(0.f, 128.f, 0.f, 255) / 255.f},
    {"olive", defaulttype::Vec4f(107.f, 142.f, 35.f, 255) / 255.f},
    {"pale green", defaulttype::Vec4f(152, 251, 152, 255) / 255.f},
    {"lime green", defaulttype::Vec4f(50, 205, 50, 255) / 255.f},

    {"blue", defaulttype::Vec4f(0, 0, 255, 255) / 255.f},
    {"navy", defaulttype::Vec4f(0, 0, 128, 255) / 255.f},
    {"dark cyan", defaulttype::Vec4f(0, 139, 139, 255) / 255.f},
    {"steel blue", defaulttype::Vec4f(70, 130, 180, 255) / 255.f},
    {"royal blue", defaulttype::Vec4f(65, 105, 225, 255) / 255.f},
    {"sky", defaulttype::Vec4f(135, 206, 235, 255) / 255.f},
    {"lavender", defaulttype::Vec4f(230, 230, 250, 255) / 255.f},

    {"red", defaulttype::Vec4f(255, 0, 0.f, 255) / 255.f},
    {"dark red", defaulttype::Vec4f(139, 0, 0, 255) / 255.f},
    {"violet", defaulttype::Vec4f(199, 21, 133, 255) / 255.f},
    {"orchid", defaulttype::Vec4f(218, 112, 214, 255) / 255.f},
    {"purple", defaulttype::Vec4f(128, 0, 128, 255) / 255.f},

    {"yellow", defaulttype::Vec4f(255, 255, 0, 255) / 255.f},
    {"orange", defaulttype::Vec4f(255, 140, 0, 255) / 255.f},
    {"gold", defaulttype::Vec4f(255, 215, 0, 255) / 255.f},
    {"coral", defaulttype::Vec4f(240, 128, 128, 255) / 255.f},
    {"salmon", defaulttype::Vec4f(255, 160, 122, 255) / 255.f},

    {"brown", defaulttype::Vec4f(244, 164, 96, 255) / 255.f},
    {"chocolate", defaulttype::Vec4f(210, 105, 30, 255) / 255.f},
    {"wood", defaulttype::Vec4f(222, 184, 135, 255) / 255.f},

    {"slate gray", defaulttype::Vec4f(112, 128, 144, 255) / 255.f},
    {"grey", defaulttype::Vec4f(128, 128, 128, 255) / 255.f},
    {"silver", defaulttype::Vec4f(192, 192, 192, 255) / 255.f},

    {"black", defaulttype::Vec4f(0, 0, 0, 255) / 255.f},
};

void DrawTool::setColors()
{
    m_colors.clear();
    m_colors.insert(defaultColors.cbegin(), defaultColors.cend());
}

defaulttype::Vec4f DrawTool::getColor(const std::string colorName)
{
    return  m_colors[colorName];
}

helper::vector<std::string> DrawTool::GetDefaultColorNames()
{
    helper::vector<std::string> colorNames;
    for (const auto& p : defaultColors)
    {
        colorNames.push_back(p.first);
    }
    return colorNames;
}

} // namespace visual

} // namespace core

} // namespace sofa
