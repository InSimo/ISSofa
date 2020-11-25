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
#ifndef SOFA_HELPER_IO_MESHTRIAN_H
#define SOFA_HELPER_IO_MESHTRIAN_H

#include <sofa/helper/io/Mesh.h>
#include <sofa/SofaFramework.h>

namespace sofa
{

namespace helper
{

namespace io
{

/// Cette classe permet la fabrication d'un visuel pour un fichier de type trian
/// ces fichiers se presentent de la maniere suivante
/// nombre de sommets
///liste des coordonnees des sommets ex 1.45 1.25 6.85
/// nombre de faces
///liste de toutes les faces ex 1 2 3 0 0 0 les 3 derniers chiffres ne sont pas utilises pour le moment

// WARNING: Mesh is dependent of sofa::core classes, and is compiled in core
// It is in the helper folder and namespace to stay backward compatible
class SOFA_CORE_API MeshTrian : public Mesh
{
private:

    void readTrian(FILE *file);

public:

    MeshTrian(const std::string& filename)
    {
        init(filename);
    }

    void init(std::string filename);
};

} // namespace io

} // namespace helper

} // namespace sofa

#endif
