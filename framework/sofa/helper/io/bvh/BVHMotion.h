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
#ifndef SOFA_HELPER_IO_BVH_BVHMOTION_H
#define SOFA_HELPER_IO_BVH_BVHMOTION_H

#include <vector>
#include <sofa/SofaFramework.h>

namespace sofa
{

namespace helper
{

namespace io
{

namespace bvh
{

class SOFA_HELPER_API BVHMotion
{
public:
    BVHMotion()
    {
        frameCount = 0;
        frameTime = 0.0;
    };

    virtual ~BVHMotion()
    {
        delete[] &frames;
    };

    void init(double _fTime, unsigned int _fCount, unsigned int _fSize);

    int frameCount;
    double frameTime;
    std::vector< std::vector<double> > frames;

    void debug(void);
};

} // namespace bvh

} // namespace io

} // namespace helper

} // namespace sofa

#endif
