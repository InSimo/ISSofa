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
#include <sofa/core/behavior/BaseAnimationLoop.h>

#include <stdlib.h>
#include <math.h>

namespace sofa
{

namespace core
{

namespace behavior
{

SOFA_ABSTRACT_CLASS_IMPL((BaseAnimationLoop));

BaseAnimationLoop::BaseAnimationLoop()
    : m_resetTime(0.)
    , d_exit(initData(&d_exit, false, "exit", "If set to true, gui will stop execution"))
{}

BaseAnimationLoop::~BaseAnimationLoop()
{}

void BaseAnimationLoop::storeResetState()
{
    const objectmodel::BaseContext * c = this->getContext();

    if (c != 0)
        m_resetTime = c->getTime();
}

double BaseAnimationLoop::getResetTime() const
{
    return m_resetTime;
}

BaseAnimationLoop::SyncPointWorkRegisterID BaseAnimationLoop::registerSyncPointWork(SyncPointID /*predecessorID*/, SyncPointID /*successorID*/, SyncPointWorkFunctor /*work*/, std::string /*taskName*/)
{
    return nullptr;
}

BaseAnimationLoop::SyncPointWorkRegisterID BaseAnimationLoop::registerSyncPointSeqWork(SyncPointID /*syncPointID*/, SyncPointWorkFunctor /*work*/, std::string /*taskName*/, defaulttype::Vec4f /*color*/)
{
    return nullptr;
}

BaseAnimationLoop::SyncPointWorkRegisterID BaseAnimationLoop::registerSyncPointWork(SyncPointID /*syncPointID*/, SyncPointWorkFunctor /*work*/, std::string /*taskName*/)
{
    return nullptr;
}

bool BaseAnimationLoop::unregisterSyncPointWork(SyncPointWorkRegisterID /*registerID*/)
{
    return false;
}

} // namespace behavior

} // namespace core

} // namespace sofa

