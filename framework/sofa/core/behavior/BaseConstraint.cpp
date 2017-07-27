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
#include "BaseConstraintSet.h"
#include "BaseConstraint.h"
#include "BaseInteractionConstraint.h"

namespace sofa
{

namespace core
{

namespace behavior
{

ConstraintResolution::ConstraintResolution(unsigned int nbLines, double tolerance)
:m_nbLines(nbLines)
,m_tolerance(tolerance)
{
}

void ConstraintResolution::init(int /*line*/, double** /*w*/, double* /*force*/)
{

}

void ConstraintResolution::initForce(int /*line*/, double* /*force*/)
{

}

SOFA_ABSTRACT_CLASS_IMPL((BaseConstraintSet));
SOFA_ABSTRACT_CLASS_IMPL((BaseConstraint));
SOFA_ABSTRACT_CLASS_IMPL((BaseInteractionConstraint));


} // namespace behavior

} // namespace core

} // namespace sofa
