/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 MGH, INRIA, USTL, UJF, CNRS, InSimo                *
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
#ifndef SOFA_SIMULATION_DEFAULTANIMATIONLOOP_H
#define SOFA_SIMULATION_DEFAULTANIMATIONLOOP_H

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>
#include <sofa/core/ExecParams.h>
#include <sofa/SofaSimulation.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/helper/AdvancedTimer.h>

namespace sofa
{

namespace simulation
{

/**
 *  \brief Default Animation Loop to be created when no AnimationLoop found on simulation::node.
 *
 *
 */

class SOFA_SIMULATION_COMMON_API DefaultAnimationLoop : public sofa::core::behavior::BaseAnimationLoop
{
public:
    typedef sofa::core::behavior::BaseAnimationLoop Inherit;
    typedef sofa::core::objectmodel::BaseContext BaseContext;
    typedef sofa::core::objectmodel::BaseObjectDescription BaseObjectDescription;
    SOFA_CLASS(DefaultAnimationLoop,sofa::core::behavior::BaseAnimationLoop);
protected:
    DefaultAnimationLoop();

    virtual ~DefaultAnimationLoop();
public:
    /// Set the simulation node this animation loop is controlling
    virtual void setNode( simulation::Node* );

    /// Set the simulation node to the local context if not specified previously
    virtual void init();

    /// perform one animation step
    virtual void step(const core::ExecParams* params, double dt);

    /// Returns the desired frequency of calls to idle() when the simulation is stopped/paused
    ///
    /// The default is 0, indicating no call is made
    double getIdleFrequency() const override { return d_idleFrequency.getValue(); }

    /// Method called with the animation is paused, at the frequency given by getIdleFrequency(), if it is greater than 0
    void idle(const core::ExecParams* params) override;


    /// Construction method called by ObjectFactory.
    template<class T>
    static typename T::SPtr create(T* t, sofa::core::objectmodel::BaseContext* context, sofa::core::objectmodel::BaseObjectDescription* arg)
    {
        return sofa::core::behavior::BaseAnimationLoop::create(t, context, arg);
    }

protected :

    Data<double> d_idleFrequency; ///< Desired frequency of calls to idle() when the simulation is stopped/paused

    simulation::Node* gnode = nullptr;  ///< the node controlled by the loop

};

} // namespace simulation

} // namespace sofa

#endif  /* SOFA_SIMULATION_DEFAULTANIMATIONLOOP_H */
