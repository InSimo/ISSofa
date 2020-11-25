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
#define SOFA_COMPONENT_COLLISION_MOUSEINTERACTOR_CPP
#include <SofaUserInteraction/MouseInteractor.inl>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_ABSTRACT_CLASS_IMPL((BaseMouseInteractor));

SOFA_DECL_CLASS(MouseInteractor)

int MouseInteractorClass = core::RegisterObject("Perform tasks related to the interaction with the mouse")
#ifndef SOFA_DOUBLE
        .add< MouseInteractor<defaulttype::Vec2fTypes> >()
        .add< MouseInteractor<defaulttype::Vec3fTypes> >()
#endif
#ifndef SOFA_FLOAT
        .add< MouseInteractor<defaulttype::Vec2dTypes> >()
        .add< MouseInteractor<defaulttype::Vec3dTypes> >()
#endif
        ;
int MouseInteractorRigidClass = core::RegisterObject("Perform tasks related to the interaction with the mouse and rigid objects")
#ifndef SOFA_DOUBLE
        .add< MouseInteractor<defaulttype::Rigid3fTypes> >()
#endif
#ifndef SOFA_FLOAT
        .add< MouseInteractor<defaulttype::Rigid3dTypes> >()
#endif
        ;

#ifndef SOFA_DOUBLE
template class SOFA_USER_INTERACTION_API MouseInteractor<defaulttype::Vec2fTypes>;
template class SOFA_USER_INTERACTION_API MouseInteractor<defaulttype::Vec3fTypes>;
template class SOFA_USER_INTERACTION_API MouseInteractor<defaulttype::Rigid3fTypes>;

#endif
#ifndef SOFA_FLOAT
template class SOFA_USER_INTERACTION_API MouseInteractor<defaulttype::Vec2dTypes>;
template class SOFA_USER_INTERACTION_API MouseInteractor<defaulttype::Vec3dTypes>;
template class SOFA_USER_INTERACTION_API MouseInteractor<defaulttype::Rigid3dTypes>;
#endif


void BaseMouseInteractor::cleanup()
{
    while (!performers.empty())
    {
        removeInteractionPerformer(*performers.begin());
    }
    lastPicked=BodyPicked();
    if (m_bodyHighlighted)
    {
        m_bodyHighlighted->setColor4f(m_bodyHighlightedOriginalColor); // restore old color
        m_bodyHighlighted = nullptr;
    }
}


void BaseMouseInteractor::handleEvent(core::objectmodel::Event *e)
{
    VecPerformer::iterator it=performers.begin(), it_end=performers.end();
    for (; it!=it_end; ++it)
    {
        (*it)->handleEvent(e);
    }
    if (core::objectmodel::KeypressedEvent* keypressed = core::objectmodel::KeypressedEvent::DynamicCast(e))
    {
        switch (keypressed->getKey())
        {
        case 'C':
            for (auto body : m_taggedBodies)
            {
                if (body)
                {
                    std::cout << "Enable picking on model " << body->getName() << std::endl;
                    body->removeTag(core::objectmodel::Tag("NoPicking"));
                }
            }
            m_taggedBodies.clear();
            break;
        case 'F':
            if (lastPicked.body)
            {
                std::cout << "Disable picking on model " << lastPicked.body->getName() << std::endl;
                lastPicked.body->addTag(core::objectmodel::Tag("NoPicking"));
                m_taggedBodies.push_back(lastPicked.body);
            }
            break;
        case 'E':
            for (auto body : m_disabledBodies)
            {
                if (body)
                {
                    std::cout << "Enable model " << body->getName() << std::endl;
                    body->setActive(true);
                }
            }
            m_disabledBodies.clear();
            break;
        case 'D':
            if (lastPicked.body)
            {
                std::cout << "Disable model " << lastPicked.body->getName() << std::endl;
                lastPicked.body->setActive(false);
                m_disabledBodies.push_back(lastPicked.body);
            }
            break;
        default:
            break;
        }
    }
}

void BaseMouseInteractor::addInteractionPerformer( InteractionPerformer *perf)
{
    performers.insert(performers.end(),perf);
}

bool BaseMouseInteractor::removeInteractionPerformer( InteractionPerformer *i)
{
    VecPerformer::iterator found=std::find(performers.begin(), performers.end(), i);
    if (found == performers.end()) return false;
    else
    {
//            delete *found; //Only remove the Performer from the Interactor, do not delete it!
        performers.erase(found);
        return true;
    }
}

void BaseMouseInteractor::updatePosition( double )
{
    VecPerformer::iterator it=performers.begin(), it_end=performers.end();
    for (; it!=it_end; ++it)
    {
        (*it)->execute();
    }
}



void BaseMouseInteractor::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL
    VecPerformer::iterator it=performers.begin(), it_end=performers.end();
    for (; it!=it_end; ++it)
        (*it)->draw(vparams);

    if( !vparams->isSupported(sofa::core::visual::API_OpenGL) ) return;

    if (lastPicked.body)
    {
        {
            if (m_bodyHighlighted != lastPicked.body)
            {
                if (m_bodyHighlighted)
                {
                    // restore old color
                    m_bodyHighlighted->setColor4f(m_bodyHighlightedOriginalColor);
                }
                m_bodyHighlighted = lastPicked.body;
                const float* originalColor = m_bodyHighlighted->getColor4f();
                // save new original color
                for (int i = 0; i < 4; ++i)
                    m_bodyHighlightedOriginalColor[i] = originalColor[i];

                // set collision model color to selection color (orange)
                m_bodyHighlighted->setColor4f(defaulttype::Vec4f(255.f, 69.f, 0.f, originalColor[3]).array());
            }
            std::vector<defaulttype::Vec3d> points = { lastPicked.point };
            defaulttype::Vec4f color = (isAttached) ? defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 0.5f) : defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 0.5f);
            vparams->drawTool()->drawPoints(points, 10.f, color);
        }
    }
#endif /* SOFA_NO_OPENGL */
}
}
}
}
