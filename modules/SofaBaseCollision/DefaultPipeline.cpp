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
#include <SofaBaseCollision/DefaultPipeline.h>

#include <sofa/core/CollisionModel.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>
#include <sofa/core/collision/BroadPhaseDetection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/core/collision/CollisionGroupManager.h>
#include <sofa/core/collision/ContactManager.h>
#include <sofa/core/visual/VisualParams.h>

#ifdef SOFA_DUMP_VISITOR_INFO
#include <sofa/simulation/common/Visitor.h>
#endif

#include <sofa/helper/system/gl.h>
#include <sofa/helper/AdvancedTimer.h>

#define VERBOSE(a) if (bVerbose.getValue()) a; else {}

namespace sofa
{

namespace component
{

namespace collision
{

using namespace core;
using namespace core::objectmodel;
using namespace core::collision;
using namespace sofa::defaulttype;

SOFA_DECL_CLASS(DefaultPipeline)

int DefaultPipelineClass = core::RegisterObject("The default collision detection and modeling pipeline")
        .add< DefaultPipeline >()
        .addAlias("CollisionPipeline")
        ;

DefaultPipeline::DefaultPipeline()
    : bVerbose(initData(&bVerbose, false, "verbose","Display current step information"))
    , bDrawEnabled(initData(&bDrawEnabled, false, "drawEnabled","Draw detected collisions"))
    , depth(initData(&depth, 6, "depth","Max depth of bounding trees"))
{
    addAlias(&bDrawEnabled, "draw");
}

void DefaultPipeline::init()
{
    Inherit1::init();

    this->getContext()->get(m_animationLoop);
    if (m_animationLoop)
    {
        m_animationLoop->registerSyncPoint("BeginBroadPhase");
        m_animationLoop->registerSyncPoint("BeginCollisionResponse");
    }
    else
        serr << "Animation loop not found" << sendl;
}

#ifdef SOFA_DUMP_VISITOR_INFO
typedef simulation::Visitor::ctime_t ctime_t;
#endif

void DefaultPipeline::doCollisionReset()
{
    VERBOSE(sout << "DefaultPipeline::doCollisionReset, Reset collisions"<<sendl);
    core::objectmodel::BaseContext* scene = getContext();
    // clear all contacts
    if (contactManager!=NULL)
    {
        const sofa::helper::vector<Contact::SPtr>& contacts = contactManager->getContacts();
        for (sofa::helper::vector<Contact::SPtr>::const_iterator it = contacts.begin(); it!=contacts.end(); ++it)
        {
            (*it)->resetResponse();
        }
    }
    // clear all collision groups
    if (groupManager!=NULL)
    {
        groupManager->clearGroups(scene);
    }
}

void DefaultPipeline::doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& collisionModels)
{
    sofa::helper::AdvancedTimer::stepBegin("doCollisionDetection");
    VERBOSE(sout << "DefaultPipeline::doCollisionDetection, Compute Bounding Trees"<<sendl);
    // First, we compute a bounding volume for the collision model (for example bounding sphere)
    // or we have loaded a collision model that knows its other model

    sofa::helper::vector<CollisionModel*> vectBoundingVolume;
    {
        sofa::helper::AdvancedTimer::stepBegin("BBox");
#ifdef SOFA_DUMP_VISITOR_INFO
        simulation::Visitor::printNode("ComputeBoundingTree");
#endif
        const bool continuous = intersectionMethod->useContinuous();
        const double dt       = getContext()->getDt();

        sofa::helper::vector<CollisionModel*>::const_iterator it;
        const sofa::helper::vector<CollisionModel*>::const_iterator itEnd = collisionModels.end();
        int nActive = 0;

        for (it = collisionModels.begin(); it != itEnd; ++it)
        {
            VERBOSE(sout << "DefaultPipeline::doCollisionDetection, consider model "<<(*it)->getName()<<sendl);
            if (!(*it)->isActive()) continue;

            int used_depth = broadPhaseDetection->needsDeepBoundingTree() ? depth.getValue() : 0;

            if (continuous)
                (*it)->computeContinuousBoundingTree(dt, used_depth);
            else
                (*it)->computeBoundingTree(used_depth);

            vectBoundingVolume.push_back ((*it)->getFirst());
            ++nActive;
        }
        sofa::helper::AdvancedTimer::stepEnd("BBox");
#ifdef SOFA_DUMP_VISITOR_INFO
        simulation::Visitor::printCloseNode("ComputeBoundingTree");
#endif
        VERBOSE(sout << "DefaultPipeline::doCollisionDetection, Computed "<<nActive<<" BBoxs"<<sendl);
    }

    if (m_animationLoop)
        m_animationLoop->doSyncPoint("BeginBroadPhase");

    // then we start the broad phase
    if (broadPhaseDetection==NULL) return; // can't go further
    VERBOSE(sout << "DefaultPipeline::doCollisionDetection, BroadPhaseDetection "<<broadPhaseDetection->getName()<<sendl);
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("BroadPhase");
#endif
    sofa::helper::AdvancedTimer::stepBegin("BroadPhase");
    intersectionMethod->beginBroadPhase();
    broadPhaseDetection->beginBroadPhase();
    broadPhaseDetection->addCollisionModels(vectBoundingVolume);  // detection is done there
    broadPhaseDetection->endBroadPhase();
    intersectionMethod->endBroadPhase();
    sofa::helper::AdvancedTimer::stepEnd  ("BroadPhase");

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("BroadPhase");
#endif

    // then we start the narrow phase
    if (narrowPhaseDetection==NULL) return; // can't go further
    VERBOSE(sout << "DefaultPipeline::doCollisionDetection, NarrowPhaseDetection "<<narrowPhaseDetection->getName()<<sendl);

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("NarrowPhase");
#endif
    sofa::helper::AdvancedTimer::stepBegin("NarrowPhase");
    intersectionMethod->beginNarrowPhase();
    narrowPhaseDetection->beginNarrowPhase();
    sofa::helper::vector<std::pair<CollisionModel*, CollisionModel*> >& vectCMPair = broadPhaseDetection->getCollisionModelPairs();
    VERBOSE(sout << "DefaultPipeline::doCollisionDetection, "<< vectCMPair.size()<<" colliding model pairs"<<sendl);
    narrowPhaseDetection->addCollisionPairs(vectCMPair);
    narrowPhaseDetection->endNarrowPhase();
    intersectionMethod->endNarrowPhase();
    sofa::helper::AdvancedTimer::stepEnd  ("NarrowPhase");

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("NarrowPhase");
#endif

    sofa::helper::AdvancedTimer::stepEnd("doCollisionDetection");
}

void DefaultPipeline::doCollisionResponse()
{
    if (m_animationLoop)
        m_animationLoop->doSyncPoint("BeginCollisionResponse");

    core::objectmodel::BaseContext* scene = getContext();
    // then we start the creation of contacts
    if (contactManager==NULL) return; // can't go further
    VERBOSE(sout << "Create Contacts "<<contactManager->getName()<<sendl);
    contactManager->createContacts(narrowPhaseDetection->getDetectionOutputs());

    // finally we start the creation of collisionGroup

    const sofa::helper::vector<Contact::SPtr>& contacts = contactManager->getContacts();

    if (groupManager==NULL)
    {
        VERBOSE(sout << "Linking all contacts to Scene"<<sendl);
        for (sofa::helper::vector<Contact::SPtr>::const_iterator it = contacts.begin(); it!=contacts.end(); ++it)
        {
            Contact* c = it->get();
            core::objectmodel::BaseContext* node = scene;
            if (!c->getCollisionModels().first->isSimulated())
            {
                node = c->getCollisionModels().second->getContext();
            }
            else if (!c->getCollisionModels().second->isSimulated())
            {
                node = c->getCollisionModels().first->getContext();
            }
            c->createResponse(node);
            c->computeResponse();
            c->finalizeResponse(node);
        }
    }
    else
    {
        // First we remove all contacts with non-simulated objects and directly add them
        sofa::helper::vector<Contact::SPtr> notStaticContacts;

        for (sofa::helper::vector<Contact::SPtr>::const_iterator it = contacts.begin(); it!=contacts.end(); ++it)
        {
            Contact::SPtr c = *it;
            if (!c->getCollisionModels().first->isSimulated())
            {
                c->createResponse(c->getCollisionModels().second->getContext());
                c->computeResponse();
                c->finalizeResponse(c->getCollisionModels().second->getContext());
            }
            else if (!c->getCollisionModels().second->isSimulated())
            {
                c->createResponse(c->getCollisionModels().first->getContext());
                c->computeResponse();
                c->finalizeResponse(c->getCollisionModels().first->getContext());
            }
            else
                notStaticContacts.push_back(c);
        }

        VERBOSE(sout << "Create Groups "<<groupManager->getName()<<sendl);
        groupManager->createGroups(scene, notStaticContacts);
    }
}

void DefaultPipeline::doRemoveContacts()
{
    if (contactManager != NULL)
        contactManager->removeContacts(contactManager->getInactiveContacts());
}

void DefaultPipeline::doUpdateMappers()
{
    if (contactManager == NULL)
        return;
    
    const sofa::helper::vector<Contact::SPtr>& contacts = contactManager->getContacts();

    if (groupManager==NULL)
    {
        for (sofa::helper::vector<Contact::SPtr>::const_iterator it = contacts.begin(); it!=contacts.end(); ++it)
        {
            Contact* c = it->get();
            c->updateContactsMappers();
        }
    }
    else
    {
        // First we remove all contacts with non-simulated objects and directly add them
        sofa::helper::vector<Contact::SPtr> notStaticContacts;

        for (sofa::helper::vector<Contact::SPtr>::const_iterator it = contacts.begin(); it!=contacts.end(); ++it)
        {
            Contact::SPtr c = *it;
            if (!c->getCollisionModels().first->isSimulated())
            {
                 c->updateContactsMappers();
            }
            else if (!c->getCollisionModels().second->isSimulated())
            {
                 c->updateContactsMappers();
            }
            else
                notStaticContacts.push_back(c);
        }
    }
}

helper::set< std::string > DefaultPipeline::getResponseList() const
{
    helper::set< std::string > listResponse;
    core::collision::Contact::Factory::iterator it;
    for (it=core::collision::Contact::Factory::getInstance()->begin(); it!=core::collision::Contact::Factory::getInstance()->end(); ++it)
    {
        listResponse.insert(it->first);
    }
    return listResponse;
}

void DefaultPipeline::draw(const core::visual::VisualParams* )
{
    if (!bDrawEnabled.getValue()) return;
    if (!narrowPhaseDetection) return;
#if 0
    glDisable(GL_LIGHTING);
    glLineWidth(2);
    glBegin(GL_LINES);
    DetectionOutputMap& outputsMap = narrowPhaseDetection->getDetectionOutputs();
    for (DetectionOutputMap::iterator it = outputsMap.begin(); it!=outputsMap.end(); it++)
    {
        /*
        DetectionOutputContainer& outputs = it->second.first;
        for (DetectionOutputContainer::iterator it2 = outputs.begin(); it2!=outputs.end(); it2++)
        {
            DetectionOutput* d = &*it2;
            if (d->distance<0)
                glColor3f(1.0f,0.5f,0.5f);
            else
                glColor3f(0.5f,1.0f,0.5f);
            glVertex3dv(d->point[0].ptr());
            glVertex3dv(d->point[1].ptr());
        }
        */
    }
    glEnd();
    glLineWidth(1);
#endif
}
} // namespace collision

} // namespace component

} // namespace sofa

