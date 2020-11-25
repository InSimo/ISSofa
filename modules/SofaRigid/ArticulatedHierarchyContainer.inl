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

#ifndef SOFA_COMPONENT_CONTAINER_ARTICULATEDHIERARCHYCONTAINER_INL
#define SOFA_COMPONENT_CONTAINER_ARTICULATEDHIERARCHYCONTAINER_INL

#include <SofaRigid/ArticulatedHierarchyContainer.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/simulation/common/Simulation.h>

namespace sofa
{

namespace component
{

namespace container
{

Articulation::Articulation():
    axis(initData(&axis, defaulttype::Vector3(1,0,0), "axis", "Set the rotation axis for the articulation")),
    rotation(initData(&rotation, (bool) false, "rotation", "Rotation")),
    translation(initData(&translation, (bool) false, "translation", "Translation")),
    articulationIndex(initData(&articulationIndex, (int) 0, "articulationIndex", "Articulation index"))
{
    this->addAlias(&axis, "rotationAxis");
}

ArticulationCenter::ArticulationCenter():
    parentIndex(initData(&parentIndex, "parentIndex", "Parent of the center articulation")),
    childIndex(initData(&childIndex, "childIndex", "Child of the center articulation")),
    globalPosition(initData(&globalPosition, "globalPosition", "Global position of the articulation center")),
    posOnParent(initData(&posOnParent, "posOnParent", "Parent position of the articulation center")),
    posOnChild(initData(&posOnChild, "posOnChild", "Child position of the articulation center")),
    articulationProcess(initData(&articulationProcess, (int) 0, "articulationProcess", " 0 - (default) hierarchy between articulations (euler angles)\n 1- ( on Parent) no hierarchy - axis are attached to the parent\n 2- (attached on Child) no hierarchy - axis are attached to the child"))
{
}

ArticulationCenter* ArticulatedHierarchyContainer::getArticulationCenterAsChild(int index, vector<ArticulationCenter*> acs)
{
    for (ArticulationCenter* ac : acs)
    {
        if (ac->childIndex.getValue() == index)
            return ac;
    }
    return NULL;
}

vector<ArticulationCenter*> ArticulatedHierarchyContainer::getAscendantList(int index, vector<ArticulationCenter*> acs)
{
    // We consider that bifurcation IS NOT possible in the ascendant list
    unsigned int maxSize = acs.size();
    ascendantList.clear();
    for (unsigned int i = 0; i < maxSize; i++)
    {
        ArticulationCenter* ac_parent = getArticulationCenterAsChild(index, acs);
        if (ac_parent != NULL)
        {
            ascendantList.push_back(ac_parent);
            index = ac_parent->parentIndex.getValue();
        }
        else
        {
            return ascendantList;
        }
    }
    return ascendantList;
}

vector<ArticulationCenter*> ArticulatedHierarchyContainer::getArticulationCenterAsParent(int index, vector<ArticulationCenter*> acs)
{
    vector<ArticulationCenter*> list;
    for (ArticulationCenter* ac : acs)
    {
        if (ac->parentIndex.getValue() == index)
        {
            list.push_back(ac);
        }
    }
    return list;
}

vector<ArticulationCenter*> ArticulatedHierarchyContainer::getDescendantList(int index, vector<ArticulationCenter*> acs)
{
    // We consider that bifurcation IS possible in the descendant list
    unsigned int maxSize = acs.size();
    vector<int> indices;
    indices.push_back(index);
    descendantList.clear();
    for (unsigned int i = 0; i < maxSize; i++)
    {
        vector<ArticulationCenter*> ac_childs = getArticulationCenterAsParent(indices[i], acs);
        if (ac_childs.size() > 0)
        {
            for (ArticulationCenter* ac_child : ac_childs)
            {
                descendantList.push_back(ac_child);
                int childIndex = ac_child->childIndex.getValue();
                indices.push_back(childIndex); 
            }
        }
    }
    return descendantList;
}

void ArticulatedHierarchyContainer::swapArticulations(vector<ArticulationCenter*> acs)
{
    for (ArticulationCenter* ascendant : acs)
    {
        //swap index
        const int childIndex = ascendant->childIndex.getValue();
        const int parentIndex = ascendant->parentIndex.getValue();

        ascendant->childIndex.setValue(parentIndex);
        ascendant->parentIndex.setValue(childIndex);

        //swap local position
        const defaulttype::Vector3 posOnChild = ascendant->posOnChild.getValue();
        const defaulttype::Vector3 posOnParent = ascendant->posOnParent.getValue();

        ascendant->posOnChild.setValue(posOnParent);
        ascendant->posOnParent.setValue(posOnChild);
    }
}

ArticulatedHierarchyContainer::ArticulatedHierarchyContainer()
    : d_rootOutIndex(initData(&d_rootOutIndex, int(-1), "rootOutIndex", "(Optional) Define the out index on which the m_fromRootModel is attached in ArticulatedSystemMapping."))
    , filename(initData(&filename, "filename", "BVH File to load the articulation", false))
{
    joint = NULL;
    id = 0;
    chargedFromFile = false;
    numOfFrames = 0;
    dtbvh = 0.0;
}


void ArticulatedHierarchyContainer::buildCenterArticulationsTree(sofa::helper::io::bvh::BVHJoint* bvhjoint, int id_buf, const char* name, simulation::Node* node)
{
    std::vector<sofa::helper::io::bvh::BVHJoint*> jointChildren = bvhjoint->getChildren();
    if (jointChildren.size()==0)
        return;

    std::string str(name);
    str.append("/");
    str.append(bvhjoint->getName());

    simulation::Node::SPtr nodeOfArticulationCenters =node->createChild(str);

    ArticulationCenter::SPtr ac = sofa::core::objectmodel::New<ArticulationCenter>();
    nodeOfArticulationCenters->addObject(ac);
    articulationCenters.push_back(ac.get());

    ac->posOnParent.setValue(defaulttype::Vector3(bvhjoint->getOffset()->x,bvhjoint->getOffset()->y,bvhjoint->getOffset()->z)); //
    ac->posOnChild.setValue(defaulttype::Vector3(0,0,0));
    ac->parentIndex.setValue(id_buf);
    ac->childIndex.setValue(bvhjoint->getId()+1);

    simulation::Node::SPtr nodeOfArticulations = nodeOfArticulationCenters->createChild("articulations");

    sofa::helper::io::bvh::BVHChannels* channels = bvhjoint->getChannels();
    sofa::helper::io::bvh::BVHMotion* motion = bvhjoint->getMotion();

    sout<<"num Frames found in BVH ="<<motion->frameCount<<sendl;

    Articulation::SPtr a;

    for (unsigned int j=0; j<channels->channels.size(); j++)
    {
        switch(channels->channels[j])
        {
        case sofa::helper::io::bvh::BVHChannels::NOP:
            break;
        case sofa::helper::io::bvh::BVHChannels::Xposition:
            a = sofa::core::objectmodel::New<Articulation>();
            nodeOfArticulations->addObject(a);
            ac->articulations.push_back(a.get());
            a->axis.setValue(defaulttype::Vector3(1,0,0));
            a->translation.setValue(true);
            a->articulationIndex.setValue(id);
            for (int k=0; k<motion->frameCount; k++)
                a->motion.push_back(motion->frames[k][j]);
            id++;
            break;
        case sofa::helper::io::bvh::BVHChannels::Yposition:
            a = sofa::core::objectmodel::New<Articulation>();
            nodeOfArticulations->addObject(a);
            ac->articulations.push_back(a.get());
            a->axis.setValue(defaulttype::Vector3(0,1,0));
            a->translation.setValue(true);
            a->articulationIndex.setValue(id);
            for (int k=0; k<motion->frameCount; k++)
                a->motion.push_back(motion->frames[k][j]);
            id++;
            break;
        case sofa::helper::io::bvh::BVHChannels::Zposition:
            a = sofa::core::objectmodel::New<Articulation>();
            nodeOfArticulations->addObject(a);
            ac->articulations.push_back(a.get());
            a->axis.setValue(defaulttype::Vector3(0,0,1));
            a->translation.setValue(true);
            a->articulationIndex.setValue(id);
            for (int k=0; k<motion->frameCount; k++)
                a->motion.push_back(motion->frames[k][j]);
            id++;
            break;
        case sofa::helper::io::bvh::BVHChannels::Xrotation:
            a = sofa::core::objectmodel::New<Articulation>();
            nodeOfArticulations->addObject(a);
            ac->articulations.push_back(a.get());
            a->axis.setValue(defaulttype::Vector3(1,0,0));
            a->rotation.setValue(true);
            a->articulationIndex.setValue(id);
            for (int k=0; k<motion->frameCount; k++)
                a->motion.push_back(motion->frames[k][j]);
            id++;
            break;
        case sofa::helper::io::bvh::BVHChannels::Yrotation:
            a = sofa::core::objectmodel::New<Articulation>();
            nodeOfArticulations->addObject(a);
            ac->articulations.push_back(a.get());
            a->axis.setValue(defaulttype::Vector3(0,1,0));
            a->rotation.setValue(true);
            a->articulationIndex.setValue(id);
            for (int k=0; k<motion->frameCount; k++)
                a->motion.push_back(motion->frames[k][j]);
            id++;
            break;
        case sofa::helper::io::bvh::BVHChannels::Zrotation:
            a = sofa::core::objectmodel::New<Articulation>();
            nodeOfArticulations->addObject(a);
            ac->articulations.push_back(a.get());
            a->axis.setValue(defaulttype::Vector3(0,0,1));
            a->rotation.setValue(true);
            a->articulationIndex.setValue(id);
            for (int k=0; k<motion->frameCount; k++)
                a->motion.push_back(motion->frames[k][j]);
            id++;
            break;
        }
    }

    for(unsigned int i=0; i<jointChildren.size(); i++)
    {
        buildCenterArticulationsTree(jointChildren[i], bvhjoint->getId()+1, bvhjoint->getName(), nodeOfArticulationCenters.get());
    }
}

void ArticulatedHierarchyContainer::init ()
{
    simulation::Node* context = simulation::Node::DynamicCast(this->getContext()); // access to current node

    std::string file = filename.getFullPath();
    if ( sofa::helper::system::DataRepository.findFile (file) )
    {

        sofa::helper::io::bvh::BVHLoader loader = sofa::helper::io::bvh::BVHLoader();
        joint = loader.load(sofa::helper::system::DataRepository.getFile ( file ).c_str());
        chargedFromFile = true;
        numOfFrames = joint->getMotion()->frameCount;
        dtbvh = joint->getMotion()->frameTime;
    }

    if (joint != NULL)
    {
        simulation::Node::SPtr articulationCenters = context->createChild("ArticulationCenters");

        buildCenterArticulationsTree(joint, 0, "Root", articulationCenters.get());

        component::container::MechanicalObject<defaulttype::Vec1Types>* mm1 = component::container::MechanicalObject<defaulttype::Vec1Types>::DynamicCast(context->getMechanicalState());
        mm1->resize(id);

        context = (context->child.begin())->get();
        component::container::MechanicalObject<defaulttype::RigidTypes>* mm2 = component::container::MechanicalObject<defaulttype::RigidTypes>::DynamicCast(context->getMechanicalState());
        mm2->resize(joint->getNumJoints()+1);
    }
    else
    {
        vector<ArticulationCenter*> unclassifiedArticulationCenters, classifiedArticulationCenters;
        context->getTreeObjects<ArticulationCenter>(&unclassifiedArticulationCenters);
        
        vector<ArticulationCenter*>::const_iterator uac = unclassifiedArticulationCenters.begin();
        vector<ArticulationCenter*>::const_iterator uacEnd = unclassifiedArticulationCenters.end();

        const int rootOutIndex = d_rootOutIndex.getValue();
        if (rootOutIndex >= 0)
        {
            // re-organize articulations according to (optional) "m_fromRootModel" defined in ArticulatedSystemMapping
            // swap articulations (child <-> parent) of ascendants
            vector<ArticulationCenter*> ascendantList = getAscendantList(rootOutIndex, unclassifiedArticulationCenters);
            swapArticulations(ascendantList);
        }

        //Find root parent
        int rootParent = 0;
        for (; uac != uacEnd; uac++)
        {
            int parent = (*uac)->parentIndex.getValue();
            if (getAscendantList(parent, unclassifiedArticulationCenters).size() == 0)
            {
                rootParent = parent;
                break;
            }
        }

        //Starting from root parent, get the descendant articulation centers
        if (uac != uacEnd)
        {
            classifiedArticulationCenters = getDescendantList(rootParent, unclassifiedArticulationCenters);
        }
        
        if (classifiedArticulationCenters.size() == unclassifiedArticulationCenters.size())
        {
            articulationCenters = classifiedArticulationCenters;
        }
        else
        {
            articulationCenters = unclassifiedArticulationCenters;
            this->serr << "Articulations Centers are not well classified." << this->sendl;
        }

        vector<ArticulationCenter*>::const_iterator ac = articulationCenters.begin();
        vector<ArticulationCenter*>::const_iterator acEnd = articulationCenters.end();
        for (; ac != acEnd; ac++)
        {
            context = simulation::Node::DynamicCast((*ac)->getContext());
            for (simulation::Node::ChildIterator it = context->child.begin(); it != context->child.end(); ++it)
            {
                simulation::Node* n =  it->get();
                n->getTreeObjects<Articulation>(&(*ac)->articulations);
            }

            // for Arboris Mapping, init the transformation for each articulation center
            defaulttype::Quat q; // TODO: add a rotation component to the positionning on the ArticulatedHierarchyContainer
            (*ac)->H_p_pLc.set((*ac)->posOnParent.getValue(),q);
            (*ac)->H_c_cLp.set((*ac)->posOnChild.getValue(), q);
            (*ac)->H_pLc_cLp.identity();

        }
    }
}

void ArticulatedHierarchyContainer::reset()
{
    init();
}




} // namespace container

} // namespace component

} // namespace sofa

#endif
