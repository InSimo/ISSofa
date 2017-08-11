/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <sofa/simulation/tree/ExportDotVisitor.h>
#include <sofa/helper/system/config.h>
#include <sofa/helper/Factory.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/Colors.h>
#include <sofa/core/topology/TopologicalMapping.h>
#include <sofa/core/topology/BaseTopologyObject.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/behavior/BaseInteractionConstraint.h>
#include <sofa/core/behavior/BaseController.h>

#include <sofa/core/collision/CollisionGroupManager.h>
#include <sofa/core/collision/ContactManager.h>

namespace sofa
{

namespace simulation
{

namespace tree
{

using sofa::core::visual::tristate;
using sofa::core::objectmodel::Base;

ExportDotVisitor::ExportDotVisitor(const sofa::core::ExecParams* params /* PARAMS FIRST */, std::ostream* out)
    : GNodeVisitor(params),
      out(out),
      showNode(true),
      showObject(true),
      showSlaves(true),
      showSolverGroups(true),
      showBehaviorModel(true),
      showCollisionModel(true),
      showVisualModel(true),
      showVisualObject(true),
      showMapping(true),
      showContext(true),
      showEngine(true),
      showLoader(true),
      showCollisionPipeline(true),
      showAnimationLoop(true),
      showController(true),
      showSolver(true),
      showMechanicalState(true),
      showForceField(true),
      showInteractionForceField(true),
      showConstraint(true),
      showProjectiveConstraint(true),
      showMass(true),
      showTopology(true),
      showTopologyObject(true),
      showTopologicalMapping(true),
      showMechanicalMapping(true),
      showOthers(true),
      labelNodeName(true),
      labelNodeClass(false),
      labelObjectName(true),
      labelObjectClass(true),
      first(true)
{
    excludeNames.insert("NoExportGraph");
}

ExportDotVisitor::~ExportDotVisitor()
{
    if (!first)
    {
        *out << olinks.str();
        *out << "}" << std::endl;
    }
}

namespace
{
void acc_favor_true(tristate& lhs, const tristate& rhs)
{
    if (rhs != tristate::neutral_value && rhs != lhs)
    {
        if (lhs == tristate::neutral_value) lhs = rhs;
        else lhs = tristate::true_value; // one is false, the other is true
    }
}
bool strPatternMatch(const std::string& pattern, const std::string& s)
{
    if (pattern.empty())
    {
        return s.empty();
    }
    else if (pattern.back() != '*')
    {
        return s == pattern;
    }
    else if (s[0] != pattern[0]) // quick way out
    {
        return false;
    }
    else if (s.length() == pattern.length()-1)
    {
        return s == pattern.substr(0, pattern.length()-1);
    }
    else if (s.length() >= pattern.length())
    {
        return s.substr(0, pattern.length()-1) == pattern.substr(0, pattern.length()-1);
    }
    else
    {
        return false;
    }
}
}

tristate ExportDotVisitor::isIncludedOrExcludedStr(const std::string& s)
{
    for (const std::string& s2 : includeNames)
    {
        if (strPatternMatch(s2, s))
        {
            return tristate::true_value;
        }
    }
    for (const std::string& s2 : excludeNames)
    {
        if (strPatternMatch(s2, s))
        {
            return tristate::false_value;
        }
    }
    return tristate::neutral_value;
}

tristate ExportDotVisitor::isIncludedOrExcludedBase(sofa::core::objectmodel::Base* b)
{
    tristate current(tristate::neutral_value);
    acc_favor_true(current, isIncludedOrExcludedStr(b->getName()));
    acc_favor_true(current, isIncludedOrExcludedStr(b->getClassName()));
    for(const sofa::core::objectmodel::Tag& t : b->getTags())
    {
        if (!t.negative())
        {
            acc_favor_true(current, isIncludedOrExcludedStr(t));
        }
    }
    return current;
}

tristate ExportDotVisitor::isIncludedOrExcluded(sofa::core::objectmodel::BaseObject* b)
{
    tristate current = isIncludedOrExcludedBase(b);
    if (auto n = sofa::core::objectmodel::BaseNode::DynamicCast(b->getContext()))
    {
        acc_favor_true(current, isIncludedOrExcludedStr(n->getPathName() + std::string("/") + b->getName()));
    }
    return current;
}

tristate ExportDotVisitor::isIncludedOrExcluded(GNode* b)
{
    tristate current = isIncludedOrExcludedBase(b);
    acc_favor_true(current, isIncludedOrExcludedStr(b->getPathName()));
    return current;
}

/// Test if a node should be displayed
bool ExportDotVisitor::display(GNode* node, const char **color)
{
    using namespace Colors;
    if (!node) return false;
    if (color) *color = COLOR[NODE];
    tristate filtered = isIncludedOrExcluded(node);
    if (!showNode)
    {
        return false;
    }
    else if (filtered != tristate::neutral_value)
    {
        return filtered;
    }
    else
    {
        return true;
    }
}

/// Test if an object should be displayed
bool ExportDotVisitor::display(core::objectmodel::BaseObject* obj, const char **color)
{
    using namespace Colors;
    const char* c = NULL;
    if (color==NULL) color=&c;
    if (!obj) return false;
    *color = COLOR[OBJECT];
    tristate filtered = isIncludedOrExcluded(obj);
    if (!showObject) return filtered;
    bool show = false;
    bool hide = false;
    if (core::visual::VisualModel::DynamicCast(obj))
    {
        *color = COLOR[VMODEL];
    }
    if (core::behavior::BaseMechanicalState::DynamicCast(obj))
    {
        *color = COLOR[MMODEL];
        if (showMechanicalState) { show = true; }
        else hide = true;
    }
    if (core::topology::Topology::DynamicCast(obj))
    {
        *color = COLOR[TOPOLOGY];
        if (showTopology) { show = true; }
        else hide = true;
    }
    if (core::CollisionModel::DynamicCast(obj))
    {
        *color = COLOR[CMODEL];
        if (showCollisionModel) { show = true; }
        else hide = true;
    }
    core::BaseMapping* bm = core::BaseMapping::DynamicCast(obj);
    bool toModelVisible = false;
    if (bm)
    {
        for (core::objectmodel::BaseObject* model2 : bm->getTo())
        {
            if (display(model2))
            {
                toModelVisible = true;
                break;
            }
        }
    }
    if (bm && !bm->isMechanical())
    {
        *color = COLOR[MAPPING];
        if (showMapping && toModelVisible) { show = true; }
        else hide = true;
    }
    if (bm && bm->isMechanical())
    {
        *color = COLOR[MMAPPING];
        if (showMechanicalMapping && toModelVisible) { show = true; }
        else hide = true;
    }
    if (core::topology::BaseTopologyObject::DynamicCast(obj))
    {
        *color = COLOR[TOPOOBJECT];
        if (showTopologyObject) { show = true; }
        else hide = true;
    }
    if (core::topology::TopologicalMapping::DynamicCast(obj))
    {
        *color = COLOR[TOPOMAPPING];
        if (showTopologicalMapping) { show = true; }
        else hide = true;
    }
    if (core::objectmodel::ContextObject::DynamicCast(obj))
    {
        *color = COLOR[CONTEXT];
        if (showContext) { show = true; }
        else hide = true;
    }
    if (core::DataEngine::DynamicCast(obj))
    {
        *color = COLOR[ENGINE];
        if (showEngine) { show = true; }
        else hide = true;
    }
    if (core::loader::BaseLoader::DynamicCast(obj))
    {
        *color = COLOR[LOADER];
        if (showLoader) { show = true; }
        else hide = true;
    }
    if (core::collision::Pipeline::DynamicCast(obj)
        || core::collision::Intersection::DynamicCast(obj)
        || core::collision::Detection::DynamicCast(obj)
        || core::collision::ContactManager::DynamicCast(obj)
        || core::collision::CollisionGroupManager::DynamicCast(obj))
    {
        *color = COLOR[COLLISION];
        if (showCollisionPipeline) { show = true; }
        else hide = true;
    }
    if (core::behavior::OdeSolver::DynamicCast(obj)
        || core::behavior::LinearSolver::DynamicCast(obj))
    {
        *color = COLOR[SOLVER];
        if (showSolver) { show = true; }
        else hide = true;
    }
    if (core::behavior::BaseInteractionForceField::DynamicCast(obj) &&
        core::behavior::BaseInteractionForceField::DynamicCast(obj)->getMechModel1()!=core::behavior::BaseInteractionForceField::DynamicCast(obj)->getMechModel2())
    {
        *color = COLOR[IFFIELD];
        if (showInteractionForceField) { show = true; }
        else hide = true;
    }
    else if (core::behavior::BaseForceField::DynamicCast(obj))
    {
        *color = COLOR[FFIELD];
        if (showForceField) { show = true; }
        else hide = true;
    }
    if (core::behavior::BaseMass::DynamicCast(obj))
    {
        *color = COLOR[MASS];
        if (showMass) { show = true; }
        else hide = true;
    }
    if (core::behavior::BaseProjectiveConstraintSet::DynamicCast(obj))
    {
        *color = COLOR[PROJECTIVECONSTRAINTSET];
        if (showProjectiveConstraint) { show = true; }
        else hide = true;
    }
    if (core::behavior::BaseConstraintSet::DynamicCast(obj))
    {
        *color = COLOR[CONSTRAINTSET];
        if (showConstraint) { show = true; }
        else hide = true;
    }
    if (core::BehaviorModel::DynamicCast(obj))
    {
        *color = COLOR[BMODEL];
        if (showBehaviorModel) { show = true; }
        else hide = true;
    }
    if (core::behavior::BaseController::DynamicCast(obj))
    {
        *color = COLOR[BMODEL];
        if (showController) { show = true; }
        else hide = true;
    }
    if (core::behavior::BaseAnimationLoop::DynamicCast(obj))
    {
        *color = COLOR[SOLVER];
        if (showAnimationLoop) { show = true; }
        else hide = true;
    }
    if (core::behavior::ConstraintSolver::DynamicCast(obj))
    {
        *color = COLOR[SOLVER];
        if (showAnimationLoop && (showSolver || showConstraint)) { show = true; }
        else hide = true;
    }
    
    if (core::visual::VisualModel::DynamicCast(obj) && core::BaseState::DynamicCast(obj))
    {
        if (showVisualModel) { show = true; }
        else hide = true;
    }
    
    if (core::visual::VisualModel::DynamicCast(obj) && !show && !hide)
    {
        if (showVisualObject) { show = true; }
        else hide = true;
    }

    if (!show && !hide)
    {
        if (showOthers) { show = true; }
        else hide = true;
    }

    if (filtered != tristate::neutral_value)
    {
        return filtered;
    }
    else
    {
        return show || !hide;
    }
}

/// Find the node or object a given object should be attached to.
/// This is the parent node if it is displayed, otherwise it is the attached MechanicalState or Solver.
/// Returns an empty string if not found.
std::string ExportDotVisitor::getParentName(core::objectmodel::BaseObject* obj)
{
    if (!obj) return "";
    core::objectmodel::BaseObject* master = obj->getMaster();
    if (showSlaves && master && display(master))
        return getName(master);
    GNode* node = GNode::DynamicCast(obj->getContext());
    if (!node) return "";
    bool isMechanicalState = !node->mechanicalState.empty() && node->mechanicalState==obj;
    bool isState = core::BaseState::DynamicCast(obj);
    while (node)
    {
        if (display(node))
            return getName(node);
        if (core::BaseMapping::DynamicCast(obj))
            return "";
        if (core::topology::TopologicalMapping::DynamicCast(obj))
            return "";
        if (!node->collisionPipeline.empty() && display(node->collisionPipeline) &&
            (core::collision::Intersection::DynamicCast(obj) ||
             core::collision::Detection::DynamicCast(obj) ||
             core::collision::ContactManager::DynamicCast(obj) ||
             core::collision::CollisionGroupManager::DynamicCast(obj)))
            return getName(node->collisionPipeline);
        if (!node->topology.empty() && node->topology!=obj && display(node->topology) &&
            core::topology::BaseTopologyObject::DynamicCast(obj))
            return getName(node->topology);
        /// \todo consider all solvers instead of the first one (FF)
        if (!isMechanicalState && !node->mechanicalState.empty() && node->mechanicalState!=obj && node->linearSolver[0]!=obj && node->solver[0]!=obj  && node->animationManager!=obj && display(node->mechanicalState))
            return getName(node->mechanicalState);
        if (!isState && !node->state.empty() && node->state!=obj && node->linearSolver[0]!=obj && node->solver[0]!=obj  && node->animationManager!=obj && display(node->state))
            return getName(node->state);
        if (!node->linearSolver.empty() && node->linearSolver[0]!=obj && node->solver[0]!=obj && node->animationManager!=obj && display(node->linearSolver[0]))
            return getName(node->linearSolver[0]);
        if (!node->solver.empty() && node->solver[0]!=obj && node->animationManager!=obj && display(node->solver[0]))
            return getName(node->solver[0]);
        if (!node->animationManager.empty() && node->animationManager!=obj && display(node->animationManager))
            return getName(node->animationManager);
        if (isMechanicalState && !node->mechanicalMapping.empty())
            break;
        if (isState && !node->mapping.empty())
            break;
        node = node->parent();
    }
    return "";
}

/// Compute the name of a given node or object
std::string ExportDotVisitor::getName(core::objectmodel::Base* o, std::string prefix)
{
    if (!o) return "";
    if (names.count(o)>0)
        return names[o];
    std::string oname = o->getName();
    std::string name = prefix;
    for (unsigned i = 0; i<oname.length(); i++)
    {
        char c = oname[i];
        static const char *chars = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
        if (strchr(chars, c))
            name += c;
    }
    if (name.length() > prefix.length())
        name += '_';
    int index = nextIndex[name]++;
    if (index)
    {
        char str[16]={"azertyazertyaze"};
        snprintf(str,sizeof(str),"%d",index+1);
        name += str;
    }
    names[o] = name;
    return name;
}

/// Compute the name of a given node
std::string ExportDotVisitor::getName(GNode* node)
{
    return getName(node, "n_");
}

/// Compute the name of a given object
std::string ExportDotVisitor::getName(core::objectmodel::BaseObject* obj)
{
    return getName(obj, "o_");
}

void ExportDotVisitor::processObject(GNode* node, core::objectmodel::BaseObject* obj)
{
    //std::cout << ' ' << obj->getName() << '(' << sofa::helper::gettypename(typeid(*obj)) << ')';
    const char* color=NULL;
    if (display(obj,&color))
    {
        if (!nodeOutStack.empty()) // output node hierarchy containing this object
        {
            for (const auto& spair : nodeOutStack)
            {
                *out << spair.first;
                olinks << spair.second;
            }
            nodeOutStack.clear();
        }

        std::set<const core::objectmodel::Base*> links;
        std::string name = getName(obj);
        *out << name << " [shape=box,";
        if (color!=NULL)
            *out << "style=\"filled\",fillcolor=\"" << color << "\",";
        if (color == NULL || !strcmp(color,"#ffffff"))
            color = "#888888"; // replace white with gray for links
        *out << "label=\"";
        if (labelObjectClass)
        {
            std::string name = helper::gettypename(typeid(*obj));
            std::string::size_type pos = name.find('<');
            if (pos != std::string::npos)
                name.erase(pos);
            *out << name;
            if (labelObjectName)
                *out << "\\n";
        }
        if (labelObjectName)
        {
            if (!showNode && (core::behavior::BaseMechanicalState::DynamicCast(obj) ||
                              (core::BaseState::DynamicCast(obj) && !core::visual::VisualModel::DynamicCast(obj))))
            {
                *out << obj->getContext()->getName() << "/";
            }
            if (std::string(obj->getName(),0,7) != "default")
                *out << obj->getName();
        }
        *out << "\"];" << std::endl;
        std::string pname = getParentName(obj);
        if (!pname.empty())
        {
            olinks << pname << " -> " << name;
            if (core::BaseMapping::DynamicCast(obj) || core::topology::TopologicalMapping::DynamicCast(obj))
                //olinks << "[constraint=false,weight=10]";
                olinks << "[weight=0]";
            else
                olinks << "[weight=10]";
            olinks << ";" << std::endl;
        }
        /*
        core::behavior::BaseMechanicalState* bms = core::behavior::BaseMechanicalState::DynamicCast(obj);
        if (bms!=NULL)
        {
            core::objectmodel::BaseLink* l_topology = bms->findLink("topology");
            if (l_topology)
            {
                core::topology::BaseMeshTopology* topo = core::topology::BaseMeshTopology::DynamicCast(l_topology->getLinkedBase());
                if (topo)
                {
                    if (display(topo))
                        olinks << name << " -> " << getName(topo) << " [style=\"dashed\",constraint=false,color=\"#808080\",arrowhead=\"none\"];" << std::endl;
                }
            }
        }
        */
        core::behavior::BaseInteractionForceField* iff = core::behavior::BaseInteractionForceField::DynamicCast(obj);
        if (iff!=NULL)
        {
            core::behavior::BaseMechanicalState* model1 = iff->getMechModel1();
            core::behavior::BaseMechanicalState* model2 = iff->getMechModel2();
            //if (model1 != model2)
            {
                if (display(model1))
                {
                    links.insert(model1);
                    olinks << getName(model1) << " -> " << name << " [style=\"dashed\",penwidth=2.0,color=\"" << color << "\",arrowhead=\"none\",arrowtail=\"open\"];" << std::endl;
                }
                if (model1 != model2 && display(model2))
                {
                    links.insert(model2);
                    olinks << getName(model2) << " -> " << name << " [style=\"dashed\",penwidth=2.0,color=\"" << color << "\",arrowhead=\"none\",arrowtail=\"open\"];" << std::endl;
                }
            }
        }
        core::behavior::BaseInteractionConstraint* ic = core::behavior::BaseInteractionConstraint::DynamicCast(obj);
        if (ic!=NULL)
        {
            core::behavior::BaseMechanicalState* model1 = ic->getMechModel1();
            core::behavior::BaseMechanicalState* model2 = ic->getMechModel2();
            if (model1 != model2)
            {
                if (display(model1))
                {
                    links.insert(model1);
                    olinks << name << " -> " << getName(model1) << " [style=\"dashed\",penwidth=2.0,color=\"" << color << "\",arrowhead=\"open\"];" << std::endl;
                }
                if (display(model2))
                {
                    links.insert(model2);
                    olinks << name << " -> " << getName(model2) << " [style=\"dashed\",penwidth=2.0,color=\"" << color << "\",arrowhead=\"open\"];" << std::endl;
                }
            }
        }
        core::BaseMapping* map = core::BaseMapping::DynamicCast(obj);
        if (map!=NULL)
        {
            double width = 1.0;
            if (map->areConstraintsMapped()) width += 1.0;
            if (map->areMatricesMapped()) width += 1.0;
            if (map->areForcesMapped()) width += 2.0;
            if (map->areMassesMapped()) width += 2.0;
            sofa::helper::vector<sofa::core::BaseState*> fromModels = map->getFrom();
            sofa::helper::vector<sofa::core::BaseState*> toModels = map->getTo();
            for (unsigned int i = 0; i < fromModels.size(); ++i)
            {
                core::objectmodel::BaseObject* model1 = fromModels[i];
                if (display(model1))
                {
                    links.insert(model1);
                    olinks << getName(model1) << " -> " << name << " [style=\"dashed\",penwidth=" << width << ",color=\"" << color << "\",arrowhead=\"none\",weight=2";
                    core::BaseMapping* bmm = core::BaseMapping::DynamicCast(obj);
                    if (bmm)
                    {
                        if(bmm->isMechanical())
                            olinks << ",arrowtail=\"open\"";
                    }
                    olinks << "];" << std::endl;
                }
            }
            for (unsigned int i = 0; i < toModels.size(); ++i)
            {
                core::objectmodel::BaseObject* model2 = toModels[i];

                if (display(model2))
                {
                    links.insert(model2);
                    olinks << name << " -> " << getName(model2) << " [style=\"dashed\",penwidth=" << width << ",color=\"" << color << "\",weight=2];" << std::endl;
                }
            }
        }
        core::topology::TopologicalMapping* tmap = core::topology::TopologicalMapping::DynamicCast(obj);
        if (tmap!=NULL)
        {
            core::objectmodel::BaseObject* model1 = tmap->getFrom();
            if (display(model1))
            {
                links.insert(model1);
                olinks << getName(model1) << " -> " << name << " [style=\"dashed\",penwidth=2.0,color=\"" << color << "\",weight=2,arrowhead=\"none\"";
                if (tmap->propagateFromInputToOutputModel())
                {
                    olinks << ",arrowtail=\"open\"";
                }
                olinks << "];" << std::endl;
            }

            core::objectmodel::BaseObject* model2 = tmap->getTo();
            
            if (display(model2))
            {
                links.insert(model2);
                olinks << name << " -> " << getName(model2) << " [style=\"dashed\",penwidth=2.0,color=\"" << color << "\",weight=2];" << std::endl;
            }
        }
        if (core::DataEngine::DynamicCast(obj) || core::loader::BaseLoader::DynamicCast(obj))
        {
            std::set<core::objectmodel::Base*> inputs;
            std::set<core::objectmodel::Base*> outputs;
            const core::objectmodel::Base::VecData& datas = obj->getDataFields();
            for (core::objectmodel::Base::VecData::const_iterator it = datas.begin(); it != datas.end(); ++it)
            {
                core::objectmodel::BaseData* data = *it;
                const core::objectmodel::DDGNode::DDGLinkContainer& dinputs = data->getInputs();
                const core::objectmodel::DDGNode::DDGLinkContainer& doutputs = data->getOutputs();
                for (core::objectmodel::DDGNode::DDGLinkContainer::const_iterator it2 = dinputs.begin(); it2 != dinputs.end(); ++it2)
                {
                    core::objectmodel::BaseData* data2 = (*it2)->getData();
                    if (data2 && data2->getOwner() && data2->getOwner() != obj &&
                        !(core::DataEngine::DynamicCast(data2->getOwner()) || core::loader::BaseLoader::DynamicCast(data2->getOwner())))
                        inputs.insert(data2->getOwner());
                }
                for (core::objectmodel::DDGNode::DDGLinkContainer::const_iterator it2 = doutputs.begin(); it2 != doutputs.end(); ++it2)
                {
                    core::objectmodel::BaseData* data2 = (*it2)->getData();
                    if (data2 && data2->getOwner() && data2->getOwner() != obj)
                        outputs.insert(data2->getOwner());
                }
            }
            for(std::set<core::objectmodel::Base*>::const_iterator it = inputs.begin(); it != inputs.end(); ++it)
            {
                GNode* node1 = GNode::DynamicCast(*it);
                core::objectmodel::BaseObject* model1 = core::objectmodel::BaseObject::DynamicCast(*it);
                if ((node1 && display(node1)) || (model1 && display(model1)))
                {
                    links.insert(node1 ? (Base*)node1 : (Base*)model1);
                    olinks << (node1 ? getName(node1) : getName(model1)) << " -> " << name << " [style=\"dotted\",penwidth=2.0,color=\"" << color << "\",arrowhead=\"none\"";
                    olinks << "];" << std::endl;
                }
            }
            for(std::set<core::objectmodel::Base*>::const_iterator it = outputs.begin(); it != outputs.end(); ++it)
            {
                GNode* node2 = GNode::DynamicCast(*it);
                core::objectmodel::BaseObject* model2 = core::objectmodel::BaseObject::DynamicCast(*it);
                if ((node2 && display(node2)) || (model2 && display(model2)))
                {
                    links.insert(node2 ? (Base*)node2 : (Base*)model2);
                    olinks << name << " -> " << (node2 ? getName(node2) : getName(model2)) << " [style=\"dotted\",penwidth=4.0,color=\"" << color << "\"];" << std::endl;
                }
            }
        }
        if (showSlaves)
        {
            const sofa::core::objectmodel::BaseObject::VecSlaves& slaves = obj->getSlaves();
            for (sofa::core::objectmodel::BaseObject::VecSlaves::const_iterator it = slaves.begin(); it != slaves.end(); ++it)
            {
                this->processObject(node, it->get());
            }
        }
        links.clear();
        for (const std::string& linkName : outputLinks)
        {
            if (sofa::core::objectmodel::BaseLink* l = obj->findLink(linkName))
            {
                const unsigned int size = l->getSize();
                for (unsigned int i = 0; i < size; ++i)
                {
                    sofa::core::objectmodel::Base* b = l->getLinkedBase(i);
                    GNode* n = GNode::DynamicCast(b);
                    core::objectmodel::BaseObject* o = core::objectmodel::BaseObject::DynamicCast(b);
                    if (((n && display(n)) || (o && display(o))) && links.insert(b).second)
                    {
                        olinks << name << " -> " << (n ? getName(n) : getName(o)) << " [style=\"solid\",penwidth=2.0,color=\"" << color << "\",arrowhead=\"empty\"];" << std::endl;
                    }
                }
            }
        }
        links.clear();
        for (const std::string& linkName : inputLinks)
        {
            if (sofa::core::objectmodel::BaseLink* l = obj->findLink(linkName))
            {
                const unsigned int size = l->getSize();
                for (unsigned int i = 0; i < size; ++i)
                {
                    sofa::core::objectmodel::Base* b = l->getLinkedBase(i);
                    GNode* n = GNode::DynamicCast(b);
                    core::objectmodel::BaseObject* o = core::objectmodel::BaseObject::DynamicCast(b);
                    if (((n && display(n)) || (o && display(o))) && links.insert(b).second)
                    {
                        olinks << (n ? getName(n) : getName(o)) << " -> " << name << " [style=\"solid\",penwidth=2.0,color=\"" << color << "\",arrowhead=\"empty\"];" << std::endl;
                    }
                }
            }
        }
    }
}

simulation::Visitor::Result ExportDotVisitor::processNodeTopDown(GNode* node)
{
    if (first)
    {
        first = false;
        *out << "digraph G {" << std::endl;
        for(const auto& a : graphAttrs)
        {
            *out << "graph [" << a.first << "=\""<< a.second <<"\"];" << std::endl;
        }
        //*out << "graph [concentrate=true];" << std::endl;
        //*out << "graph [splines=curved];" << std::endl;
        for(const auto& a : nodeAttrs)
        {
            *out << "node [" << a.first << "=\""<< a.second <<"\"];" << std::endl;
        }
        for(const auto& a : edgeAttrs)
        {
            *out << "edge [" << a.first << "=\""<< a.second <<"\"];" << std::endl;
        }
    }
    if (isIncludedOrExcluded(node) == sofa::core::visual::tristate::false_value)
    {
        return RESULT_PRUNE;
    }

    std::ostringstream nodeOut, nodeOlinks;

    if (showSolverGroups && !node->solver.empty())
    {
        nodeOut << "subgraph cluster_" << getName(node) << " {\n";
        nodeOut << "  label=\"" << node->getPathName() << "\" fontsize=20 style=rounded color=grey80 bgcolor=grey95;\n";
    }

    const char* color=NULL;
    if (display(node,&color))
    {
        nodeOut << getName(node) << " [shape=hexagon,width=0.25,height=0.25,style=\"filled\"";
        if (color) nodeOut << ",fillcolor=\"" << color << "\"";
        nodeOut << ",label=\"";
        if (labelNodeClass)
        {
            std::string name = helper::gettypename(typeid(*node));
            std::string::size_type pos = name.find('<');
            if (pos != std::string::npos)
                name.erase(pos);
            nodeOut << name;
            if (labelNodeName)
                nodeOut << "\\n";
        }
        if (labelNodeName)
        {
            if (std::string(node->getName(),0,7) != "default")
                nodeOut << node->getName();
        }
        nodeOut << "\"];" << std::endl;
        if (node->parent())
        {
            nodeOlinks << getName(node->parent()) << " -> " << getName(node)<< " [minlen=2,style=\"bold\",weight=10];" << std::endl;
        }
    }
    if (!showObject) // output all nodes
    {
        *out << nodeOut.str();
        olinks << nodeOlinks.str();
    }
    else
    {
        nodeOutStack.emplace_back(nodeOut.str(),nodeOlinks.str());
    }

    for (GNode::ObjectIterator it = node->object.begin(); it != node->object.end(); ++it)
    {
        this->processObject(node, it->get());
    }

    return RESULT_CONTINUE;
}

void ExportDotVisitor::processNodeBottomUp(GNode* node)
{
    if (isIncludedOrExcluded(node) == sofa::core::visual::tristate::false_value)
    {
        return;
    }

    if (showObject && !nodeOutStack.empty())
    {
        nodeOutStack.pop_back();
    }

    if (showSolverGroups && !node->solver.empty())
    {
        *out << "}\n";
    }
}

} // namespace tree

} // namespace simulation

} // namespace sofa

