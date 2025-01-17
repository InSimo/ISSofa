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
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/Node.h>
#include <iostream>

namespace sofa
{

namespace simulation
{
using std::cerr;
using std::endl;
using namespace sofa::core;

Visitor::Result BaseMechanicalVisitor::processNodeTopDown(simulation::Node* node, VisitorContext* ctx)
{
    Result res = RESULT_CONTINUE;

    for (unsigned i=0; i<node->solver.size() && res!=RESULT_PRUNE; i++ )
    {
        if(testTags(node->solver[i]))
        {
            debug_write_state_before(node->solver[i]);
            ctime_t t = begin(node, node->solver[i], "fwd");
            res = this->fwdOdeSolver(ctx, node->solver[i]);
            end(node, node->solver[i], t);
            debug_write_state_after(node->solver[i]);
        }
    }


    if (res != RESULT_PRUNE)
    {

        if (node->mechanicalState != NULL)
        {
            if (node->mechanicalMapping != NULL)
            {
                //cerr<<"MechanicalVisitor::processNodeTopDown, node "<<node->getName()<<" is a mapped model"<<endl;
                if (stopAtMechanicalMapping(node, node->mechanicalMapping))
                {
                    // stop all mechanical computations
                    //std::cerr << "Pruning " << this->getClassName() << " at " << node->getPathName() << " with non-mechanical mapping" << std::endl;
                    return RESULT_PRUNE;
                }
                //else if (!node->mechanicalMapping->isMechanical()) std::cerr << "Continuing " << this->getClassName() << " at " << node->getPathName() << " with non-mechanical mapping" << std::endl;

                Result res2 = RESULT_CONTINUE;
                if(testTags(node->mechanicalMapping))
                {
                    debug_write_state_before(node->mechanicalMapping);
                    ctime_t t = begin(node, node->mechanicalMapping, "fwd");
                    res = this->fwdMechanicalMapping(ctx, node->mechanicalMapping);
                    end(node, node->mechanicalMapping , t);
                    debug_write_state_after(node->mechanicalMapping);
                }

                if(testTags(node->mechanicalState))
                {
                    debug_write_state_before(node->mechanicalState);
                    ctime_t t = begin(node, node->mechanicalState, "fwd");
                    res2 = this->fwdMappedMechanicalState(ctx, node->mechanicalState);
                    end(node, node->mechanicalState, t);
                    debug_write_state_after(node->mechanicalState);
                }

                if (res2 == RESULT_PRUNE)
                    res = res2;
            }
            else
            {
                if(testTags(node->mechanicalState))
                {
                    //cerr<<"MechanicalVisitor::processNodeTopDown, node "<<node->getName()<<" is a no-map model"<<endl;
                    debug_write_state_before(node->mechanicalState);
                    ctime_t t = begin(node, node->mechanicalState, "fwd");
                    res = this->fwdMechanicalState(ctx, node->mechanicalState);
                    end(node, node->mechanicalState, t);
                    debug_write_state_after(node->mechanicalState);
                }
            }
        }
        else if (node->mechanicalMapping != NULL) // rare case of a mechanical mapping which controls a state located elsewhere.
        {
            if (stopAtMechanicalMapping(node, node->mechanicalMapping))
            {
                // stop all mechanical computations
                //std::cerr << "Pruning " << this->getClassName() << " at " << node->getPathName() << " with non-mechanical mapping" << std::endl;
                return RESULT_PRUNE;
            }
            //else if (!node->mechanicalMapping->isMechanical()) std::cerr << "Continuing " << this->getClassName() << " at " << node->getPathName() << " with non-mechanical mapping" << std::endl;

            Result res2 = RESULT_CONTINUE;
            if(testTags(node->mechanicalMapping))
            {
                debug_write_state_before(node->mechanicalMapping);
                ctime_t t = begin(node, node->mechanicalMapping, "fwd");
                res = this->fwdMechanicalMapping(ctx, node->mechanicalMapping);
                end(node, node->mechanicalMapping , t);
                debug_write_state_after(node->mechanicalMapping);
            }

            if (res2 == RESULT_PRUNE)
                res = res2;
        }
    }

    if (res != RESULT_PRUNE)
    {
        if (node->mass != NULL)
        {
            if(testTags(node->mass))
            {
                debug_write_state_before(node->mass);
                ctime_t t = begin(node, node->mass, "fwd");
                res = this->fwdMass(ctx, node->mass);
                end(node, node->mass, t);
                debug_write_state_after(node->mass);
            }
        }
    }

    if (res != RESULT_PRUNE)
    {
        res = for_each_r(this, ctx, node->constraintSolver, &MechanicalVisitor::fwdConstraintSolver);
    }

    if (res != RESULT_PRUNE)
    {
        res = for_each_r(this, ctx, node->forceField, &MechanicalVisitor::fwdForceField);
    }

    if (res != RESULT_PRUNE)
    {
        res = for_each_r(this, ctx, node->interactionForceField, &MechanicalVisitor::fwdInteractionForceField);
    }


    if (res != RESULT_PRUNE)
    {
        res = for_each_r(this, ctx, node->projectiveConstraintSet, &MechanicalVisitor::fwdProjectiveConstraintSet);
    }

    if (res != RESULT_PRUNE)
    {
        res = for_each_r(this, ctx, node->constraintSet, &MechanicalVisitor::fwdConstraintSet);
    }


    return res;
}


void BaseMechanicalVisitor::processNodeBottomUp(simulation::Node* node, VisitorContext* ctx)
{
    for_each(this, ctx, node->projectiveConstraintSet, &MechanicalVisitor::bwdProjectiveConstraintSet);
    for_each(this, ctx, node->constraintSet, &MechanicalVisitor::bwdConstraintSet);
    for_each(this, ctx, node->constraintSolver, &MechanicalVisitor::bwdConstraintSolver);

    if (node->mechanicalState != NULL)
    {
        if (node->mechanicalMapping != NULL)
        {
            if (!stopAtMechanicalMapping(node, node->mechanicalMapping))
            {
                if(testTags(node->mechanicalState))
                {
                    ctime_t t = begin(node, node->mechanicalState, "bwd");
                    this->bwdMappedMechanicalState(ctx, node->mechanicalState);
                    end(node, node->mechanicalState, t);
                    t = begin(node, node->mechanicalMapping, "bwd");
                    this->bwdMechanicalMapping(ctx, node->mechanicalMapping);
                    end(node, node->mechanicalMapping, t);
                }
            }
        }
        else
        {
            if(testTags(node->mechanicalState))
            {
                ctime_t t = begin(node, node->mechanicalState, "bwd");
                this->bwdMechanicalState(ctx, node->mechanicalState);
                end(node, node->mechanicalState, t);
            }
        }

    }

    for (unsigned i=0; i<node->solver.size(); i++ )
    {
        if(testTags(node->solver[i]))
        {
            ctime_t t = begin(node, node->solver[i], "bwd");
            this->bwdOdeSolver(ctx, node->solver[i]);
            end(node, node->solver[i], t);
        }
    }

    if (node == root)
    {
        root = NULL;
    }
}


Visitor::Result BaseMechanicalVisitor::processNodeTopDown(simulation::Node* node)
{
    if (root == NULL)
    {
        root = node;
    }

    VisitorContext ctx;
    ctx.root = root;
    ctx.node = node;
    ctx.nodeData = rootData;
    return processNodeTopDown(node, &ctx);
}


void BaseMechanicalVisitor::processNodeBottomUp(simulation::Node* node)
{
    VisitorContext ctx;
    ctx.root = root;
    ctx.node = node;
    ctx.nodeData = rootData;
    processNodeBottomUp(node, &ctx);
}


Visitor::Result BaseMechanicalVisitor::processNodeTopDown(simulation::Node* node, LocalStorage* stack)
{
    if (root == NULL)
    {
        root = node;
    }

    VisitorContext ctx;
    ctx.root = root;
    ctx.node = node;
    ctx.nodeData = rootData;

    const bool writeData = writeNodeData();
    if (writeData)
    {
        // create temporary accumulation buffer for parallel reductions (dot products)
        if (node != root)
        {
            double* parentData = stack->empty() ? rootData : (double*)stack->top();
            ctx.nodeData = new double(0.0);
            setNodeData(node, ctx.nodeData, parentData);
            stack->push(ctx.nodeData);
        }
    }

    return processNodeTopDown(node, &ctx);
}


void BaseMechanicalVisitor::processNodeBottomUp(simulation::Node* node, LocalStorage* stack)
{
    VisitorContext ctx;
    ctx.root = root;
    ctx.node = node;
    ctx.nodeData = rootData;
    double* parentData = rootData;

    const bool writeData = writeNodeData();

    if (writeData)
    {
        // use temporary accumulation buffer for parallel reductions (dot products)
        if (node != root)
        {
            ctx.nodeData = (double*)stack->pop();
            parentData = stack->empty() ? rootData : (double*)stack->top();
        }
    }

    processNodeBottomUp(node, &ctx);

    if (writeData && parentData != ctx.nodeData)
        addNodeData(node, parentData, ctx.nodeData);
}


#ifdef SOFA_DUMP_VISITOR_INFO

void BaseMechanicalVisitor::printReadVectors(core::behavior::BaseMechanicalState* mm)
{
    if (!mm || !readVector.size() || !Visitor::printActivated || !Visitor::outputStateVector) return;

    printNode("Input");
    for (unsigned int i=0; i<readVector.size(); ++i)
    {
        sofa::core::ConstVecId id = readVector[i].getId(mm);
        if( ! id.isNull() ) printVector(mm, id );
    }
    printCloseNode("Input");
}


void BaseMechanicalVisitor::printWriteVectors(core::behavior::BaseMechanicalState* mm)
{
    if (!mm || !writeVector.size() || !Visitor::printActivated || !Visitor::outputStateVector) return;

    printNode("Output");
    for (unsigned int i=0; i<writeVector.size(); ++i)
    {
        sofa::core::VecId id = writeVector[i].getId(mm);
        if( ! id.isNull() ) printVector(mm, id );
    }
    printCloseNode("Output");
}


void BaseMechanicalVisitor::printReadVectors(simulation::Node* node, core::objectmodel::BaseObject* obj)
{
    using sofa::core::behavior::BaseInteractionForceField;
    using sofa::core::behavior::BaseInteractionProjectiveConstraintSet;
    using sofa::core::behavior::BaseInteractionConstraint;

    if (!Visitor::printActivated || !Visitor::outputStateVector) return;

    if (readVector.size())
    {
        core::behavior::BaseMechanicalState *dof1, *dof2;

        if (BaseInteractionForceField* interact = BaseInteractionForceField::DynamicCast(obj))
        {
            dof1 = interact->getMechModel1();
            dof2 = interact->getMechModel2();
        }
        else if (BaseInteractionProjectiveConstraintSet* interact = BaseInteractionProjectiveConstraintSet::DynamicCast(obj))
        {
            dof1 = interact->getMechModel1();
            dof2 = interact->getMechModel2();
        }
        else if (BaseInteractionConstraint* interact = BaseInteractionConstraint::DynamicCast(obj))
        {
            dof1 = interact->getMechModel1();
            dof2 = interact->getMechModel2();
        }/*
        else if (BaseLMConstraint* interact = BaseLMConstraint::DynamicCast(obj))
        {
                dof1 = interact->getConstrainedMechModel1();
                dof2 = interact->getConstrainedMechModel2();
}*/
        else
        {
            printReadVectors(node->mechanicalState);
            return;
        }

        TRACE_ARGUMENT arg1;
        arg1.push_back(std::make_pair("type", dof1->getClassName()));
        printNode("Components", dof1->getName(), arg1);
        printReadVectors(dof1);
        printCloseNode("Components");

        TRACE_ARGUMENT arg2;
        arg2.push_back(std::make_pair("type", dof2->getClassName()));
        printNode("Components", dof2->getName(), arg2);
        printReadVectors(dof2);
        printCloseNode("Components");
    }
}


void BaseMechanicalVisitor::printWriteVectors(simulation::Node* node, core::objectmodel::BaseObject* obj)
{
    using sofa::core::behavior::BaseInteractionForceField;
    using sofa::core::behavior::BaseInteractionProjectiveConstraintSet;
    using sofa::core::behavior::BaseInteractionConstraint;
    using sofa::core::behavior::BaseMechanicalState;

    if (!Visitor::printActivated) return;

    if (writeVector.size())
    {
        BaseMechanicalState *dof1, *dof2;

        if (BaseInteractionForceField* interact = BaseInteractionForceField::DynamicCast(obj))
        {
            dof1 = interact->getMechModel1();
            dof2 = interact->getMechModel2();
        }
        else if (BaseInteractionProjectiveConstraintSet* interact = BaseInteractionProjectiveConstraintSet::DynamicCast(obj))
        {
            dof1 = interact->getMechModel1();
            dof2 = interact->getMechModel2();
        }
        else if (BaseInteractionConstraint* interact = BaseInteractionConstraint::DynamicCast(obj))
        {
            dof1 = interact->getMechModel1();
            dof2 = interact->getMechModel2();
        }/*
        else if (BaseLMConstraint* interact = BaseLMConstraint::DynamicCast(obj))
        {
                dof1 = interact->getConstrainedMechModel1();
                dof2 = interact->getConstrainedMechModel2();
}*/
        else
        {
            BaseMechanicalState* dof = node->mechanicalState;
            if (dof == NULL)
                node->getContext()->get(dof);
            printWriteVectors(dof);
            return;
        }

        TRACE_ARGUMENT arg1;
        arg1.push_back(std::make_pair("type", dof1->getClassName()));
        printNode("Components", dof1->getName(), arg1);
        printWriteVectors(dof1);
        printCloseNode("Components");

        TRACE_ARGUMENT arg2;
        arg2.push_back(std::make_pair("type", dof2->getClassName()));
        printNode("Components", dof2->getName(), arg2);
        printWriteVectors(dof2);
        printCloseNode("Components");
    }
}


Visitor::ctime_t BaseMechanicalVisitor::begin(simulation::Node* node, core::objectmodel::BaseObject* obj, const std::string &info)
{
    ctime_t t=Visitor::begin(node, obj, info);
    printReadVectors(node, obj);
    return t;
}


void BaseMechanicalVisitor::end(simulation::Node* node, core::objectmodel::BaseObject* obj, ctime_t t0)
{
    printWriteVectors(node, obj);
    Visitor::end(node, obj, t0);
}
#endif


Visitor::Result MechanicalGetDimensionVisitor::fwdMechanicalState(VisitorContext* ctx, core::behavior::BaseMechanicalState* mm)
{
    const unsigned int n = mm->getMatrixSize();
    *ctx->nodeData += (double)n;
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalIntegrationVisitor::fwdOdeSolver(simulation::Node* node, core::behavior::OdeSolver* obj)
{
    double nextTime = node->getTime() + dt;
    MechanicalBeginIntegrationVisitor beginVisitor( this->params /* PARAMS FIRST */, dt );
    node->execute(&beginVisitor);

    sofa::core::MechanicalParams mparams(*this->params);
    mparams.setDt(dt);

#ifdef SOFA_HAVE_EIGEN2
    {
        unsigned int constraintId=0;
        core::ConstraintParams cparams;
        simulation::MechanicalAccumulateConstraint(&cparams, core::MatrixDerivId::constraintJacobian(), constraintId).execute(node);

    }
#endif
    //cerr<<"MechanicalIntegrationVisitor::fwdOdeSolver start solve obj"<<endl;
    obj->solve(params /* PARAMS FIRST */, dt);
    //cerr<<"MechanicalIntegrationVisitor::fwdOdeSolver endVisitor ok"<<endl;

    //cerr<<"MechanicalIntegrationVisitor::fwdOdeSolver end solve obj"<<endl;
    //obj->propagatePositionAndVelocity(nextTime,core::VecCoordId::position(),core::VecDerivId::velocity());

    MechanicalProjectPositionAndVelocityVisitor(&mparams /* PARAMS FIRST */, nextTime,VecCoordId::position(),VecDerivId::velocity()
            ).execute( node );

    MechanicalPropagatePositionAndVelocityVisitor(&mparams /* PARAMS FIRST */, nextTime,VecCoordId::position(),VecDerivId::velocity()
#ifdef SOFA_SUPPORT_MAPPED_MASS
            ,VecDerivId::dx()
#endif
            ).execute( node );

    MechanicalEndIntegrationVisitor endVisitor( this->params /* PARAMS FIRST */, dt );
    node->execute(&endVisitor);

    return RESULT_PRUNE;
}


Visitor::Result MechanicalIntegrationVisitor::fwdInteractionForceField(simulation::Node* /*node*/, core::behavior::BaseInteractionForceField* obj)
{
    MultiVecDerivId   ffId      = VecDerivId::externalForce();
    MechanicalParams m_mparams(*this->params);
    m_mparams.setDt(this->dt);

    obj->addForce(&m_mparams /* PARAMS FIRST */, ffId);
    return RESULT_CONTINUE;
}

template< VecType vtype>
Visitor::Result MechanicalVInitVisitor<vtype>::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState *mm)
{
    mm->vInit(this->params /* PARAMS FIRST */, vDest.getId(mm), vSrc.getId(mm));
    return RESULT_CONTINUE;
}

template< VecType vtype>
Visitor::Result MechanicalVInitVisitor<vtype>::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    if (m_propagate)
    {
        mm->vInit(this->params /* PARAMS FIRST */, vDest.getId(mm), vSrc.getId(mm));
    }

    return RESULT_CONTINUE;
}

template< VecType vtype>
std::string  MechanicalVInitVisitor<vtype>::getInfos() const
{
    std::string name = "[" + vDest.getName() + "]";
    return name;
}

template< VecType vtype>
Visitor::Result  MechanicalVAvailVisitor<vtype>::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->vAvail( this->params /* PARAMS FIRST */, v );
    this->states.insert(mm);
    return RESULT_CONTINUE;
}

template< VecType vtype>
std::string  MechanicalVAvailVisitor<vtype>::getInfos() const
{
    std::string name="[" + v.getName() + "]";
    return name;
}

template< VecType vtype>
Visitor::Result MechanicalVAllocVisitor<vtype>::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->vAlloc(this->params /* PARAMS FIRST */, v.getId(mm) );
    return RESULT_CONTINUE;
}


template< VecType vtype>
std::string  MechanicalVAllocVisitor<vtype>::getInfos() const
{
    std::string name="[" + v.getName() + "]";
    return name;
}

template< VecType vtype>
Visitor::Result MechanicalVReallocVisitor<vtype>::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState *mm)
{
    mm->vRealloc( this->params, this->getId(mm) );
    return RESULT_CONTINUE;
}

template< VecType vtype>
Visitor::Result MechanicalVReallocVisitor<vtype>::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    if (m_propagate)
    {
        mm->vRealloc(this->params /* PARAMS FIRST */, this->getId(mm) );
    }

    return RESULT_CONTINUE;
}

template< VecType vtype>
Visitor::Result MechanicalVReallocVisitor<vtype>::fwdInteractionForceField(simulation::Node* /*node*/, core::behavior::BaseInteractionForceField* /*ff*/)
{/*
    if (m_interactionForceField)
    {
        core::behavior::BaseMechanicalState* mm = ff->getMechModel1();
        mm->vRealloc( this->params, this->getId(mm) );
        mm = ff->getMechModel2();
        mm->vRealloc( this->params, this->getId(mm) );
    }
 */
    return RESULT_CONTINUE;
}

template< VecType vtype>
typename MechanicalVReallocVisitor<vtype>::MyVecId MechanicalVReallocVisitor<vtype>::getId( core::behavior::BaseMechanicalState* mm )
{
    MyVecId vid = v->getId(mm);
    if( vid.isNull() ) // not already allocated
    {
        vid = MyVecId(MyVecId::V_FIRST_DYNAMIC_INDEX);
        mm->vAvail( this->params, vid );
        v->setId( mm, vid );
    }
    return vid;
}

template< VecType vtype>
std::string  MechanicalVReallocVisitor<vtype>::getInfos() const
{
    std::string name = "[" + v->getName() + "]";
    return name;
}


template< VecType vtype>
Visitor::Result MechanicalVFreeVisitor<vtype>::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->vFree( this->params /* PARAMS FIRST */, v.getId(mm) );
    return RESULT_CONTINUE;
}

template< VecType vtype>
Visitor::Result MechanicalVFreeVisitor<vtype>::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->vFree( this->params /* PARAMS FIRST */, v.getId(mm) );
    return RESULT_CONTINUE;
}

template< VecType vtype>
Visitor::Result MechanicalVFreeVisitor<vtype>::fwdInteractionForceField(simulation::Node* /*node*/, core::behavior::BaseInteractionForceField* /*ff*/)
{
    /*
    if( interactionForceField )
    {
        core::behavior::BaseMechanicalState* mm = ff->getMechModel1();
        mm->vFree( this->params, v.getId(mm) );
        mm = ff->getMechModel2();
        mm->vFree( this->params, v.getId(mm) );
    }
*/
    return RESULT_CONTINUE;
}

template< VecType vtype>
std::string  MechanicalVFreeVisitor<vtype>::getInfos() const
{
    std::string name="[" + v.getName() + "]";
    return name;
}

Visitor::Result MechanicalVOpVisitor::fwdMechanicalState(VisitorContext* ctx, core::behavior::BaseMechanicalState* mm)
{
    //cerr<<"    MechanicalVOpVisitor::fwdMechanicalState, model "<<mm->getName()<<endl;
    if (!only_mapped)
        mm->vOp(this->params /* PARAMS FIRST */, v.getId(mm) ,a.getId(mm),b.getId(mm),((ctx->nodeData && *ctx->nodeData != 1.0) ? *ctx->nodeData * f : f) );
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalVOpVisitor::fwdMappedMechanicalState(VisitorContext* ctx, core::behavior::BaseMechanicalState* mm)
{
    //cerr<<"    MechanicalVOpVisitor::fwdMappedMechanicalState, model "<<mm->getName()<<endl;
    if (mapped || only_mapped)
    {
        mm->vOp(this->params /* PARAMS FIRST */, v.getId(mm) ,a.getId(mm),b.getId(mm),((ctx->nodeData && *ctx->nodeData != 1.0) ? *ctx->nodeData * f : f) );
    }
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalVMultiOpVisitor::fwdMechanicalState(VisitorContext* /*ctx*/, core::behavior::BaseMechanicalState* mm)
{
    //cerr<<"    MechanicalVOpVisitor::fwdMechanicalState, model "<<mm->getName()<<endl;
    mm->vMultiOp(this->params /* PARAMS FIRST */, ops );
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalVMultiOpVisitor::fwdMappedMechanicalState(VisitorContext* ctx, core::behavior::BaseMechanicalState* mm)
{
    //cerr<<"    MechanicalVOpVisitor::fwdMappedMechanicalState, model "<<mm->getName()<<endl;
    //mm->vMultiOp(ops);
    if (mapped)
    {
        if (ctx->nodeData && *ctx->nodeData != 1.0)
        {
            VMultiOp ops2 = ops;
            const double fact = *ctx->nodeData;
            for (VMultiOp::iterator it = ops2.begin(), itend = ops2.end(); it != itend; ++it)
                for (unsigned int i = 1; i < it->second.size(); ++i)
                    it->second[i].second *= fact;
            mm->vMultiOp(this->params /* PARAMS FIRST */, ops2 );
        }
        else
        {
            mm->vMultiOp(this->params /* PARAMS FIRST */, ops );
        }
    }
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalVDotVisitor::fwdMechanicalState(VisitorContext* ctx, core::behavior::BaseMechanicalState* mm)
{
    *ctx->nodeData += mm->vDot(this->params /* PARAMS FIRST */, a.getId(mm),b.getId(mm) );
    return RESULT_CONTINUE;
}

Visitor::Result MechanicalVNormVisitor::fwdMechanicalState(VisitorContext* /*ctx*/, core::behavior::BaseMechanicalState* mm)
{
    if( l>0 ) accum += mm->vSum(this->params, a.getId(mm), l );
    else {
        SReal mmax = mm->vMax(this->params, a.getId(mm) );
        if( mmax>accum ) accum=mmax;
    }
    return RESULT_CONTINUE;
}

SReal MechanicalVNormVisitor::getResult() const
{
    if( l>1 )
        return exp( log(accum) / l);
    else return accum;
}

#if 0
/// Parallel code
Visitor::Result MechanicalVDotVisitor::processNodeTopDown(simulation::Node* /*node*/, LocalStorage* stack)
{
    double* localTotal = new double(0.0);
    stack->push(localTotal);
    if (node->mechanicalState && !node->mechanicalMapping)
    {
        core::behavior::BaseMechanicalState* mm = node->mechanicalState;
        *localTotal += mm->vDot(a,b);
    }
    return RESULT_CONTINUE;
}


/// Parallel code
void MechanicalVDotVisitor::processNodeBottomUp(simulation::Node* /*node*/, LocalStorage* stack)
{
    double* localTotal = static_cast<double*>(stack->pop());
    double* parentTotal = static_cast<double*>(stack->top());
    if (!parentTotal)
        *total += *localTotal; // root
    else
        *parentTotal += *localTotal;
    delete localTotal;
}
#endif

Visitor::Result MechanicalPropagateDxVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    //<TO REMOVE>
    //mm->setDx(dx);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalPropagateDxVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    //map->propagateDx();
    //std::cout << getClassName() << getInfos() << " : " << map->getName() << "::applyJ()" << std::endl;
    map->applyJ(mparams /* PARAMS FIRST */, dx, dx);

    return RESULT_CONTINUE;
}


void MechanicalPropagateDxVisitor::bwdMechanicalState(simulation::Node* , core::behavior::BaseMechanicalState* /*mm*/)
{
}


Visitor::Result MechanicalPropagateDxAndResetForceVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    //mm->setDx(dx);
    //mm->setF(f);
    mm->resetForce(this->params /* PARAMS FIRST */, f.getId(mm));
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalPropagateDxAndResetForceVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    map->applyJ(mparams /* PARAMS FIRST */, dx, dx);

    return RESULT_CONTINUE;
}


void MechanicalPropagateDxAndResetForceVisitor::bwdMechanicalState(simulation::Node* , core::behavior::BaseMechanicalState* /*mm*/)
{
}


Visitor::Result MechanicalPropagateDxAndResetForceVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->resetForce(this->params /* PARAMS FIRST */, f.getId(mm));
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalPropagatePositionAndResetForceVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    //mm->setX(x);
    //mm->setF(f);
    mm->resetForce(this->params /* PARAMS FIRST */, f.getId(mm));
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalPropagatePositionAndResetForceVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    map->apply(mparams /* PARAMS FIRST */, x, x);

    return RESULT_CONTINUE;
}


Visitor::Result MechanicalPropagatePositionAndResetForceVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->resetForce(this->params /* PARAMS FIRST */, f.getId(mm));
    return RESULT_CONTINUE;
}



Visitor::Result MechanicalAddMDxVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    //mm->setF(res);
    //if (!dx.isNull())
    //    mm->setDx(dx);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalAddMDxVisitor::fwdMass(simulation::Node* /*node*/, core::behavior::BaseMass* mass)
{
    mass->addMDx(mparams /* PARAMS FIRST */, res, factor);
    return RESULT_PRUNE;
}


Visitor::Result MechanicalAccFromFVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    /////<TO REMOVE>
    //mm->setDx(a);
    //mm->setF(f);
    /// \todo Check presence of Mass
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalAccFromFVisitor::fwdMass(simulation::Node* /*node*/, core::behavior::BaseMass* mass)
{
    mass->accFromF(mparams /* PARAMS FIRST */, a);
    return RESULT_CONTINUE;
}



MechanicalPropagatePositionAndVelocityVisitor::MechanicalPropagatePositionAndVelocityVisitor(
    const sofa::core::MechanicalParams* mparams /* PARAMS FIRST */,
    double time, MultiVecCoordId x, MultiVecDerivId v
#ifdef SOFA_SUPPORT_MAPPED_MASS
    , MultiVecDerivId a
#endif
    )
    : MechanicalVisitor(mparams), currentTime(time), x(x), v(v)
#ifdef SOFA_SUPPORT_MAPPED_MASS
    , a(a)
#endif
{
#ifdef SOFA_DUMP_VISITOR_INFO
    setReadWriteVectors();
#endif
    //cerr<<"::MechanicalPropagatePositionAndVelocityVisitor"<<endl;
}

MechanicalPropagatePositionVisitor::MechanicalPropagatePositionVisitor(const sofa::core::MechanicalParams* mparams /* PARAMS FIRST */, double t, MultiVecCoordId x)
    : MechanicalVisitor(mparams) , t(t), x(x)
{
#ifdef SOFA_DUMP_VISITOR_INFO
    setReadWriteVectors();
#endif
    //cerr<<"::MechanicalPropagatePositionAndVelocityVisitor"<<endl;
}

#ifdef SOFA_SUPPORT_MAPPED_MASS

Visitor::Result MechanicalAddMDxVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    if (!mparams->dx().isNull())
    {
        map->applyJ(mparams /* PARAMS FIRST */, dx, dx); // TODO : check

    }
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalAddMDxVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->resetForce(mparams /* PARAMS FIRST */, res.getId(mm));
    return RESULT_CONTINUE;
}


void MechanicalAddMDxVisitor::bwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    map->applyJT(mparams /* PARAMS FIRST */, res, res);
}


void MechanicalAddMDxVisitor::bwdMechanicalState(simulation::Node* , core::behavior::BaseMechanicalState* mm)
{
}

#else

Visitor::Result MechanicalAddMDxVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* /*map*/)
{
    return RESULT_PRUNE;
}


Visitor::Result MechanicalAddMDxVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    return RESULT_PRUNE;
}

#endif // SOFA_SUPPORT_MAPPED_MASS

Visitor::Result MechanicalProjectJacobianMatrixVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* /*map*/)
{
    return RESULT_PRUNE;
}
Visitor::Result MechanicalProjectJacobianMatrixVisitor::fwdProjectiveConstraintSet(simulation::Node* /*node*/, core::behavior::BaseProjectiveConstraintSet* c)
{
    c->projectJacobianMatrix(mparams /* PARAMS FIRST */, cId);
    return RESULT_CONTINUE;
}

Visitor::Result MechanicalProjectVelocityVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* /*map*/)
{
    return RESULT_PRUNE;
}
Visitor::Result MechanicalProjectVelocityVisitor::fwdProjectiveConstraintSet(simulation::Node* /*node*/, core::behavior::BaseProjectiveConstraintSet* c)
{
    c->projectVelocity(mparams /* PARAMS FIRST */, vel);
    return RESULT_CONTINUE;
}

Visitor::Result MechanicalProjectPositionVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* /*map*/)
{
    return RESULT_PRUNE;
}
Visitor::Result MechanicalProjectPositionVisitor::fwdProjectiveConstraintSet(simulation::Node* /*node*/, core::behavior::BaseProjectiveConstraintSet* c)
{
    c->projectPosition(mparams /* PARAMS FIRST */, pos);
    return RESULT_CONTINUE;
}

Visitor::Result MechanicalProjectPositionAndVelocityVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* /*map*/)
{
    return RESULT_PRUNE;
}
Visitor::Result MechanicalProjectPositionAndVelocityVisitor::fwdProjectiveConstraintSet(simulation::Node* /*node*/, core::behavior::BaseProjectiveConstraintSet* c)
{
    c->projectPosition(mparams /* PARAMS FIRST */, pos);
    c->projectVelocity(mparams /* PARAMS FIRST */, vel);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalPropagatePositionVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    //mm->setX(x);
    return RESULT_CONTINUE;
}

Visitor::Result MechanicalPropagatePositionVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    map->apply(mparams /* PARAMS FIRST */, x, x);

    return RESULT_CONTINUE;
}


#ifdef SOFA_SUPPORT_MAPPED_MASS
Visitor::Result MechanicalPropagatePositionAndVelocityVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
#else
Visitor::Result MechanicalPropagatePositionAndVelocityVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
#endif
{
    //mm->setX(x);
    //mm->setV(v);
#ifdef SOFA_SUPPORT_MAPPED_MASS
    //    mm->setDx(a);
    mm->resetAcc(mparams /* PARAMS FIRST */, a.getId(mm));
#endif
    return RESULT_CONTINUE;
}

Visitor::Result MechanicalPropagatePositionAndVelocityVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    //map->propagateX();
    //map->propagateV();
    //#ifdef SOFA_SUPPORT_MAPPED_MASS
    //map->propagateA();
    //#endif
    map->apply(mparams /* PARAMS FIRST */, x, x);
    map->applyJ(mparams /* PARAMS FIRST */, v, v);
#ifdef SOFA_SUPPORT_MAPPED_MASS
    map->computeAccFromMapping(mparams /* PARAMS FIRST */, a, v, a);
#endif
    return RESULT_CONTINUE;
}


MechanicalPropagateVelocityVisitor::MechanicalPropagateVelocityVisitor(
    const sofa::core::MechanicalParams* mparams /* PARAMS FIRST */,
    double time, MultiVecDerivId v
#ifdef SOFA_SUPPORT_MAPPED_MASS
    ,MultiVecDerivId a
#endif
    )
    : MechanicalVisitor(mparams), currentTime(time), v(v)
#ifdef SOFA_SUPPORT_MAPPED_MASS
    , a(a)
#endif
{
#ifdef SOFA_DUMP_VISITOR_INFO
    setReadWriteVectors();
#endif
}

#ifdef SOFA_SUPPORT_MAPPED_MASS
Visitor::Result MechanicalPropagateVelocityVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
#else
Visitor::Result MechanicalPropagateVelocityVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
#endif
{
    //mm->setV(v);
#ifdef SOFA_SUPPORT_MAPPED_MASS
    //    mm->setDx(a);
    mm->resetAcc(mparams /* PARAMS FIRST */, a.getId(mm));
#endif
    return RESULT_CONTINUE;
}

Visitor::Result MechanicalPropagateVelocityVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    //map->propagateV();
    //#ifdef SOFA_SUPPORT_MAPPED_MASS
    //map->propagateA();
    //#endif
    map->applyJ(mparams /* PARAMS FIRST */, v, v);
#ifdef SOFA_SUPPORT_MAPPED_MASS
    map->computeAccFromMapping(mparams /* PARAMS FIRST */, a, v, a);
#endif
    return RESULT_CONTINUE;
}

#ifdef SOFA_SUPPORT_MAPPED_MASS

MechanicalSetPositionAndVelocityVisitor::MechanicalSetPositionAndVelocityVisitor(const sofa::core::MechanicalParams* mparams ,
        double time, MultiVecCoordId x, MultiVecDerivId v, MultiVecDerivId a)
    : MechanicalVisitor(mparams) , t(time), x(x), v(v), a(a)
{

#ifdef SOFA_DUMP_VISITOR_INFO
    setReadWriteVectors();
#endif // SOFA_DUMP_VISITOR_INFO
    //	cerr<<"::MechanicalSetPositionAndVelocityVisitor"<<endl;
}
#else
MechanicalSetPositionAndVelocityVisitor::MechanicalSetPositionAndVelocityVisitor(const sofa::core::MechanicalParams* mparams ,
        double time, MultiVecCoordId x, MultiVecDerivId v)
    : MechanicalVisitor(mparams) , t(time), x(x), v(v)
{
#ifdef SOFA_DUMP_VISITOR_INFO
    setReadWriteVectors();
#endif
    //	cerr<<"::MechanicalSetPositionAndVelocityVisitor"<<endl;
}

#endif // SOFA_SUPPORT_MAPPED_MASS

#ifdef SOFA_SUPPORT_MAPPED_MASS
Visitor::Result MechanicalSetPositionAndVelocityVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
#else
Visitor::Result MechanicalSetPositionAndVelocityVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
#endif
{
    //mm->setX(x);
    //mm->setV(v);
#ifdef SOFA_SUPPORT_MAPPED_MASS
    //mm->setDx(a);
    mm->resetAcc(mparams /* PARAMS FIRST */, a.getId(mm));
#endif
    return RESULT_CONTINUE;
}



Visitor::Result MechanicalResetForceVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    //mm->setF(res);
    if (!onlyMapped)
        mm->resetForce(this->params /* PARAMS FIRST */, res.getId(mm));
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalResetForceVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->resetForce(this->params /* PARAMS FIRST */, res.getId(mm));
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalComputeForceVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
//    cerr << "MechanicalComputeForceVisitor::fwdMechanicalState " << node->getName() << endl;
    mm->accumulateForce(this->params /* PARAMS FIRST */, res.getId(mm));
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalComputeForceVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->accumulateForce(this->params /* PARAMS FIRST */, res.getId(mm));
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalComputeForceVisitor::fwdForceField(simulation::Node* /*node*/, core::behavior::BaseForceField* ff)
{
    //cerr<<"MechanicalComputeForceVisitor::fwdForceField "<<ff->getName()<<endl;
    if( !neglectingCompliance || !ff->isCompliance.getValue() ) ff->addForce(this->mparams /* PARAMS FIRST */, res);
    return RESULT_CONTINUE;
}

void MechanicalComputeForceVisitor::bwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    //       cerr<<"MechanicalComputeForceVisitor::bwdMechanicalMapping "<<map->getName()<<endl;
    if (accumulate)
    {
        //map->accumulateForce();
        map->applyJT(mparams /* PARAMS FIRST */, res, res);
        //map->computeGeometricStiffness(mparams);
    }
}


void MechanicalComputeForceVisitor::bwdMechanicalState(simulation::Node* , core::behavior::BaseMechanicalState* /*mm*/)
{
}


Visitor::Result MechanicalComputeDfVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /* mm */)
{
    //<TO REMOVE>
    //mm->setF(res);
    //mm->accumulateDf();
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalComputeDfVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    //mm->accumulateDf();
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalComputeDfVisitor::fwdForceField(simulation::Node* /*node*/, core::behavior::BaseForceField* ff)
{
    if( !ff->isCompliance.getValue() ) ff->addDForce(this->mparams /* PARAMS FIRST */, res);
    return RESULT_CONTINUE;
}


void MechanicalComputeDfVisitor::bwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    if (accumulate)
    {
//        cerr<<"MechanicalComputeDfVisitor::bwdMechanicalMapping"<<endl;
        map->applyJT(mparams /* PARAMS FIRST */, res, res);  // apply material stiffness: variation of force below the mapping
        map->applyDJT(mparams /* PARAMS FIRST */, res, res); // apply geometric stiffness: variation due to a change of mapping, with a constant force below the mapping
    }
}


void MechanicalComputeDfVisitor::bwdMechanicalState(simulation::Node* , core::behavior::BaseMechanicalState* /*mm*/)
{
}

Visitor::Result MechanicalComputeGeometricStiffness::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    map->updateK( mparams, childForce );
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalAddMBKdxVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    //mm->setF(res);
    //<TO REMOVE>
    //mm->accumulateDf();
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalAddMBKdxVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    //mm->accumulateDf();
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalAddMBKdxVisitor::fwdForceField(simulation::Node* /*node*/, core::behavior::BaseForceField* ff)
{
    if( !ff->isCompliance.getValue() )
        ff->addMBKdx( this->mparams, res);
    else
        ff->addMBKdx( &mparamsWithoutStiffness, res);
    return RESULT_CONTINUE;
}


void MechanicalAddMBKdxVisitor::bwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    if (accumulate)
    {
        //map->accumulateDf();
        map->applyJT(mparams /* PARAMS FIRST */, res, res);
        map->applyDJT(mparams /* PARAMS FIRST */, res, res);
    }
}


void MechanicalAddMBKdxVisitor::bwdMechanicalState(simulation::Node* , core::behavior::BaseMechanicalState* /*mm*/)
{
}


Visitor::Result MechanicalResetConstraintVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    // mm->setC(res);
    mm->resetConstraint(m_cparams);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalResetConstraintVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->resetConstraint(m_cparams);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalResetConstraintVisitor::fwdConstraintSet(simulation::Node* /*node*/, core::behavior::BaseConstraintSet* c)
{
    // mm->setC(res);
    c->resetConstraint();
    return RESULT_CONTINUE;
}


#ifdef SOFA_HAVE_EIGEN2

//MechanicalExpressJacobianVisitor::MechanicalExpressJacobianVisitor(simulation::Node* /*n*/)
//{
//#ifdef SOFA_DUMP_VISITOR_INFO
//setReadWriteVectors();
//#endif
//constraintId=0;
//}


//Visitor::Result MechanicalExpressJacobianVisitor::fwdLMConstraint(simulation::Node* /*node*/, core::behavior::BaseLMConstraint* c)
//{
//	c->buildJacobian(constraintId);
//	return RESULT_CONTINUE;
//}


//void MechanicalExpressJacobianVisitor::bwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
//{
//	map->accumulateConstraint();
//}


//Visitor::Result MechanicalSolveLMConstraintVisitor::fwdConstraintSolver(simulation::Node* /*node*/, core::behavior::ConstraintSolver* s)
//{
//	typedef core::behavior::BaseMechanicalState::VecId VecId;
//	s->solveConstraint(propagateState,state);
//	return RESULT_PRUNE;
//}


Visitor::Result MechanicalWriteLMConstraint::fwdConstraintSet(simulation::Node* /*node*/, core::behavior::BaseConstraintSet* c)
{
    if (core::behavior::BaseLMConstraint* LMc=core::behavior::BaseLMConstraint::DynamicCast(c))
    {
        LMc->writeConstraintEquations(offset, id, order);
        datasC.push_back(LMc);
    }
    return RESULT_CONTINUE;
}

#endif

Visitor::Result MechanicalAccumulateConstraint::fwdConstraintSet(simulation::Node* node, core::behavior::BaseConstraintSet* c)
{
    ctime_t t0 = begin(node, c);
    c->buildConstraintMatrix(cparams, res, contactId);
    end(node, c, t0);
    return RESULT_CONTINUE;
}

void MechanicalAccumulateConstraint::bwdMechanicalMapping(simulation::Node* node, core::BaseMapping* map)
{
    ctime_t t0 = begin(node, map);
    map->applyJT(cparams, res, res);
    end(node, map, t0);
}

Visitor::Result MechanicalBuildConstraintMatrix::fwdConstraintSet(simulation::Node* node, core::behavior::BaseConstraintSet* c)
{
    const bool applyConstraint = (c->d_isConstitutiveConstraint.getValue(cparams) == 
                                  cparams->isAssemblingConstitutiveConstraints() );
    if (!applyConstraint)
    {
        return RESULT_CONTINUE;
    }

    ctime_t t0 = begin(node, c);
    c->buildConstraintMatrix(cparams, res, contactId);
    end(node, c, t0);
    return RESULT_CONTINUE;
}

void MechanicalAccumulateMatrixDeriv::bwdMechanicalMapping(simulation::Node* node, core::BaseMapping* map)
{
    ctime_t t0 = begin(node, map);
    map->applyJT(cparams, res, res);
    end(node, map, t0);
}


Visitor::Result MechanicalApplyConstraintsVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    //mm->setDx(res);
    //mm->projectResponse();
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalApplyConstraintsVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* /*mm*/)
{
    //mm->projectResponse();
    return RESULT_CONTINUE;
}


void MechanicalApplyConstraintsVisitor::bwdProjectiveConstraintSet(simulation::Node* /*node*/, core::behavior::BaseProjectiveConstraintSet* c)
{
    c->projectResponse(mparams /* PARAMS FIRST */, res);
    if (W != NULL)
    {
        c->projectResponse(mparams /* PARAMS FIRST */, W);
    }
}


Visitor::Result MechanicalBeginIntegrationVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->beginIntegration(dt);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalBeginIntegrationVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->beginIntegration(dt);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalEndIntegrationVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->endIntegration(params /* PARAMS FIRST */, dt);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalEndIntegrationVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->endIntegration(params /* PARAMS FIRST */, dt);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalComputeContactForceVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    //mm->setF(res);
    mm->accumulateForce(this->params /* PARAMS FIRST */, res.getId(mm));
    return RESULT_PRUNE;
}


Visitor::Result MechanicalComputeContactForceVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    mm->accumulateForce(this->params /* PARAMS FIRST */, res.getId(mm));
    return RESULT_CONTINUE;
}


void MechanicalComputeContactForceVisitor::bwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    //map->accumulateForce();
    map->applyJT(mparams /* PARAMS FIRST */, res, res);
}


Visitor::Result MechanicalAddSeparateGravityVisitor::fwdMass(simulation::Node* /*node*/, core::behavior::BaseMass* mass)
{
    if( mass->m_separateGravity.getValue() )
    {
        //<TO REMOVE>
        //if (! (res == VecId::velocity())) core::behavior::BaseMechanicalState::DynamicCast(node->getMechanicalState())->setV(res);
        mass->addGravityToV(this->mparams /* PARAMS FIRST */, res);
        //if (! (res == VecId::velocity())) core::behavior::BaseMechanicalState::DynamicCast(node->getMechanicalState())->setV(VecId::velocity());
    }
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalPickParticlesVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
//    std::cerr << "MechanicalPickParticlesVisitor::fwdMechanicalState, Picking particles on state " << mm->getName() << " within radius " << radius0 << " + dist * " << dRadius << std::endl;
    if (mm->hasTag(tagNoPicking)) // picking disabled for this model
    {
        return RESULT_CONTINUE;
    }

    //We deactivate the Picking with static objects (not simulated)
    core::CollisionModel *c;
    mm->getContext()->get(c, core::objectmodel::BaseContext::Local);
    if (c && !c->isSimulated()) //If it is an obstacle, we don't try to pick
    {
        return RESULT_CONTINUE;
    }
    mm->pickParticles(this->params /* PARAMS FIRST */, rayOrigin[0], rayOrigin[1], rayOrigin[2], rayDirection[0], rayDirection[1], rayDirection[2], radius0, dRadius, particles);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalPickParticlesVisitor::fwdMappedMechanicalState(simulation::Node* node, core::behavior::BaseMechanicalState* mm)
{
    if (node->mechanicalMapping  && !node->mechanicalMapping->isMechanical())
        return RESULT_PRUNE;
    mm->pickParticles(this->params /* PARAMS FIRST */, rayOrigin[0], rayOrigin[1], rayOrigin[2], rayDirection[0], rayDirection[1], rayDirection[2], radius0, dRadius, particles);
    return RESULT_CONTINUE;
}


Visitor::Result MechanicalPickParticlesVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map)
{
    if (!map->isMechanical())
        return RESULT_PRUNE;
    return RESULT_CONTINUE;
}





Visitor::Result MechanicalVSizeVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    ConstVecId id = v.getId(mm);
    if( !id.isNull() )
        *result += mm->vSize(this->params, id );
    return RESULT_CONTINUE;
}

Visitor::Result MechanicalVSizeVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    ConstVecId id = v.getId(mm);
    if( !id.isNull() )
        *result += mm->vSize(this->params, id );
    return RESULT_CONTINUE;
}


template class SOFA_SIMULATION_COMMON_API MechanicalVAvailVisitor<V_COORD>;
template class SOFA_SIMULATION_COMMON_API MechanicalVAvailVisitor<V_DERIV>;
template class SOFA_SIMULATION_COMMON_API MechanicalVAvailVisitor<V_MATDERIV>;

template class SOFA_SIMULATION_COMMON_API MechanicalVAllocVisitor<V_COORD>;
template class SOFA_SIMULATION_COMMON_API MechanicalVAllocVisitor<V_DERIV>;
template class SOFA_SIMULATION_COMMON_API MechanicalVAllocVisitor<V_MATDERIV>;

template class SOFA_SIMULATION_COMMON_API MechanicalVReallocVisitor<V_COORD>;
template class SOFA_SIMULATION_COMMON_API MechanicalVReallocVisitor<V_DERIV>;
template class SOFA_SIMULATION_COMMON_API MechanicalVReallocVisitor<V_MATDERIV>;

template class SOFA_SIMULATION_COMMON_API MechanicalVFreeVisitor<V_COORD>;
template class SOFA_SIMULATION_COMMON_API MechanicalVFreeVisitor<V_DERIV>;
template class SOFA_SIMULATION_COMMON_API MechanicalVFreeVisitor<V_MATDERIV>;

template class SOFA_SIMULATION_COMMON_API MechanicalVInitVisitor<V_COORD>;
template class SOFA_SIMULATION_COMMON_API MechanicalVInitVisitor<V_DERIV>;
template class SOFA_SIMULATION_COMMON_API MechanicalVInitVisitor<V_MATDERIV>;


} // namespace simulation

} // namespace sofa

