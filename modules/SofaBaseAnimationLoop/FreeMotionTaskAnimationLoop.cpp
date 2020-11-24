/*******************************************************************************
*          Private SOFA components, (c) 2018 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include "FreeMotionTaskAnimationLoop.h"


#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/common/VectorOperations.h>
#include <sofa/simulation/common/PropagateEventVisitor.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/IntegrateBeginEvent.h>
#include <sofa/simulation/common/IntegrateEndEvent.h>
#include <sofa/simulation/common/SolveVisitor.h>
#include <sofa/simulation/common/CollisionBeginEvent.h> 
#include <sofa/simulation/common/CollisionEndEvent.h>
#include <sofa/simulation/common/UpdateContextVisitor.h>
#include <sofa/simulation/common/UpdateMappingVisitor.h>
#include <sofa/simulation/common/UpdateMappingEndEvent.h>
#include <sofa/simulation/common/UpdateBoundingBoxVisitor.h>
#include <sofa/simulation/common/MechanicalProjectPositionAndVelocityVisitorMT.h>
#include <sofa/simulation/common/MechanicalPropagatePositionAndVelocityVisitorMT.inl>
#include <sofa/simulation/common/MechanicalTaskDependencyGraph.h>
#include <sofa/simulation/common/TopologyChangeEvents.h>

#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace animationloop
{

using namespace sofa::simulation;

SOFA_DECL_CLASS(FreeMotionTaskAnimationLoop)

int FreeMotionTaskAnimationLoopClass = sofa::core::RegisterObject("A task based animation loop which uses a single linearisation of the mappings per time step.")
.add<FreeMotionTaskAnimationLoop>();

template< typename TSofaEvent >
void sendEvent(FreeMotionTaskAnimationLoop* obj, const sofa::core::ExecParams* params, SReal dt)
{
    TSofaEvent ev(dt);
    sofa::simulation::PropagateEventVisitor act(params, &ev);
    obj->getContext()->executeVisitor(&act);
}

template< typename TSofaEvent >
void sendEvent(FreeMotionTaskAnimationLoop* obj, const sofa::core::ExecParams* params)
{
    TSofaEvent ev;
    sofa::simulation::PropagateEventVisitor act(params, &ev);
    obj->getContext()->executeVisitor(&act);
}

template< typename TSofaEvent >
void sendEvent(FreeMotionTaskAnimationLoop* obj, const sofa::core::ExecParams* params, SReal dt, const char* timer)
{
    sofa::helper::ScopedAdvancedTimer t(timer);
    sendEvent<TSofaEvent>(obj, params, dt);
}

template< typename TSofaEvent >
void sendEvent(FreeMotionTaskAnimationLoop* obj, const sofa::core::ExecParams* params, const char* timer)
{
    sofa::helper::ScopedAdvancedTimer t(timer);
    sendEvent<TSofaEvent>(obj, params);
}


void sendVisitor(FreeMotionTaskAnimationLoop* obj, sofa::simulation::Visitor* v)
{
    obj->getContext()->executeVisitor(v);
}

void sendVisitor(FreeMotionTaskAnimationLoop* obj, sofa::simulation::Visitor* v, const char* timer)
{
    sofa::helper::ScopedAdvancedTimer t(timer);
    sendVisitor(obj, v);
}

void sendVisitors(FreeMotionTaskAnimationLoop* obj, std::initializer_list<sofa::simulation::Visitor*> v_list)
{
    for (auto& v : v_list)
    {
        obj->getContext()->executeVisitor(v);
    }
}

void sendVisitors(FreeMotionTaskAnimationLoop* obj, std::initializer_list<sofa::simulation::Visitor*> v_list, const char* timer)
{
    sofa::helper::ScopedAdvancedTimer t(timer);
    sendVisitors(obj, v_list);
}

class OdeSolverVisitorTask : public sofa::simulation::Visitor
{
public:

    using OdeSolverTask = FreeMotionTaskAnimationLoop::OdeSolverTask;
    using OdeSolverTaskContainer = FreeMotionTaskAnimationLoop::OdeSolverTaskContainer;

    OdeSolverVisitorTask(const sofa::core::ExecParams* params, 
        sofa::simulation::WorkerThread* thread, 
        sofa::simulation::TaskStatus* status, 
        OdeSolverTaskContainer& taskContainer)
        :sofa::simulation::Visitor(params)
        , m_thread(thread)
        , m_status(status)
        , m_taskContainer(taskContainer)
    {
    }

    Result processNodeTopDown(sofa::simulation::Node* node) override
    {
        if (!node->solver.empty())
        {
            for_each(this, node, node->solver, &OdeSolverVisitorTask::processOdeSolver);
            return RESULT_PRUNE;
        }
        return RESULT_CONTINUE;
    }

    void processOdeSolver(sofa::simulation::Node* node, core::behavior::OdeSolver* odeSolver)
    {
        SReal dt = node->getDt();
        auto func = [odeSolver, dt]() 
        {  
            odeSolver->solve(sofa::core::ExecParams::defaultInstance(), dt, 
                             sofa::core::VecCoordId::freePosition(), sofa::core::VecDerivId::freeVelocity());
        };
        m_taskContainer.push_back( std::make_unique<OdeSolverTask>(std::move(func)) );

        auto& task = m_taskContainer.back();
        task->disable();
        task->enable(m_status, "OdeSolve_" + odeSolver->getName() );
        m_thread->addStealableTask(task.get());
    }
private:
    sofa::simulation::WorkerThread* m_thread;
    sofa::simulation::TaskStatus*   m_status;
    OdeSolverTaskContainer& m_taskContainer;
};


FreeMotionTaskAnimationLoop::FreeMotionTaskAnimationLoop()
    :VisualModel()
    , DefaultAnimationLoop()
    , d_constraintRhsVelocity(initData(&d_constraintRhsVelocity, false, "constraintRhsVelocity", "Set to true to express constraint solver rhs in VEL mode"))
    , d_threadsCount(initData(&d_threadsCount, 1u, "threadsCount", "Threads count"))
    , d_showTasks(initData(&d_showTasks, false, "showTasks", "Display tasks"))
    , d_lineSpacing(initData(&d_lineSpacing, 3.0f, "lineSpacing", "The line spacing"))
    , d_taskTextSize(initData(&d_taskTextSize, 8.0f, "taskTextSize", "The text size for task labels"))
    , d_timeScale(initData(&d_timeScale, 0.04f, "timeScale", "Scale for tasks display"))
    , d_timeOffset(initData(&d_timeOffset, 0.0f, "timeOffset", "Offset for tasks display (ms)"))
    , d_taskLabelMinDuration(initData(&d_taskLabelMinDuration, 0.001f, "taskLabelMinDuration", "The minimum task duration above which the label is shown"))
    , d_maxStepDurationGoal(initData(&d_maxStepDurationGoal, 0.02f, "maxStepDurationGoal", "The maximum duration of a step that is deemed sufficient for real time"))
    , l_constraintSolver(initLink("constraintSolver", "Path to the constraint solver used by the simulation thread"))
    , l_collisionPipeline(initLink("collisionPipeline", "Path to the collision pipeline"))
{
    {
        const char* taskDataGroup = "MultiThreading";
        d_threadsCount.setGroup(taskDataGroup);
        d_showTasks.setGroup(taskDataGroup);
        d_lineSpacing.setGroup(taskDataGroup);
        d_taskTextSize.setGroup(taskDataGroup);
        d_timeScale.setGroup(taskDataGroup);
        d_timeOffset.setGroup(taskDataGroup);
        d_taskLabelMinDuration.setGroup(taskDataGroup);
        d_maxStepDurationGoal.setGroup(taskDataGroup);
    }

    this->f_listening.setValue(true);
}

void FreeMotionTaskAnimationLoop::init()
{
    DefaultAnimationLoop::init();
    VisualModel::init();

    if (!l_constraintSolver)
    {
        l_constraintSolver.set(getContext()->get<sofa::core::behavior::ConstraintSolver>());
    }
    if (!l_collisionPipeline)
    {
        l_collisionPipeline.set(getContext()->get<sofa::core::collision::Pipeline>());
    }

    if (!l_constraintSolver)
    {
        sout << "Could not find an instantce of " << l_constraintSolver.getName() << sendl;
    }

    if (!l_collisionPipeline)
    {
        sout << "Could not find an instance of " << l_collisionPipeline.getName() << sendl;
    }

    // The TaskScheduler is constructed at init() so that we are sure that the main worker thread is
    // initialized using the execution thread of the physics engine, which may differ from the thread which is 
    // used to construct the physics scene.
    if (!m_taskScheduler)
    {
        m_taskScheduler.reset(new sofa::simulation::TaskScheduler());
    }

    {
        auto func = [this]() { stepAnimateBegin(sofa::core::ExecParams::defaultInstance(), getContext()->getDt()); };
        m_animateBeginEventTask = std::make_unique<EventTask>(std::move(func));
    }
    {
        auto func = [this]() { stepCollisionBegin(sofa::core::ExecParams::defaultInstance()); };
        m_collisionBeginEventTask = std::make_unique<EventTask>(std::move(func));
    }
    {
        auto func = [this]() { stepCollisionEnd(sofa::core::ExecParams::defaultInstance()); };
        m_collisionEndEventTask = std::make_unique<EventTask>(std::move(func));
    }
    {
        auto func = [this]() { stepIntegrateEnd(sofa::core::ExecParams::defaultInstance()); };
        m_integrateEndEventTask = std::make_unique<EventTask>(std::move(func));
    }
    {
        auto func = [this]() { stepPreTopologyChange(sofa::core::ExecParams::defaultInstance(), getContext()->getDt()); };
        m_preTopologyChangeEventTask = std::make_unique<EventTask>(std::move(func));
    }
    {
        auto func = [this]() { stepTopologyChange(sofa::core::ExecParams::defaultInstance(), getContext()->getDt()); };
        m_topologyChangeEventTask = std::make_unique<EventTask>(std::move(func));
    }
    {
        auto func = [this]() { stepPostTopologyChange(sofa::core::ExecParams::defaultInstance(), getContext()->getDt()); };
        m_postTopologyChangeEventTask = std::make_unique<EventTask>(std::move(func));
    }
    {
        auto func = [this]() { stepAnimateEnd(sofa::core::ExecParams::defaultInstance(), getContext()->getDt()); };
        m_animateEndEventTask = std::make_unique<EventTask>(std::move(func));
    }
    {
        auto func = [this]() { stepInitFreeMotionStateVectors(sofa::core::ExecParams::defaultInstance()); };
        m_initFreeMotionTask = std::make_unique<FunctorTask>(std::move(func));
    }
    {
        auto func = [this]() { stepBehaviorModelsUpdate(sofa::core::ExecParams::defaultInstance(), getContext()->getDt()); };
        m_behaviorUpdateTask = std::make_unique<FunctorTask>(std::move(func));
    }
    {
        auto func = [this]() { stepMechanicalBeginIntegration(sofa::core::ExecParams::defaultInstance(), getContext()->getDt()); };
        m_mechanicalBeginIntegrationTask = std::make_unique<FunctorTask>(std::move(func));
    }
    {
        auto func = [this](sofa::core::MechanicalParams mparams, sofa::core::ConstraintParams cparams)
        {
            mparams.update();
            cparams.update();
            stepMappingsGeometricStiffness(&mparams, &cparams, this->getContext()->getDt());
        };
        m_mappingGeometricStiffnessTask = std::make_unique<MappingsGeometricStiffnessTask>(std::move(func));
    }
    {
        auto func = [this]() { stepCollisionReset();  };
        m_collisionResetTask = std::make_unique<CollisionPipelineTask>(std::move(func));
    }
    {
        auto func = [this]() { stepCollisionDetection();  };
        m_collisionDetectionTask = std::make_unique<CollisionPipelineTask>(std::move(func));
    }
    {
        auto func = [this]() { stepCollisionResponse();  };
        m_collisionResponseTask = std::make_unique<CollisionPipelineTask>(std::move(func));
    }
    {
        auto func = [this](sofa::core::ConstraintParams cparams)
        {
            cparams.update();
            stepConstraintSolverPrepare(&cparams);
        };
        m_constraintSolverPrepare = std::make_unique<ConstraintSolverTask>(std::move(func));
    }
    {
        auto func = [this](sofa::core::ConstraintParams cparams)
        {
            cparams.update();
            stepConstraintSolverBuildSystem(&cparams);
        };
        m_constraintSolverBuildSystem = std::make_unique<ConstraintSolverTask>(std::move(func));
    }
    {
        auto func = [this](sofa::core::ConstraintParams cparams)
        {
            cparams.update();
            stepConstraintSolverSolveSystem(&cparams);
        };
        m_constraintSolverSolve = std::make_unique<ConstraintSolverTask>(std::move(func));
    }
    {
        auto func = [this](sofa::core::ConstraintParams cparams)
        {
            cparams.update();
            stepConstraintSolverApplyMotionCorrection(&cparams, getContext()->getDt());
        };
        m_constraintSolverApplyCorrection = std::make_unique<ConstraintSolverTask>(std::move(func));
    }
    {
        auto func = [this]() { stepMechanicalEndIntegration(sofa::core::ExecParams::defaultInstance(), getContext()->getDt()); };
        m_mechanicalEndIntegrationTask = std::make_unique<FunctorTask>(std::move(func));
    }
    {
        auto func = [this]() { stepUpdateTime(sofa::core::ExecParams::defaultInstance(), getContext()->getDt());  };
        m_updateTimeTask = std::make_unique<FunctorTask>(std::move(func));
    }
    reinit();
}

void FreeMotionTaskAnimationLoop::reinit()
{

}

void FreeMotionTaskAnimationLoop::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::core::objectmodel::MouseEvent* mouse_event = sofa::core::objectmodel::MouseEvent::DynamicCast(event))
    {
        switch (mouse_event->getState())
        {
        case sofa::core::objectmodel::MouseEvent::Move:
            m_taskDrawInfo.mousePosition = { mouse_event->getPosX(), mouse_event->getPosY() };
            break;
        default:
            break;
        }
    }
}

void FreeMotionTaskAnimationLoop::stepInitFreeMotionStateVectors(const sofa::core::ExecParams* params)
{
    sofa::helper::ScopedAdvancedTimer t("InitFreeMotionStateVectors");
    sofa::simulation::MechanicalVInitVisitor< sofa::core::V_COORD >(params, sofa::core::VecCoordId::freePosition(), sofa::core::ConstVecCoordId::position(), true).execute(getContext());
    sofa::simulation::MechanicalVInitVisitor< sofa::core::V_DERIV >(params, sofa::core::VecDerivId::freeVelocity(), sofa::core::ConstVecDerivId::velocity(), true).execute(getContext());
}

void FreeMotionTaskAnimationLoop::stepAnimateBegin(const sofa::core::ExecParams* params, SReal dt)
{
    sendEvent<sofa::simulation::AnimateBeginEvent>(this, params, dt, "AnimateBegin");
}

void FreeMotionTaskAnimationLoop::stepBehaviorModelsUpdate(const sofa::core::ExecParams* params, SReal dt)
{
    sofa::simulation::BehaviorUpdatePositionVisitor v(params, dt);
    sendVisitor(this, &v, "BehaviorModelsUpdate");
}

void FreeMotionTaskAnimationLoop::stepMechanicalBeginIntegration(const sofa::core::ExecParams* params, SReal dt)
{
    sofa::simulation::MechanicalBeginIntegrationVisitor v(params, dt);
    sendVisitor(this, &v, "MechanicalBeginIntegration");
}

void FreeMotionTaskAnimationLoop::stepMappingsGeometricStiffness(const sofa::core::MechanicalParams* mparams, const sofa::core::ConstraintParams* cparams, SReal dt)
{
    sofa::simulation::MechanicalVOpVisitor lambdaMultInvDt(cparams, cparams->lambda(), sofa::core::ConstMultiVecId::null(), cparams->lambda(), 1.0 / dt);
    lambdaMultInvDt.setMapped(true);
    sofa::simulation::MechanicalComputeGeometricStiffness geometricStiffnessVisitor(mparams, cparams->lambda());
    sendVisitors(this, { &lambdaMultInvDt,&geometricStiffnessVisitor }, "MappingGeometricStiffness");
}

template<typename T>
struct DisableTask : public std::unary_function<typename T::value_type, void>
{
    void operator () (typename T::value_type& arg) const
    {
        typename T::mapped_type pTask = arg.second;
        pTask->disable();
    }
};

void addProjectiveConstraintTasks(MechanicalProjectPositionAndVelocityVisitorMT::MapProjectTask& projectTasks,
    MechanicalPropagatePositionAndVelocityVisitorMT::TaskDependencyGraph& taskDependencyGraph,
    sofa::simulation::Node* node,
    const sofa::core::ExecParams* params,
    const sofa::core::VecCoordId& x, const sofa::core::VecDerivId& v,
    sofa::simulation::Task::Status& taskStatus)
{
    std::for_each(projectTasks.begin(), projectTasks.end(), DisableTask<MechanicalProjectPositionAndVelocityVisitorMT::MapProjectTask>());

    const ProjectiveConstraintTaskInfo taskInfo = {
        node,
        params,
        x,
        v,
        true // project the velocity
    };

    MechanicalProjectPositionAndVelocityVisitorMT projectVisitor(taskInfo, projectTasks, taskDependencyGraph, taskStatus);
    node->executeVisitor(&projectVisitor);
}

void runTasks(MechanicalPropagatePositionAndVelocityVisitorMT::TaskDependencyGraph& taskDependencyGraph,
    sofa::simulation::Task::Status& taskStatus)
{
    taskDependencyGraph.computeGraph();
    sofa::simulation::WorkerThread* pThread = sofa::simulation::WorkerThread::GetCurrent();
    taskDependencyGraph.runTasks(*pThread, &taskStatus);
}

template< typename MapTask >
void deleteAndEraseDisabledTask(MapTask& mapTask)
{
    for (auto it = mapTask.begin(); it != mapTask.end(); )
    {
        typename MapTask::mapped_type pTask = it->second;
        if (!pTask->isEnabled())
        {
            delete pTask;
            mapTask.erase(it++);
        }
        else
        {
            ++it;
        }
    }
}

template<typename TPropagateVisitor>
void addMechanicalMappingPropagateTasks(
    typename TPropagateVisitor::MapPropagationTask& propagationTasks,
    typename TPropagateVisitor::TaskDependencyGraph& taskDependencyGraph,
    sofa::simulation::Node* node,
    const sofa::core::ExecParams* params,
    const sofa::core::VecCoordId& x, const sofa::core::VecDerivId& v, bool propagateVelocity,
    sofa::simulation::Task::Status& taskStatus, std::string tasksBaseName)
{
    std::for_each(propagationTasks.begin(), propagationTasks.end(), DisableTask<typename TPropagateVisitor::MapPropagationTask>());

    const MappingPropagateTaskInfo taskInfo = {
        node,
        params,
        x,
        v,
        propagateVelocity
    };
    TPropagateVisitor propagateVisitor(taskInfo, propagationTasks, taskDependencyGraph, taskStatus, tasksBaseName);
    node->executeVisitor(&propagateVisitor);

    // delete tasks related to the dynamically created mappings that are no longer relevant.
    deleteAndEraseDisabledTask(propagationTasks);
}

void FreeMotionTaskAnimationLoop::stepOdeFreeMotion(
    sofa::simulation::Node* node,
    const sofa::core::MechanicalParams* mparams,
    OdeSolverTaskContainer& odeSolveTaskContainer,
    sofa::simulation::Task::Status& freeMotionStatus)
{
    sofa::helper::ScopedAdvancedTimer t("FreeMotion");
    odeSolveTaskContainer.clear();
    OdeSolverVisitorTask freeMotion(mparams, sofa::simulation::WorkerThread::GetCurrent(), &freeMotionStatus, odeSolveTaskContainer );
    node->execute(&freeMotion);
}

void FreeMotionTaskAnimationLoop::stepCollisionBegin(const sofa::core::ExecParams* params)
{
    sendEvent< sofa::simulation::CollisionBeginEvent >(this, params, "CollisionBegin");
}

void FreeMotionTaskAnimationLoop::stepCollisionReset()
{
    sofa::helper::ScopedAdvancedTimer t("CollisionReset");
    l_collisionPipeline->computeCollisionReset();
}

void FreeMotionTaskAnimationLoop::stepCollisionDetection()
{
    sofa::helper::ScopedAdvancedTimer t("CollisionDetection");
    l_collisionPipeline->computeCollisionDetection();
}

void FreeMotionTaskAnimationLoop::stepPropagateFreeMotion(
    const sofa::core::MechanicalParams* mparams,
    sofa::simulation::Node* node,
    ProjectiveConstraintTaskContainer& projectXfreeTasks,
    FreeMotionPropagationTaskContainer& propagateXfreeTasks,
    MechanicalTaskDependencyGraph& taskDependencyPropagateFreeMotion)
{

    sofa::simulation::Task::Status taskStatus;

    taskDependencyPropagateFreeMotion.clear();

    // call projectPosition and projectVelocity method of every projective constraint on the freemotion state vectors
    addProjectiveConstraintTasks(projectXfreeTasks, taskDependencyPropagateFreeMotion, node, mparams,
        sofa::core::VecCoordId::freePosition(), sofa::core::VecDerivId::freeVelocity(), taskStatus);

    using MechanicalPropagateVisitor = TMechanicalPropagatePositionAndVelocityVisitorMT<MappingPropagateVelocityTask>;

    addMechanicalMappingPropagateTasks<MechanicalPropagateVisitor>(
        propagateXfreeTasks,
        taskDependencyPropagateFreeMotion,
        node, mparams,
        sofa::core::VecCoordId::freePosition(), sofa::core::VecDerivId::freeVelocity(), true,
        taskStatus, "MappingPropagateXVFreeMotion");

    if (f_printLog.getValue())
    {
        std::ostringstream oss;
        m_taskDependencyPropagateFreeMotion.dumpGraph(oss);
        sout << oss.str() << sendl;
    }

    runTasks(m_taskDependencyPropagateFreeMotion, taskStatus);

    SOFA_ASSERT_FAST_MSG(!taskStatus.IsBusy(), "Task status is busy, some tasks have not completed !");
}


void FreeMotionTaskAnimationLoop::stepCollisionResponse()
{
    sofa::helper::ScopedAdvancedTimer t("CollisionResponse");
    l_collisionPipeline->computeCollisionResponse();
    l_collisionPipeline->computeCollisionRemoveContacts();
    l_collisionPipeline->computeCollisionUpdateMappers();
}

void FreeMotionTaskAnimationLoop::stepCollisionEnd(const sofa::core::ExecParams* params)
{
    sendEvent< sofa::simulation::CollisionEndEvent >(this, params, "CollisionEnd");
}

void FreeMotionTaskAnimationLoop::stepIntegrateEnd(const sofa::core::ExecParams* params)
{
    sendEvent<sofa::simulation::IntegrateEndEvent>(this, params, "IntegrateEndEvent");
}

void FreeMotionTaskAnimationLoop::stepConstraintSolverPrepare(const sofa::core::ConstraintParams* cparams)
{
    sofa::helper::ScopedAdvancedTimer t("ConstraintSolverPrepare");
    l_constraintSolver->prepareStates(cparams, sofa::core::VecCoordId::position(), sofa::core::VecDerivId::velocity());
}

void FreeMotionTaskAnimationLoop::stepConstraintSolverBuildSystem(const sofa::core::ConstraintParams* cparams)
{
    sofa::helper::ScopedAdvancedTimer t("ConstraintSolverBuildSystem");
    l_constraintSolver->buildSystem(cparams, sofa::core::VecCoordId::position(), sofa::core::VecDerivId::velocity());
}

void FreeMotionTaskAnimationLoop::stepConstraintSolverSolveSystem(const sofa::core::ConstraintParams* cparams)
{
    sofa::helper::ScopedAdvancedTimer t("ConstraintSolverSolveSystem");
    l_constraintSolver->solveSystem(cparams, sofa::core::VecCoordId::position(), sofa::core::VecDerivId::velocity());
}

void FreeMotionTaskAnimationLoop::stepConstraintSolverApplyMotionCorrection(const sofa::core::ConstraintParams* cparams, SReal dt)
{
    sofa::helper::ScopedAdvancedTimer t("ConstraintSolverApplyMotionCorrection");
    l_constraintSolver->applyCorrection(cparams, sofa::core::VecCoordId::position(), sofa::core::VecDerivId::velocity());
    if (cparams->constOrder() == sofa::core::ConstraintParams::VEL)
    {
        sofa::simulation::common::VectorOperations vop(cparams, this->getContext());
        sofa::core::behavior::MultiVecCoord pos(&vop, sofa::core::VecCoordId::position());
        sofa::core::behavior::MultiVecDeriv vel(&vop, sofa::core::VecDerivId::velocity());
        // x_t+1 = x_t + ( vfree + dv ) * dt
        // will even go through mechanical objects under mechanical mappings unless they have mapForces set to False.
        // It is not efficient, and not the correct result for mapped mechanical objects, but this does not **really** matter 
        // since this will be overwritten during the step related to the propagation of the integrated motion. 
        pos.eq(pos, vel, dt);
    }
}

void FreeMotionTaskAnimationLoop::stepMechanicalEndIntegration(const sofa::core::ExecParams* params, SReal dt)
{
    sofa::simulation::MechanicalEndIntegrationVisitor v(params, dt);
    sendVisitor(this, &v, "MechanicalEndIntegration");
}

void FreeMotionTaskAnimationLoop::stepPreTopologyChange(const sofa::core::ExecParams* params, SReal dt)
{
    sendEvent<PreTopologyChangeEvent>(this, params, dt, "PreTopologyChange");
}

void FreeMotionTaskAnimationLoop::stepTopologyChange(const sofa::core::ExecParams* params, SReal dt)
{
    sendEvent<TopologyChangeEvent>(this, params, dt, "TopologyChange");
}

void FreeMotionTaskAnimationLoop::stepPostTopologyChange(const sofa::core::ExecParams* params, SReal dt)
{
    sendEvent<PostTopologyChangeEvent>(this, params, dt, "PostTopologyChange");
}

void FreeMotionTaskAnimationLoop::stepPropagateRestPosition(
    sofa::simulation::Node* node,
    const sofa::core::MechanicalParams* mparams,
    MechanicalTaskDependencyGraph& taskDependencyGraph,
    sofa::simulation::Task::Status& taskStatus)
{
    addMechanicalMappingPropagateTasks<MechanicalPropagatePositionAndVelocityVisitorMT>(
        m_propagateRestXTasks,
        taskDependencyGraph,
        node,
        mparams,
        sofa::core::VecCoordId::restPosition(),
        sofa::core::VecDerivId::null(), false, // do not propagate velocity
        taskStatus,
        "MappingPropagateRestX");
}

void FreeMotionTaskAnimationLoop::stepPropagateIntegratedMotion(sofa::simulation::Node* node,
    const sofa::core::MechanicalParams* mparams,
    MechanicalTaskDependencyGraph& taskDependencyGraph,
    sofa::simulation::Task::Status& taskStatus)
{
    addMechanicalMappingPropagateTasks<MechanicalPropagatePositionAndVelocityVisitorMT>(m_propagateXVTasks, taskDependencyGraph,
        node, mparams, sofa::core::VecCoordId::position(), sofa::core::VecDerivId::velocity(), true, taskStatus, "MappingPropagateXVCorrected");
}

void FreeMotionTaskAnimationLoop::stepUpdateVisual(sofa::simulation::Node* node,
    const sofa::core::ExecParams* params,
    MechanicalTaskDependencyGraph& taskDependencyGraph,
    sofa::simulation::Task::Status& taskStatus)
{

    std::for_each(m_propagateVisualTasks.begin(), m_propagateVisualTasks.end(),
        DisableTask<MechanicalPropagatePositionAndVelocityVisitorMT::MapPropagationTask>());

    const MappingPropagateTaskInfo taskInfo = {
        node,
        params,
        sofa::core::VecCoordId::position(),
        sofa::core::VecDerivId::velocity(),
        true
    };

    UpdateMappingVisitorMT updateMappingVisitor(taskInfo, m_propagateVisualTasks, taskDependencyGraph, taskStatus);
    node->executeVisitor(&updateMappingVisitor);
}

void FreeMotionTaskAnimationLoop::stepUpdateTime(const sofa::core::ExecParams* params, SReal dt)
{
    sofa::helper::ScopedAdvancedTimer t("UpdateTime");
    sofa::simulation::Node::DynamicCast(this->getContext())->setTime(this->getContext()->getTime() + dt);
    sofa::simulation::UpdateSimulationContextVisitor v(params);
    sendVisitor(this, &v);
}

void FreeMotionTaskAnimationLoop::stepAnimateEnd(const sofa::core::ExecParams* params, SReal dt)
{
    sendEvent<sofa::simulation::AnimateEndEvent>(this, params, dt, "AnimateEnd");
}

void FreeMotionTaskAnimationLoop::step(const sofa::core::ExecParams* params, SReal /*dt*/)
{
    sofa::simulation::taskscheduler::setup(*m_taskScheduler.get(), m_lastThreadsCount, d_threadsCount.getValue(), d_showTasks.getValue());

    sofa::simulation::Task::Status stepTaskStatus;
    m_taskScheduler.get()->notifyWorkersForWork(&stepTaskStatus);

    start(m_taskDrawInfo.timer);

    sofa::simulation::Node* node = sofa::simulation::Node::DynamicCast(getContext());

    sofa::helper::AdvancedTimer::begin("Animate");

    sofa::simulation::common::VectorOperations vop(params, this->getContext());
    sofa::core::behavior::MultiVecCoord freePos(&vop, sofa::core::VecCoordId::freePosition());
    sofa::core::behavior::MultiVecDeriv freeVel(&vop, sofa::core::VecDerivId::freeVelocity());

    sofa::core::ConstraintParams cparams(*params);
    cparams.setX(freePos);
    cparams.setV(freeVel);
    if (l_constraintSolver)
    {
        cparams.setDx(l_constraintSolver->getDx());
        cparams.setLambda(l_constraintSolver->getLambda());
    }

    if (d_constraintRhsVelocity.getValue(params))
    {
        cparams.setOrder(sofa::core::ConstraintParams::ConstOrder::VEL);

    }
    else
    {
        cparams.setOrder(sofa::core::ConstraintParams::ConstOrder::POS_AND_VEL);
    }

    sofa::core::MechanicalParams mparams(*params);

    sofa::simulation::WorkerThread* thread = sofa::simulation::WorkerThread::GetCurrent();

    sofa::simulation::TaskStatus taskStatus;

    {
        m_initFreeMotionTask->disable();
        m_initFreeMotionTask->enable(&taskStatus, "InitFreeMotion");
        thread->runTask(m_initFreeMotionTask.get());
    }
    {
        m_animateBeginEventTask->disable();
        m_animateBeginEventTask->enable(&taskStatus, "AnimateBeginEvent");
        thread->runTask(m_animateBeginEventTask.get());
    }
    {
        m_behaviorUpdateTask->disable();
        m_behaviorUpdateTask->enable(&taskStatus, "BehaviorModelUpdate");
        thread->runTask(m_behaviorUpdateTask.get());
    }
    {
        m_mechanicalBeginIntegrationTask->disable();
        m_mechanicalBeginIntegrationTask->enable(&taskStatus, "MechanicalBeginIntegration");
        thread->runTask(m_mechanicalBeginIntegrationTask.get());
    }
    {
        m_mechanicalBeginIntegrationTask->disable();
        m_mechanicalBeginIntegrationTask->enable(&taskStatus, "MechanicalBeginIntegration");
        thread->runTask(m_mechanicalBeginIntegrationTask.get());
    }
    {
        m_mappingGeometricStiffnessTask->disable();
        m_mappingGeometricStiffnessTask->enable(&taskStatus, "MappingGeometricStiffness");
        m_mappingGeometricStiffnessTask->setArguments(mparams, cparams);
        thread->runTask(m_mappingGeometricStiffnessTask.get());
    }
    {
        SOFA_ASSERT_FAST_MSG(!taskStatus.IsBusy(), "Task status is busy, some tasks have not completed !");

        //Same task status used for both freeMotion and CollisionDetection tasks

        if (l_collisionPipeline)
        {
            // Fire event synchronously
            m_collisionBeginEventTask->disable();
            m_collisionBeginEventTask->enable(&taskStatus, "CollisionBeginEvent");
            thread->runTask(m_collisionBeginEventTask.get());
        }
        stepOdeFreeMotion(node, &mparams, m_odeSolveTasks, taskStatus);

        if (l_collisionPipeline)
        {
            m_collisionResetTask->disable();
            m_collisionResetTask->enable(&taskStatus, "CollisionReset");

            m_collisionDetectionTask->disable();
            m_collisionDetectionTask->enable(&taskStatus, "CollisionDetection");

            m_collisionResetTask->addSuccessor(m_collisionDetectionTask.get());

            thread->addStealableTask(m_collisionResetTask.get());
        }

        thread->workUntilDone(&taskStatus);
    }

    SOFA_ASSERT_FAST_MSG(!taskStatus.IsBusy(), "Task status is busy, some tasks have not completed !"); 

    stepPropagateFreeMotion(&mparams, node, m_projectXVfreeTasks, m_propagateVXfreeTasks, m_taskDependencyPropagateFreeMotion);

    if (l_collisionPipeline)
    {
        {
            m_collisionResponseTask->disable();
            m_collisionResponseTask->enable(&taskStatus, "CollisionResponse");
            thread->runTask(m_collisionResponseTask.get());
        }
        {
            m_collisionEndEventTask->disable();
            m_collisionEndEventTask->enable(&taskStatus, "CollisionEndEvent");
            thread->runTask(m_collisionEndEventTask.get());
        }
    }
    {
        m_integrateEndEventTask->disable();
        m_integrateEndEventTask->enable(&taskStatus, "IntegrateEndEvent");
        thread->runTask(m_integrateEndEventTask.get());
    }
    if (l_constraintSolver)
    {
        m_constraintSolverPrepare->disable();
        m_constraintSolverPrepare->enable(&taskStatus, "ConstraintSolverPrepare");
        m_constraintSolverPrepare->setArguments(cparams);
        thread->runTask(m_constraintSolverPrepare.get());

        m_constraintSolverBuildSystem->disable();
        m_constraintSolverBuildSystem->enable(&taskStatus, "ConstraintSolverBuildSystem");
        m_constraintSolverBuildSystem->setArguments(cparams);
        thread->runTask(m_constraintSolverBuildSystem.get());

        m_constraintSolverSolve->disable();
        m_constraintSolverSolve->enable(&taskStatus, "ConstraintSolverSolve");
        m_constraintSolverSolve->setArguments(cparams);
        thread->runTask(m_constraintSolverSolve.get());

        m_constraintSolverApplyCorrection->disable();
        m_constraintSolverApplyCorrection->enable(&taskStatus, "ConstraintSolverApplyCorrection");
        m_constraintSolverApplyCorrection->setArguments(cparams);
        thread->runTask(m_constraintSolverApplyCorrection.get());
    }
    {
        m_mechanicalEndIntegrationTask->disable();
        m_mechanicalEndIntegrationTask->enable(&taskStatus, "MechanicalEndIntegration");
        thread->runTask(m_mechanicalEndIntegrationTask.get());
    }
    {
        m_preTopologyChangeEventTask->disable();
        m_preTopologyChangeEventTask->enable(&taskStatus,"PreTopologyChangeEvent");
        thread->runTask(m_preTopologyChangeEventTask.get());

        m_topologyChangeEventTask->disable();
        m_topologyChangeEventTask->enable(&taskStatus,"TopologyChangeEvent");
        thread->runTask(m_topologyChangeEventTask.get());

        m_postTopologyChangeEventTask->disable();
        m_postTopologyChangeEventTask->enable(&taskStatus,"PostTopologyChangeEvent");
        thread->runTask(m_postTopologyChangeEventTask.get());
    }
    {
        SOFA_ASSERT_FAST_MSG(!taskStatus.IsBusy(), "Task status is busy, some tasks have not completed !");

        m_taskDependencyPropagateIntegratedMotion.clear();

        stepPropagateRestPosition(node, &mparams, m_taskDependencyPropagateIntegratedMotion, taskStatus);
        stepPropagateIntegratedMotion(node, &mparams, m_taskDependencyPropagateIntegratedMotion, taskStatus);
        stepUpdateVisual(node, params, m_taskDependencyPropagateIntegratedMotion, taskStatus);

        if (f_printLog.getValue())
        {
            std::ostringstream oss;
            m_taskDependencyPropagateIntegratedMotion.dumpGraph(oss);
            sout << oss.str() << sendl;
        }

        runTasks(m_taskDependencyPropagateIntegratedMotion, taskStatus);

        SOFA_ASSERT_FAST_MSG(!taskStatus.IsBusy(), "Task status is busy, some tasks have not completed !");

    }
    {
        m_updateTimeTask->disable();
        m_updateTimeTask->enable(&taskStatus, "UpdateTime");
        thread->runTask(m_updateTimeTask.get());
    }
    {
        m_animateEndEventTask->disable();
        m_animateEndEventTask->enable(&taskStatus, "AnimateEndEvent");
        thread->runTask(m_animateEndEventTask.get());
    }

    sofa::helper::AdvancedTimer::end("Animate");
    
    SOFA_ASSERT_FAST_MSG(!taskStatus.IsBusy(), "Task status is busy, some tasks have not completed !");

    thread->workUntilDone(&stepTaskStatus);

    SOFA_ASSERT_FAST_MSG(!stepTaskStatus.IsBusy(), "Task status is busy, some tasks have not completed !");

    sofa::simulation::taskscheduler::stop(m_taskDrawInfo.timer);
}


void FreeMotionTaskAnimationLoop::bwdDraw(sofa::core::visual::VisualParams* vparams)
{
    sofa::core::visual::VisualModel::bwdDraw(vparams);

    if (d_showTasks.getValue())
    {
        sofa::simulation::taskscheduler::draw(
            *m_taskScheduler.get(),
            vparams,
            m_taskDrawInfo,
            d_lineSpacing.getValue(),
            d_taskTextSize.getValue(),
            d_timeScale.getValue(),
            d_timeOffset.getValue(),
            d_taskLabelMinDuration.getValue(),
            d_maxStepDurationGoal.getValue());
    }
}

}

}
