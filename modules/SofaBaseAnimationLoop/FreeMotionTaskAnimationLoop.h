/*******************************************************************************
*          Private SOFA components, (c) 2018 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef SOFA_BASE_FREEMOTIONTASKANIMATIONLOOP_H
#define SOFA_BASE_FREEMOTIONTASKANIMATIONLOOP_H

#include <sofa/SofaBase.h>

#include <sofa/simulation/common/TaskSchedulerHelpers.h>
#include <sofa/simulation/common/FunctorTask.h>
//#include "FreeMotionTasks.h"
#include <sofa/simulation/common/MappingTasks.h>
#include <sofa/simulation/common/MechanicalTaskDependencyGraph.h>
#include <sofa/simulation/common/ProjectiveConstraintTasks.h>

#include <sofa/simulation/common/TaskScheduler.h>

#include <sofa/core/ConstraintParams.h>
#include <sofa/core/MechanicalParams.h>

#include <sofa/core/behavior/ConstraintSolver.h>
#include <sofa/core/collision/Pipeline.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/simulation/common/DefaultAnimationLoop.h>
#include <sofa/simulation/common/MechanicalOperations.h>

#include <sofa/helper/OptionsGroup.h>


namespace sofa
{
namespace animationloop
{

/// \brief FreeMotionTaskAnimationLoop 
/// A time stepping scheme for non smooth contact dynamics.
/// Internally uses task scheduling to dispatch computations to several threads
class SOFA_BASE_ANIMATION_LOOP_API FreeMotionTaskAnimationLoop
    : public sofa::simulation::DefaultAnimationLoop
    , public sofa::core::visual::VisualModel
{
public:
    SOFA_CLASS2(FreeMotionTaskAnimationLoop, sofa::simulation::DefaultAnimationLoop, sofa::core::visual::VisualModel);

    using TaskStatus              = sofa::simulation::TaskStatus;
    template<typename TTask>
    using TaskContainer = typename sofa::simulation::TaskTraits<TTask>::TaskContainer;

    using FunctorTask    = sofa::simulation::FunctorTask<std::function<void()> >;
    using FunctorTaskPtr = std::unique_ptr<FunctorTask>;

    using OdeSolverTask            = FunctorTask;
    using OdeSolverTaskPtr         = FunctorTaskPtr;
    using MappingPropateDxTask     = sofa::simulation::MappingPropagateVelocityTask;
    using MappingPropagateTask     = sofa::simulation::MappingPropagateTask;
    using ProjectiveConstraintTask = sofa::simulation::ProjectiveConstraintTask;
    using MechanicalTaskDependencyGraph = sofa::simulation::MechanicalTaskDependencyGraph;

    using FreeMotionPropagationTaskContainer = TaskContainer<MappingPropateDxTask>;
    using PropagationTaskContainer           = TaskContainer<MappingPropagateTask>;
    using ProjectiveConstraintTaskContainer  = TaskContainer<ProjectiveConstraintTask>;
    using OdeSolverTaskContainer             = std::vector< OdeSolverTaskPtr >;

    using LinkConstraintSolver = sofa::SingleLink<FreeMotionTaskAnimationLoop, sofa::core::behavior::ConstraintSolver,
        sofa::BaseLink::FLAG_STRONGLINK | sofa::BaseLink::FLAG_STOREPATH > ;
    using LinkCollisionPipeline = sofa::SingleLink<FreeMotionTaskAnimationLoop, sofa::core::collision::Pipeline,
        sofa::BaseLink::FLAG_STRONGLINK | sofa::BaseLink::FLAG_STOREPATH >;

    void init() override;

    void reinit() override;

    void handleEvent(sofa::core::objectmodel::Event* event) override;

    void step(const sofa::core::ExecParams* params, SReal dt) override;

    sofa::Data<bool>          d_constraintRhsVelocity;
    sofa::Data<unsigned int>  d_threadsCount;

    sofa::Data<bool>  d_showTasks;
    sofa::Data<float> d_lineSpacing;
    sofa::Data<float> d_taskTextSize;
    sofa::Data<float> d_timeScale;
    sofa::Data<float> d_timeOffset;
    sofa::Data<float> d_taskLabelMinDuration;
    sofa::Data<float> d_maxStepDurationGoal;

protected:
    FreeMotionTaskAnimationLoop();

private:
    void bwdDraw(sofa::core::visual::VisualParams* vparams) override;

    void stepInitFreeMotionStateVectors(const sofa::core::ExecParams* params);

    void stepAnimateBegin(const sofa::core::ExecParams* params, SReal dt);

    void stepBehaviorModelsUpdate(const sofa::core::ExecParams* params, SReal dt);

    void stepMechanicalBeginIntegration(const sofa::core::ExecParams* params, SReal dt);

    void stepMappingsGeometricStiffness(const sofa::core::MechanicalParams* mparams, const sofa::core::ConstraintParams* cParams, SReal dt);

    void stepOdeFreeMotion(sofa::simulation::Node* node,
        const sofa::core::MechanicalParams* mparams,
        OdeSolverTaskContainer& odeSolveTasks,
        sofa::simulation::Task::Status& taskStatus);

    void stepCollisionBegin(const sofa::core::ExecParams* params);

    void stepCollisionReset();

    void stepCollisionDetection();

    void stepPropagateFreeMotion(
        const sofa::core::MechanicalParams* mparams,
        sofa::simulation::Node* node,
        ProjectiveConstraintTaskContainer& projectXfreeTasks,
        FreeMotionPropagationTaskContainer& propagateXfreeTasks,
        MechanicalTaskDependencyGraph& taskDependencyPropagateFreeMotion);

    void stepCollisionResponse();

    void stepCollisionEnd(const sofa::core::ExecParams* params);

    void stepIntegrateEnd(const sofa::core::ExecParams* params);

    void stepConstraintSolverPrepare(const sofa::core::ConstraintParams* cparams);

    void stepConstraintSolverBuildSystem(const sofa::core::ConstraintParams* cparams);

    void stepConstraintSolverSolveSystem(const sofa::core::ConstraintParams* cparams);

    void stepConstraintSolverApplyMotionCorrection(const sofa::core::ConstraintParams* cparams, SReal dt);

    void stepMechanicalEndIntegration(const sofa::core::ExecParams* params, SReal dt);

    void stepPreTopologyChange(const sofa::core::ExecParams* params, SReal dt);

    void stepTopologyChange(const sofa::core::ExecParams* params, SReal dt);

    void stepPostTopologyChange(const sofa::core::ExecParams* params, SReal dt);

    void stepPropagateRestPosition(
        sofa::simulation::Node* node,
        const sofa::core::MechanicalParams* params,
        MechanicalTaskDependencyGraph& taskDependencyGraph,
        sofa::simulation::Task::Status& taskStatus);

    void stepPropagateIntegratedMotion(sofa::simulation::Node* node, const sofa::core::MechanicalParams* mparams,
        MechanicalTaskDependencyGraph& taskDependencyGraph, sofa::simulation::Task::Status& taskStatus);

    void stepUpdateVisual(sofa::simulation::Node* node, const sofa::core::ExecParams* params,
        MechanicalTaskDependencyGraph& taskDependencyGraph, sofa::simulation::Task::Status& taskStatus);

    void stepUpdateTime(const sofa::core::ExecParams* params, SReal dt);

    void stepAnimateEnd(const sofa::core::ExecParams* params, SReal dt);

    LinkConstraintSolver           l_constraintSolver;
    LinkCollisionPipeline          l_collisionPipeline;

    std::unique_ptr<sofa::simulation::TaskScheduler> m_taskScheduler;
    unsigned int                                     m_lastThreadsCount;
    sofa::simulation::taskscheduler::TaskDrawInfo    m_taskDrawInfo;

    using EventTask = FunctorTask;
    using EventTaskPtr = FunctorTaskPtr;

    EventTaskPtr m_animateBeginEventTask;
    EventTaskPtr m_collisionBeginEventTask;
    EventTaskPtr m_collisionEndEventTask;
    EventTaskPtr m_integrateEndEventTask;
    EventTaskPtr m_preTopologyChangeEventTask;
    EventTaskPtr m_topologyChangeEventTask;
    EventTaskPtr m_postTopologyChangeEventTask;
    EventTaskPtr m_animateEndEventTask;

    using MappingsGeometricStiffnessTask    = sofa::simulation::FunctorTaskWithArgs<sofa::core::MechanicalParams, sofa::core::ConstraintParams>;
    using MappingsGeometricStiffnessTaskPtr = std::unique_ptr< MappingsGeometricStiffnessTask >;

    FunctorTaskPtr m_initFreeMotionTask;
    FunctorTaskPtr m_behaviorUpdateTask;
    FunctorTaskPtr m_mechanicalBeginIntegrationTask;
    MappingsGeometricStiffnessTaskPtr m_mappingGeometricStiffnessTask;

    using CollisionPipelineTask = sofa::simulation::FunctorDependencyTask < std::function<void()> > ;
    using CollisionPipelineTaskPtr = std::unique_ptr< CollisionPipelineTask >;

    CollisionPipelineTaskPtr m_collisionResetTask;
    CollisionPipelineTaskPtr m_collisionDetectionTask;
    CollisionPipelineTaskPtr m_collisionResponseTask;

    using ConstraintSolverTask = sofa::simulation::FunctorDependencyTaskWithArgs< sofa::core::ConstraintParams >;
    using ConstraintSolverTaskPtr = std::unique_ptr< ConstraintSolverTask >;

    ConstraintSolverTaskPtr m_constraintSolverPrepare;
    ConstraintSolverTaskPtr m_constraintSolverBuildSystem;
    ConstraintSolverTaskPtr m_constraintSolverSolve;
    ConstraintSolverTaskPtr m_constraintSolverApplyCorrection;

    FunctorTaskPtr   m_mechanicalEndIntegrationTask;
    FunctorTaskPtr   m_updateTimeTask;

    OdeSolverTaskContainer                      m_odeSolveTasks;
    ProjectiveConstraintTaskContainer           m_projectXVfreeTasks;
    FreeMotionPropagationTaskContainer          m_propagateVXfreeTasks;

    PropagationTaskContainer   m_propagateRestXTasks;
    PropagationTaskContainer   m_propagateXVTasks;
    PropagationTaskContainer   m_propagateVisualTasks;

    MechanicalTaskDependencyGraph          m_taskDependencyPropagateFreeMotion;
    MechanicalTaskDependencyGraph          m_taskDependencyPropagateIntegratedMotion;

};

}

}

#endif //ISPHYSICSBASE_ISPHYSICSANIMATIONLOOP_H

