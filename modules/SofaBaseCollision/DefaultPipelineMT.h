/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/
#ifndef ISPHYSICS_COLLISION_DEFAULTPIPELINEMT_H
#define ISPHYSICS_COLLISION_DEFAULTPIPELINEMT_H

#include "initPlugin.h"
#include <sofa/core/collision/Contact.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <SofaBaseCollision/DefaultPipeline.h>
#include <sofa/simulation/common/Tasks.h>
#include <sofa/simulation/common/TaskScheduler.h>

ISPHYSICS_INTERNAL

namespace isphysics
{
namespace collision
{

class ComputeResponseTaskInfo
{
public:
    typedef sofa::core::collision::Contact  Contact;

    ComputeResponseTaskInfo();
    ComputeResponseTaskInfo(Contact* contact);

    const Contact* getContact() const { return m_contact; }
    Contact* getContact() { return m_contact; }

private:
    Contact* m_contact;
};

class ComputeResponseTask : public sofa::simulation::Task
{
public:
    typedef sofa::simulation::Task::Color Color;

    ComputeResponseTask();

    void enable(const sofa::simulation::Task::Status* pStatus, const ComputeResponseTaskInfo& taskInfo);

    bool run(sofa::simulation::WorkerThread* thread) override;

    const char* getName() const override;

    Color getColor() const override;

private:

    ComputeResponseTaskInfo m_taskInfo;
    std::string m_name;
};


struct CreateContactsTaskInfo
{
    typedef sofa::core::collision::NarrowPhaseDetection::DetectionOutputMap DetectionOutputMap;
    const DetectionOutputMap* detectionOutputMap;
    sofa::core::collision::ContactManager* contactManager;
};

class CreateContactsTask : public sofa::simulation::Task
{
public:
    CreateContactsTask();

    void enable(const sofa::simulation::Task::Status* pStatus, const CreateContactsTaskInfo& taskInfo);

    bool run(sofa::simulation::WorkerThread* thread) override;

    const char* getName() const override;

private:
    CreateContactsTaskInfo m_taskInfo;
};

struct ProcessReponseTaskInfo
{
    typedef sofa::core::collision::Contact Contact;
    const sofa::helper::vector<Contact::SPtr>* contacts;
    sofa::core::objectmodel::BaseContext*      sceneGraph;
};

namespace detail
{

struct Create {};

struct Finalize {};

}

template <typename ResponsePolicy>
class ProcessReponseTask : public sofa::simulation::Task
{
public:
    ProcessReponseTask();

    void enable(const sofa::simulation::Task::Status* pStatus, const ProcessReponseTaskInfo& taskInfo);

    bool run(sofa::simulation::WorkerThread* thread) override;

    const char* getName() const override;
    
private:
    ProcessReponseTaskInfo m_taskInfo;
};


struct ProcessGroupManagerTaskInfo
{
    typedef sofa::core::collision::Contact Contact;

    sofa::core::collision::CollisionGroupManager* groupManager;
    sofa::core::objectmodel::BaseContext*         sceneGraph;
    const sofa::helper::vector<Contact::SPtr>*    contacts;
};

class ProcessGroupManagerTask : public sofa::simulation::Task
{
public:
    ProcessGroupManagerTask();

    void enable(const sofa::simulation::Task::Status* pStatus, const ProcessGroupManagerTaskInfo& taskInfo);

    bool run(sofa::simulation::WorkerThread* thread) override;

    const char* getName() const override;

private:

    ProcessGroupManagerTaskInfo m_taskInfo;
};


typedef sofa::helper::vector<ComputeResponseTask* >  VecComputeResponseTask;


class SOFA_ISPHYSICS_COLLISION_API DefaultPipelineMT : public sofa::component::collision::DefaultPipeline
{
public:
    SOFA_CLASS(DefaultPipelineMT, DefaultPipeline);
    typedef Inherit1 Inherit;

    void init() override;

    sofa::Data<bool>  d_useComputeResponseMT;

protected:

    DefaultPipelineMT();

    void doCollisionResponse() override;
    
    CreateContactsTask                   m_createContactsTask;
    ProcessReponseTask<detail::Create>   m_createReponseTask;
    VecComputeResponseTask               m_computeResponseTasks;
    ProcessReponseTask<detail::Finalize> m_finalizeResponseTask;
    ProcessGroupManagerTask              m_processGroupManagerTask;


};

void computeResponseMT(const sofa::helper::vector<sofa::core::collision::Contact::SPtr>& contacts,
                       VecComputeResponseTask& computeResponseTasks,
                       sofa::simulation::WorkerThread* thread);

} // namespace collision
} // namespace isphysics

#endif
