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
#ifndef SOFA_COLLISION_DEFAULTPIPELINEMT_H
#define SOFA_COLLISION_DEFAULTPIPELINEMT_H

#include <sofa/core/collision/Contact.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <SofaBaseCollision/DefaultPipeline.h>
#include <sofa/simulation/common/Tasks.h>
#include <sofa/simulation/common/TaskScheduler.h>
#include <sofa/SofaBase.h>

namespace sofa
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


class SOFA_BASE_COLLISION_API DefaultPipelineMT : public sofa::component::collision::DefaultPipeline
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
} // namespace sofa

#endif
