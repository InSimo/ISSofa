/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#include "DefaultPipelineMT.h"

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/core/collision/ContactManager.h>
#include <sofa/core/collision/CollisionGroupManager.h>
#ifdef ISSOFA_VERSION
#include <sofa/simulation/common/Node.h>
#else
#include <sofa/simulation/Node.h>
#endif
#include <sofa/core/ObjectFactory.h>

#include <sofa/helper/assert.h>

#define VERBOSE(a) if (bVerbose.getValue()) a; else {}

namespace sofa
{
namespace collision
{

using namespace sofa::core::collision;

SOFA_DECL_CLASS(DefaultPipelineMT)

int DefaultPipelineMTClass = sofa::core::RegisterObject("Multi Thread version of the default collision detection and modeling pipeline with added code to monitor contact information.")
.add< DefaultPipelineMT>();


namespace detail
{
template< class T >
sofa::simulation::Task::Color getColorFromPointerAdress(const T* ptr)
{
    std::uintptr_t address = reinterpret_cast<std::uintptr_t>(ptr);
    return sofa::simulation::Task::Color(float(address & 0xff) / float(255),
                                         float((address >> 8) & 0xff) / float(255),
                                         float((address >> 16) & 0xff) / float(255),
                                         1.0f);
}


ComputeResponseTask* registerTask(Contact* contact,
    VecComputeResponseTask& computeResponseTasks,
    VecComputeResponseTask::size_type taskIndex,
    sofa::simulation::Task::Status& taskStatus)
{
    SOFA_ASSERT_FAST(contact != NULL);

    ComputeResponseTask* task = NULL;
    if (taskIndex < computeResponseTasks.size()) {
        task = computeResponseTasks[taskIndex];
    }
    else
    {
        task = new ComputeResponseTask();
        computeResponseTasks.push_back(task);
    }
    ComputeResponseTaskInfo taskInfo(contact);
    task->enable(&taskStatus, taskInfo);

    return task;
}

struct ComputeResponse
{
    void operator () (const Contact::SPtr& contactPtr) const
    {
        contactPtr->computeResponse();
    }
};

inline sofa::core::objectmodel::BaseContext* selectNode(Contact* contact, sofa::core::objectmodel::BaseContext* scene)
{
    sofa::core::objectmodel::BaseContext* node = scene;
    if (!contact->getCollisionModels().first->isSimulated())
    {
        node = contact->getCollisionModels().second->getContext();
    }
    else if (!contact->getCollisionModels().second->isSimulated())
    {
        node = contact->getCollisionModels().first->getContext();
    }

    return node;
}



inline void processResponse(Contact* contact, sofa::core::objectmodel::BaseContext* node, Create)
{
    contact->createResponse(node);
}


inline void processResponse(Contact* contact, sofa::core::objectmodel::BaseContext* node, Finalize)
{
    contact->finalizeResponse(node);
}

template <typename ResponsePolicy>
struct ProcessResponse : public std::binary_function<Contact::SPtr, sofa::core::objectmodel::BaseContext*, void>
{
    void operator () (const Contact::SPtr& contactPtr, sofa::core::objectmodel::BaseContext* scene) const
    {
        Contact* contact = contactPtr.get();

        sofa::core::objectmodel::BaseContext* node = selectNode(contact, scene);
        processResponse(contact, node, ResponsePolicy());
    }
};

} // namespace detail


ComputeResponseTaskInfo::ComputeResponseTaskInfo()
    : m_contact(NULL)
{}

ComputeResponseTaskInfo::ComputeResponseTaskInfo(Contact* contact)
    : m_contact(contact)
{}

ComputeResponseTask::ComputeResponseTask()
{}

void ComputeResponseTask::enable(const sofa::simulation::Task::Status* pStatus, const ComputeResponseTaskInfo& taskInfo)
{
    sofa::simulation::Task::enable(pStatus);
    m_taskInfo = taskInfo;
    m_name = m_taskInfo.getContact() ? m_taskInfo.getContact()->getName() : std::string();
}

const char* ComputeResponseTask::getName() const
{
    return m_name.c_str();
}

bool ComputeResponseTask::run(sofa::simulation::WorkerThread* /*thread*/)
{
    m_taskInfo.getContact()->computeResponse();

    return true;
}

sofa::simulation::Task::Color ComputeResponseTask::getColor() const
{
    return detail::getColorFromPointerAdress(this);
}


CreateContactsTask::CreateContactsTask()
{
}

void CreateContactsTask::enable(const sofa::simulation::Task::Status* pStatus, const CreateContactsTaskInfo& taskInfo)
{
    sofa::simulation::Task::enable(pStatus);
    m_taskInfo = taskInfo;
}

bool CreateContactsTask::run(sofa::simulation::WorkerThread* /*thread*/)
{
    m_taskInfo.contactManager->createContacts(*m_taskInfo.detectionOutputMap);
    return true;
}

const char* CreateContactsTask::getName() const
{
    return "CreateContacts";
}

template<>
const char* ProcessReponseTask<detail::Create>::getName() const
{
    return "CreateReponse";
}

template<>
const char* ProcessReponseTask<detail::Finalize>::getName() const
{
    return "FinalizeReponse";
}

ProcessGroupManagerTask::ProcessGroupManagerTask()
{
}

void ProcessGroupManagerTask::enable(const sofa::simulation::Task::Status* pStatus, const ProcessGroupManagerTaskInfo& taskInfo)
{
    sofa::simulation::Task::enable(pStatus);
    m_taskInfo = taskInfo;
}

bool ProcessGroupManagerTask::run(sofa::simulation::WorkerThread* /*thread*/)
{
    // First we remove all contacts with non-simulated objects and directly add them
    sofa::helper::vector<Contact::SPtr> notStaticContacts;

    for (auto it = m_taskInfo.contacts->begin(); it != m_taskInfo.contacts->end(); ++it)
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

    m_taskInfo.groupManager->createGroups(m_taskInfo.sceneGraph, notStaticContacts);

    return true;
}

const char* ProcessGroupManagerTask::getName() const
{
    return "ProcessGroupManager";
}



template <typename ResponsePolicy>
ProcessReponseTask<ResponsePolicy>::ProcessReponseTask()
{
}

template <typename ResponsePolicy>
void ProcessReponseTask<ResponsePolicy>::enable(const sofa::simulation::Task::Status* pStatus, const ProcessReponseTaskInfo& taskInfo)
{
    sofa::simulation::Task::enable(pStatus);
    m_taskInfo = taskInfo;
}

template <typename ResponsePolicy>
bool ProcessReponseTask<ResponsePolicy>::run(sofa::simulation::WorkerThread* /*thread*/)
{
    std::for_each(m_taskInfo.contacts->begin(), m_taskInfo.contacts->end(),
        std::bind2nd(detail::ProcessResponse<ResponsePolicy>(), m_taskInfo.sceneGraph));

    return true;
}

template <typename ResponsePolicy>
const char* ProcessReponseTask<ResponsePolicy>::getName() const
{
    return "";
}

inline void computeResponseMT(const sofa::helper::vector<Contact::SPtr>& contacts,
    VecComputeResponseTask& computeResponseTasks,
    sofa::simulation::WorkerThread* thread)
{
    std::for_each(computeResponseTasks.begin(), computeResponseTasks.end(), std::mem_fun(&ComputeResponseTask::disable));

    computeResponseTasks.reserve(contacts.size());

    sofa::simulation::Task::Status taskStatus;
    VecComputeResponseTask::size_type taskIndex = 0;
    for (sofa::helper::vector<Contact::SPtr>::const_iterator itContact = contacts.begin(); itContact != contacts.end(); ++itContact)
    {
        ComputeResponseTask* task = detail::registerTask(itContact->get(), computeResponseTasks, taskIndex, taskStatus);
        thread->addStealableTask(task);
        ++taskIndex;
    }

    thread->workUntilDone(&taskStatus);
}

DefaultPipelineMT::DefaultPipelineMT()
    :d_useComputeResponseMT(initData(&d_useComputeResponseMT, false, "useComputeResponseMT", "If true, the response computation of each contact will be executed in parallel"))
{
}

void DefaultPipelineMT::init()
{
    Inherit1::init();
}


void DefaultPipelineMT::doCollisionResponse()
{
    sofa::simulation::WorkerThread* localThread = sofa::simulation::WorkerThread::GetCurrent();
    if (!localThread)
    {
        return;
    }
    if (contactManager == NULL)
    {
        return;
    }

    sofa::core::objectmodel::BaseContext* scene = getContext();

    VERBOSE(sout << "Create Contacts " << contactManager->getName() << sendl);

    sofa::helper::AdvancedTimer::stepBegin("CreateContacts");
    {
        CreateContactsTaskInfo createContactsTaskInfo{ &narrowPhaseDetection->getDetectionOutputs(), contactManager };
        sofa::simulation::Task::Status taskStatus;
        m_createContactsTask.disable();
        m_createContactsTask.enable(&taskStatus, createContactsTaskInfo);
        localThread->runTask(&m_createContactsTask);
    }
    sofa::helper::AdvancedTimer::stepEnd("CreateContacts");

    // finally we start the creation of collisionGroup
    const sofa::helper::vector<Contact::SPtr>& contacts = contactManager->getContacts();

    if (groupManager == NULL)
    {
        VERBOSE(sout << "Linking all contacts to Scene" << sendl);

        sofa::helper::AdvancedTimer::stepBegin("CreateResponse");
        {
            ProcessReponseTaskInfo taskInfo{ &contacts, scene };
            sofa::simulation::Task::Status taskStatus;
            m_createReponseTask.disable();
            m_createReponseTask.enable(&taskStatus, taskInfo);
            localThread->runTask(&m_createReponseTask);
        }
        sofa::helper::AdvancedTimer::stepEnd("CreateResponse");

        sofa::helper::AdvancedTimer::stepBegin("ComputeResponse");
        {
            if (d_useComputeResponseMT.getValue() == true) {
                computeResponseMT(contacts, m_computeResponseTasks, localThread);
            }
            else {
                std::for_each(contacts.begin(), contacts.end(), detail::ComputeResponse());
            }
        }
        sofa::helper::AdvancedTimer::stepEnd("ComputeResponse");


        sofa::helper::AdvancedTimer::stepBegin("FinalizeResponse");
        {
            ProcessReponseTaskInfo taskInfo{ &contacts, scene };
            sofa::simulation::Task::Status taskStatus;
            m_finalizeResponseTask.disable();
            m_finalizeResponseTask.enable(&taskStatus, taskInfo);
            localThread->runTask(&m_finalizeResponseTask);
        }
        sofa::helper::AdvancedTimer::stepEnd("FinalizeResponse");
    }
    else
    {
        VERBOSE(sout << "Create Groups " << groupManager->getName() << sendl);

        sofa::helper::AdvancedTimer::stepBegin("ProcessGroupManager");
        sofa::simulation::Task::Status taskStatus;
        ProcessGroupManagerTaskInfo taskInfo{ groupManager, scene, &contacts };
        m_processGroupManagerTask.disable();
        m_processGroupManagerTask.enable(&taskStatus, taskInfo);
        localThread->runTask(&m_processGroupManagerTask);
        sofa::helper::AdvancedTimer::stepEnd("ProcessGroupManager");
    }
}

} // namespace collision
} // namespace sofa
