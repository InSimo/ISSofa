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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/


#include "OmniDriver.h"
#include <sofa/core/objectmodel/HapticDeviceEvent.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/UpdateMappingVisitor.h>
#include <sofa/core/ObjectFactory.h>

using namespace sofa::defaulttype;

namespace sofa
{

    namespace component
    {

        namespace controller
        {

            // Haptics
            HHD hHD = HD_INVALID_HANDLE;
            vector< HHD > hHDVector;
            HDSchedulerHandle hStateHandle = HD_INVALID_HANDLE;

            void OmniDriver::printError(const HDErrorInfo *error, const char *message)
            {
                std::cout << "[OmniDriver] Device error :" << std::endl;
                std::cout << hdGetErrorString(error->errorCode) << std::endl;
                std::cout << "HHD: " << error->hHD << std::endl;
                std::cout << "Error Code: " << error->hHD << std::endl;
                std::cout << "Internal Error Code: " << error->internalErrorCode << std::endl;
                std::cout << "Message: " << message << std::endl;
            }


            HDCallbackCode HDCALLBACK copyDeviceDataCallback(void * userData);

            static sofa::helper::system::atomic<int> doUpdate;
            vector<OmniDriver*> otherOmniDriver;

            //boucle qui recupere les info sur l'interface et les copie sur data->servoDeviceData
            HDCallbackCode HDCALLBACK stateCallback(void * userData)
            {
                if (doUpdate)
                {
                    copyDeviceDataCallback(userData);
                    doUpdate.dec(); // set to 0
                }

                //vector<OmniDriver*> otherOmniDriver = static_cast<vector<OmniDriver*>>(userData);
                //NewOmniData* data = static_cast<NewOmniData*>(userData);
                //FIXME : Apparenlty, this callback is run before the mechanical state initialisation. I've found no way to know whether the mechcanical state is initialized or not, so i wait ...

                RigidTypes::VecCoord positionDevs;
                RigidTypes::VecDeriv forceDevs;
                forceDevs.clear();
                positionDevs.resize(otherOmniDriver.size());
                forceDevs.resize(otherOmniDriver.size());

                for (unsigned int i = 0; i<otherOmniDriver.size(); i++)
                {
                    if (otherOmniDriver[i]->data.m_servoDeviceData.m_stop)
                    {
                        return HD_CALLBACK_DONE;
                    }
                    if (!otherOmniDriver[i]->data.m_servoDeviceData.m_ready)
                    {
                        return HD_CALLBACK_CONTINUE;
                    }
                    HHD hapticHD = hHDVector[i];
                    hdMakeCurrentDevice(hapticHD);
                    hdBeginFrame(hapticHD);

                    if ((otherOmniDriver[i]->data.m_servoDeviceData.m_buttonState & HD_DEVICE_BUTTON_1) || otherOmniDriver[i]->data.m_permanent_feedback)
                        hdSetDoublev(HD_CURRENT_FORCE, otherOmniDriver[i]->data.m_currentForce);

                    otherOmniDriver[i]->data.m_servoDeviceData.m_id = hapticHD;

                    // Retrieve the current button(s).
                    hdGetIntegerv(HD_CURRENT_BUTTONS, &otherOmniDriver[i]->data.m_servoDeviceData.m_buttonState);

                    //get the position
                    hdGetDoublev(HD_CURRENT_POSITION, otherOmniDriver[i]->data.m_servoDeviceData.m_devicePosition);

                    // Get the column major transform
                    HDdouble transform[16];
                    hdGetDoublev(HD_CURRENT_TRANSFORM, transform);

                    // get Position and Rotation from transform => put in servoDeviceData
                    Mat3x3d mrot;
                    Quat rot;
                    for (int u = 0; u<3; u++)
                        for (int j = 0; j<3; j++)
                            mrot[u][j] = transform[j * 4 + u];

                    rot.fromMatrix(mrot);
                    rot.normalize();

                    double factor = 0.001;
                    Vec3d pos(transform[12 + 0] * factor, transform[12 + 1] * factor, transform[12 + 2] * factor); // omni pos is in mm => sofa simulation are in meters by default
                    otherOmniDriver[i]->data.m_servoDeviceData.m_pos = pos;


                    // verify that the quaternion does not flip:
                    if ((rot[0] * otherOmniDriver[i]->data.m_servoDeviceData.m_quat[0]
                        + rot[1] * otherOmniDriver[i]->data.m_servoDeviceData.m_quat[1]
                        + rot[2] * otherOmniDriver[i]->data.m_servoDeviceData.m_quat[2]
                        + rot[3] * otherOmniDriver[i]->data.m_servoDeviceData.m_quat[3]) < 0)
                        for (int u = 0; u<4; u++)
                            rot[u] *= -1;

                    for (int u = 0; u<4; u++)
                        otherOmniDriver[i]->data.m_servoDeviceData.m_quat[u] = rot[u];

                    SolidTypes<double>::Transform baseOmni_H_endOmni(pos* otherOmniDriver[i]->data.m_scale, rot);
                    SolidTypes<double>::Transform world_H_virtualTool = otherOmniDriver[i]->data.m_world_H_baseOmni * baseOmni_H_endOmni * otherOmniDriver[i]->data.m_endOmni_H_virtualTool;

                    positionDevs[i].getCenter() = world_H_virtualTool.getOrigin();
                    positionDevs[i].getOrientation() = world_H_virtualTool.getOrientation();

                    //angles
                    hdGetFloatv(HD_CURRENT_JOINT_ANGLES, otherOmniDriver[i]->angle1);
                    hdGetFloatv(HD_CURRENT_GIMBAL_ANGLES, otherOmniDriver[i]->angle2);

                    hdEndFrame(hapticHD);

                }

                for (unsigned int i = 0; i<otherOmniDriver.size(); i++)
                {

                    for (unsigned int j = 0; j< positionDevs.size(); j++)
                    {
                        SReal fx, fy, fz;
                        (otherOmniDriver[i]->data.m_forceFeedback)->computeForce(positionDevs[j].getCenter().x(), positionDevs[j].getCenter().y(), positionDevs[j].getCenter().z(), 0, 0, 0, 0, fx, fy, fz);
                        forceDevs[j] = RigidTypes::Deriv(Vec3d(fx, fy, fz), Vec3d());
                    }

                    /// COMPUTATION OF THE virtualTool 6D POSITION IN THE World COORDINATES
                    SolidTypes<double>::Transform baseOmni_H_endOmni((otherOmniDriver[i]->data.m_servoDeviceData.m_pos)* otherOmniDriver[i]->data.m_scale, otherOmniDriver[i]->data.m_servoDeviceData.m_quat);

                    Vec3d world_pos_tool = positionDevs[i].getCenter();
                    Quat world_quat_tool = positionDevs[i].getOrientation();

                    // we compute its value in the current Tool frame:
                    SolidTypes<double>::SpatialVector Wrench_tool_inTool(world_quat_tool.inverseRotate(forceDevs[i].getVCenter()), world_quat_tool.inverseRotate(forceDevs[i].getVOrientation()));
                    // we transport (change of application point) its value to the endOmni frame
                    SolidTypes<double>::SpatialVector Wrench_endOmni_inEndOmni = otherOmniDriver[i]->data.m_endOmni_H_virtualTool * Wrench_tool_inTool;
                    // we compute its value in the baseOmni frame
                    SolidTypes<double>::SpatialVector Wrench_endOmni_inBaseOmni(baseOmni_H_endOmni.projectVector(Wrench_endOmni_inEndOmni.getForce()), baseOmni_H_endOmni.projectVector(Wrench_endOmni_inEndOmni.getTorque()));

                    otherOmniDriver[i]->data.m_currentForce[0] = Wrench_endOmni_inBaseOmni.getForce()[0] * otherOmniDriver[i]->data.m_forceScale;
                    otherOmniDriver[i]->data.m_currentForce[1] = Wrench_endOmni_inBaseOmni.getForce()[1] * otherOmniDriver[i]->data.m_forceScale;
                    otherOmniDriver[i]->data.m_currentForce[2] = Wrench_endOmni_inBaseOmni.getForce()[2] * otherOmniDriver[i]->data.m_forceScale;

                    otherOmniDriver[i]->data.m_servoDeviceData.m_nupdates++;
                }
                return HD_CALLBACK_CONTINUE;
            }

            // copy info on the device from data->servoDeviceData to data->deviceData
            //TODO: fill PosD  here and remove data->deviceData becoming useless
            HDCallbackCode HDCALLBACK copyDeviceDataCallback(void * /*pUserData*/)
            {
                for (unsigned int i = 0; i< otherOmniDriver.size(); i++)
                {
                    memcpy(&otherOmniDriver[i]->data.m_deviceData, &otherOmniDriver[i]->data.m_servoDeviceData, sizeof(NewDeviceData));
                    otherOmniDriver[i]->data.m_servoDeviceData.m_nupdates = 0;
                    otherOmniDriver[i]->data.m_servoDeviceData.m_ready = true;
                }
                return HD_CALLBACK_DONE;
            }

            //stop callback > what's the difference with exithandler??
            HDCallbackCode HDCALLBACK stopCallback(void * /*pUserData*/)
            {
                for (unsigned int i = 0; i<otherOmniDriver.size(); i++)
                    otherOmniDriver[i]->data.m_servoDeviceData.m_stop = true;
                return HD_CALLBACK_DONE;
            }

            int OmniDriver::initDevice()
            {
                sout << "[OmniDriver] InitDevice called" << sendl;
                HDErrorInfo error;
                for (unsigned int i = 0; i<otherOmniDriver.size(); i++)
                {
                    while (otherOmniDriver[i]->m_initialized && i<otherOmniDriver.size())
                    {
                        i++;
                        if (i == otherOmniDriver.size())
                            return 0;
                    }

                    otherOmniDriver[i]->m_initialized = true;
                    otherOmniDriver[i]->data.m_deviceData.m_quat.clear();
                    otherOmniDriver[i]->data.m_servoDeviceData.m_quat.clear();

                    if (hHDVector[i] == HD_INVALID_HANDLE)
                    {
                        if (otherOmniDriver[i]->d_deviceName.getValue().c_str() == HD_DEFAULT_DEVICE)
                        {
                            serr << "[OmniDriver] Failed to initialize, default name isn't supported ; change the name of your new omni device using its configuration manager." << sendl;
                            return -1;
                        }
                        hHDVector[i] = hdInitDevice(otherOmniDriver[i]->d_deviceName.getValue().c_str());

                        if (HD_DEVICE_ERROR(error = hdGetError()))
                        {
                            serr << "[OmniDriver] Failed to initialize the device " << otherOmniDriver[i]->d_deviceName.getValue() << sendl;
                        }
                        else
                        {
                            sout << d_deviceName.getValue() << "[OmniDriver] Found device " << otherOmniDriver[i]->d_deviceName.getValue() << sendl;

                            hdEnable(HD_FORCE_OUTPUT);
                            hdEnable(HD_MAX_FORCE_CLAMPING);
                        }
                    }
                }

                doUpdate = 0;
                //Start the servo loop scheduler.
                hdStartScheduler();
                if (HD_DEVICE_ERROR(error = hdGetError()))
                {
                    serr << "[OmniDriver] Failed to start the scheduler" << sendl;
                }

                for (unsigned int i = 0; i<otherOmniDriver.size(); i++)
                {
                    otherOmniDriver[i]->data.m_servoDeviceData.m_ready = false;
                    otherOmniDriver[i]->data.m_servoDeviceData.m_stop = false;
                }

                hStateHandle = hdScheduleAsynchronous(stateCallback, (void*)&otherOmniDriver, HD_DEFAULT_SCHEDULER_PRIORITY);

                if (HD_DEVICE_ERROR(error = hdGetError()))
                {
                    OmniDriver::printError(&error, "Error with the omni");
                    serr << d_deviceName.getValue() << sendl;
                }
                return 0;
            }

            //constructeur
            OmniDriver::OmniDriver()
                : d_forceScale(initData(&d_forceScale, 1.0, "forceScale", "Default forceScale applied to the force feedback. "))
                , d_scale(initData(&d_scale, 100.0, "scale", "Default scale applied to the Phantom Coordinates. "))
                , d_positionBase(initData(&d_positionBase, Vec3d(0, 0, 0), "positionBase", "Position of the interface base in the scene world coordinates"))
                , d_orientationBase(initData(&d_orientationBase, Quat(0, 0, 0, 1), "orientationBase", "Orientation of the interface base in the scene world coordinates"))
                , d_positionTool(initData(&d_positionTool, Vec3d(0, 0, 0), "positionTool", "Position of the tool in the omni end effector frame"))
                , d_orientationTool(initData(&d_orientationTool, Quat(0, 0, 0, 1), "orientationTool", "Orientation of the tool in the omni end effector frame"))
                , d_permanent(initData(&d_permanent, false, "permanent", "Apply the force feedback permanently"))
                , d_posDevice(initData(&d_posDevice, "posDevice", "position of the base of the part of the device"))
                , d_posStylus(initData(&d_posStylus, "posStylus", "position of the base of the stylus"))
                , d_locDOF(initData(&d_locDOF, "locDOF", "localisation of the DOFs MechanicalObject"))
                , d_deviceName(initData(&d_deviceName, std::string("DefaultDevice"), "deviceName", "name of the device"))
                , d_deviceIndex(initData(&d_deviceIndex, 1, "deviceIndex", "index of the device"))
                , d_openTool(initData(&d_openTool, "openTool", "opening of the tool"))
                , d_maxTool(initData(&d_maxTool, 1.0, "maxTool", "maxTool value"))
                , d_minTool(initData(&d_minTool, 0.0, "minTool", "minTool value"))
                , d_openSpeedTool(initData(&d_openSpeedTool, 0.1, "openSpeedTool", "openSpeedTool value"))
                , d_closeSpeedTool(initData(&d_closeSpeedTool, 0.1, "closeSpeedTool", "closeSpeedTool value"))
                , d_useScheduler(initData(&d_useScheduler, false, "useScheduler", "Enable use of OpenHaptics Scheduler methods to synchronize haptics thread"))
                , d_setRestShape(initData(&d_setRestShape, false, "setRestShape", "True to control the rest position instead of the current position directly"))
                , d_applyMappings(initData(&d_applyMappings, true, "applyMappings", "True to enable applying the mappings after setting the position"))
                , d_alignOmniWithCamera(initData(&d_alignOmniWithCamera, true, "alignOmniWithCamera", "True to keep the Omni's movements in the same reference frame as the m_camera"))
                , d_id(initData(&d_id, "id", "id"))
                , d_nupdates(initData(&d_nupdates, "nupdates", "nupdates"))
                , d_buttonState(initData(&d_buttonState, "buttonState", "buttonState"))
                , d_devicePosition(initData(&d_devicePosition, "devicePosition", "d_devicePosition"))
                , d_pos(initData(&d_pos, "pos", "pos"))
                , d_quat(initData(&d_quat, "quat", "quat"))
                , d_ready(initData(&d_ready, "ready", "ready"))
                , d_stop(initData(&d_stop, "stop", "stop"))
                , m_noDeviceDetected(false)
                , m_isFirstDevice(true)
            {
                this->f_listening.setValue(true);
                data.m_forceFeedback = NULL;

                d_id.setGroup("OmniState");
                d_nupdates.setGroup("OmniState");
                d_buttonState.setGroup("OmniState");
                d_devicePosition.setGroup("OmniState");
                d_pos.setGroup("OmniState");
                d_quat.setGroup("OmniState");
                d_ready.setGroup("OmniState");
                d_stop.setGroup("OmniState");

                d_id.setReadOnly(true);
                d_nupdates.setReadOnly(true);
                d_buttonState.setReadOnly(true);
                d_devicePosition.setReadOnly(true);
                d_pos.setReadOnly(true);
                d_quat.setReadOnly(true);
                d_ready.setReadOnly(true);
                d_stop.setReadOnly(true);
            }

            // Destructor
            OmniDriver::~OmniDriver()
            {

            }

            // Stop call back TODO: To launch only from the first interface
            void OmniDriver::cleanup()
            {
                sout << "[OmniDriver] Cleanup()" << std::endl;
                if (m_isFirstDevice)
                {
                    hdScheduleSynchronous(stopCallback, (void*)&otherOmniDriver, HD_MAX_SCHEDULER_PRIORITY);
                }
                m_initialized = false;
            }

            // Configure feedback
            //void OmniDriver::setForceFeedback(LCPForceFeedback<Rigid3dTypes>* ff)
            void OmniDriver::setForceFeedback(ForceFeedback* ff)
            {
                // the forcefeedback is already set
                if (data.m_forceFeedback == ff)
                {
                    return;
                }

                data.m_forceFeedback = ff;
            }

            void OmniDriver::init()
            {
                if (m_isFirstDevice)
                {
                    simulation::Node *context = dynamic_cast<simulation::Node*>(this->getContext()->getRootContext());
                    context->getTreeObjects<OmniDriver>(&otherOmniDriver);
                    sout << " [OmniDriver] Detected OmniDriver : " << sendl;
                    for (unsigned int i = 0; i < otherOmniDriver.size(); i++)
                    {
                        sout << "  device " << i << " = " << otherOmniDriver[i]->getName() << otherOmniDriver[i]->d_deviceName.getValue() << sendl;
                        otherOmniDriver[i]->d_deviceIndex.setValue(i);
                        hHDVector.push_back(HD_INVALID_HANDLE);
                        otherOmniDriver[i]->m_isFirstDevice = false;
                        otherOmniDriver[i]->data.m_currentForce[0] = 0;
                        otherOmniDriver[i]->data.m_currentForce[1] = 0;
                        otherOmniDriver[i]->data.m_currentForce[2] = 0;
                    }
                    m_isFirstDevice = true;
                }

                sout << d_deviceName.getValue() + " init" << sendl;

                if (d_alignOmniWithCamera.getValue())
                {
                    m_camera = this->getContext()->get<component::visualmodel::InteractiveCamera>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
                    if (!m_camera)
                    {
                        m_camera = this->getContext()->get<component::visualmodel::InteractiveCamera>();
                    }
                    if (!m_camera)
                    {
                        sofa::simulation::Node::SPtr groot = dynamic_cast<simulation::Node*>(this->getContext());
                        m_camera = sofa::core::objectmodel::New<component::visualmodel::InteractiveCamera>();
                        m_camera->setName(core::objectmodel::Base::shortName(m_camera.get()));
                        groot->addObject(m_camera);
                        m_camera->bwdInit();
                    }
                    if (!m_camera)
                    {
                        serr << "Cannot find or create m_camera." << sendl;
                    }
                }

                VecCoord& posD = (*d_posDevice.beginEdit());
                posD.resize(NVISUALNODE + 1);
                d_posDevice.endEdit();
                m_initialized = false;
            }


            void OmniDriver::bwdInit()
            {
                sout << "[OmniDriver] bwdInit()" << sendl;

                simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node

                                                                                                  // Added by Valerian : use tag to find which ff is associated to the OmniDriver otherwise all omnis will hace the same ff
                sofa::core::objectmodel::Tag tag(d_deviceName.getValue());
                ForceFeedback* ff = context->get<ForceFeedback>(tag, sofa::core::objectmodel::BaseContext::SearchRoot);
                if (ff)
                {
                    setForceFeedback(ff);
                }
                else
                {
                    serr << "[OmniDriver] No ForceFeedback found" << sendl;
                }

                setDataValue();

                if (m_isFirstDevice && initDevice() == -1)
                {
                    m_noDeviceDetected = true;
                    serr << "[OmniDriver] No device detected" << sendl;
                }

                DOFs = context->get<sofa::component::container::MechanicalObject<sofa::defaulttype::Rigid3dTypes> >(this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);

                if (DOFs == NULL)
                {
                    serr << "[OmniDriver] No MechanicalObject with a Rigid template found" << sendl;
                }
                else
                {
                    otherOmniDriver[this->d_deviceIndex.getValue()]->DOFs = DOFs;
                }
            }

            void OmniDriver::setDataValue()
            {
                data.m_scale = d_scale.getValue();
                data.m_forceScale = d_forceScale.getValue();

                Quat q = d_orientationBase.getValue();
                q.normalize();
                d_orientationBase.setValue(q);
                data.m_world_H_baseOmni.set(d_positionBase.getValue(), q);
                q = d_orientationTool.getValue();
                q.normalize();
                data.m_endOmni_H_virtualTool.set(d_positionTool.getValue(), q);
                data.m_permanent_feedback = d_permanent.getValue();
            }


            // Run everything but reset
            void OmniDriver::reset()
            {
                sout << "[OmniDriver] Reset() called" << sofa::core::objectmodel::Base::sendl;
                this->reinit();
            }

            // Run everything but reset
            void OmniDriver::reinit()
            {
                sout << "[OmniDriver] Reinit() called" << sofa::core::objectmodel::Base::sendl;

                this->cleanup();
                this->bwdInit();

                sout << "[OmniDriver] Reinit() done" << sofa::core::objectmodel::Base::sendl;
            }

            void OmniDriver::updateDataValueVizualisation()
            {
                d_id.setValue(data.m_deviceData.m_id);
                d_nupdates.setValue(data.m_deviceData.m_nupdates);
                d_buttonState.setValue(data.m_deviceData.m_buttonState);
                d_devicePosition.setValue(data.m_deviceData.m_devicePosition);
                d_pos.setValue(data.m_deviceData.m_pos);
                d_quat.setValue(data.m_deviceData.m_quat);
                d_ready.setValue(data.m_deviceData.m_ready);
                d_stop.setValue(data.m_deviceData.m_stop);
            }

            void OmniDriver::onAnimateBeginEvent()
            {
                updateDataValueVizualisation();

                //Propagate Button Event
                int currentToolIndex = data.m_deviceData.m_id;
                unsigned char buttonState = data.m_deviceData.m_buttonState;

                Vector3 dummyVector;
                Quat dummyQuat;
                sofa::core::objectmodel::HapticDeviceEvent event(currentToolIndex, dummyVector, dummyQuat, buttonState);
                simulation::Node *groot = dynamic_cast<simulation::Node *>(getContext()->getRootContext());
                groot->propagateEvent(core::ExecParams::defaultInstance(), &event);

                // copy data->servoDeviceData to gDeviceData
                if (d_useScheduler.getValue())
                {
                    hdScheduleSynchronous(copyDeviceDataCallback, (void*)&otherOmniDriver, HD_MAX_SCHEDULER_PRIORITY);
                }
                else
                {
                    doUpdate.inc(); // set to 1
                    while (doUpdate)
                    {
#ifdef SOFA_HAVE_BOOST
                        boost::thread::yield();
#else
                        sofa::helper::system::thread::CTime::sleep(0);
#endif
                    }
                }
                if (data.m_deviceData.m_ready)
                {
                    data.m_deviceData.m_quat.normalize();

                    // COMPUTATION OF THE virtualTool 6D POSITION IN THE World COORDINATES
                    SolidTypes<double>::Transform baseOmni_H_endOmni(data.m_deviceData.m_pos*data.m_scale, data.m_deviceData.m_quat);


                    Quat& orientB = (*d_orientationBase.beginEdit());
                    Vec3d& posB = (*d_positionBase.beginEdit());
                    if (d_alignOmniWithCamera.getValue())
                    {
                        Quat m_cameraRotation = m_camera->getOrientation();
                        orientB = m_cameraRotation;
                    }
                    orientB.normalize();
                    data.m_world_H_baseOmni.set(posB, orientB);
                    d_orientationBase.endEdit();
                    d_positionBase.endEdit();

                    VecCoord& posD = (*d_posDevice.beginEdit());

                    SolidTypes<double>::Transform world_H_virtualTool = data.m_world_H_baseOmni * baseOmni_H_endOmni * data.m_endOmni_H_virtualTool;
                    SolidTypes<double>::Transform tampon = data.m_world_H_baseOmni;

                    sofa::helper::Quater<double> q;

                    //get position base
                    posD[0].getCenter() = tampon.getOrigin();
                    posD[0].getOrientation() = tampon.getOrientation();

                    //get position stylus
                    tampon *= baseOmni_H_endOmni;
                    posD[1 + VN_stylus] = Coord(tampon.getOrigin(), tampon.getOrientation());

                    //get pos joint 2
                    sofa::helper::Quater<double> quarter2(Vec3d(0.0, 0.0, 1.0), angle2[2]);
                    SolidTypes<double>::Transform transform_segr2(Vec3d(0.0, 0.0, 0.0), quarter2);
                    tampon *= transform_segr2;
                    posD[1 + VN_joint2] = Coord(tampon.getOrigin(), tampon.getOrientation());

                    //get pos joint 1
                    sofa::helper::Quater<double> quarter3(Vec3d(1.0, 0.0, 0.0), angle2[1]);
                    SolidTypes<double>::Transform transform_segr3(Vec3d(0.0, 0.0, 0.0), quarter3);
                    tampon *= transform_segr3;
                    posD[1 + VN_joint1] = Coord(tampon.getOrigin(), tampon.getOrientation());

                    //get pos arm 2
                    sofa::helper::Quater<double> quarter4(Vec3d(0.0, 1.0, 0.0), -angle2[0]);
                    SolidTypes<double>::Transform transform_segr4(Vec3d(0.0, 0.0, 0.0), quarter4);
                    tampon *= transform_segr4;
                    posD[1 + VN_arm2] = Coord(tampon.getOrigin(), tampon.getOrientation());
                    //get pos arm 1
                    sofa::helper::Quater<double> quarter5(Vec3d(1.0, 0.0, 0.0), -(M_PI / 2) + angle1[2] - angle1[1]);
                    SolidTypes<double>::Transform transform_segr5(Vec3d(0.0, 13.33*data.m_scale / 100, 0.0), quarter5);
                    tampon *= transform_segr5;
                    posD[1 + VN_arm1] = Coord(tampon.getOrigin(), tampon.getOrientation());

                    //get pos joint 0
                    sofa::helper::Quater<double> quarter6(Vec3d(1.0, 0.0, 0.0), angle1[1]);
                    SolidTypes<double>::Transform transform_segr6(Vec3d(0.0, 13.33*data.m_scale / 100, 0.0), quarter6);
                    tampon *= transform_segr6;
                    posD[1 + VN_joint0] = Coord(tampon.getOrigin(), tampon.getOrientation());

                    //get pos base
                    sofa::helper::Quater<double> quarter7(Vec3d(0.0, 0.0, 1.0), angle1[0]);
                    SolidTypes<double>::Transform transform_segr7(Vec3d(0.0, 0.0, 0.0), quarter7);
                    tampon *= transform_segr7;
                    posD[1 + VN_base] = Coord(tampon.getOrigin(), tampon.getOrientation());

                    //get pos of axes

                    posD[1 + VN_X].getCenter() = data.m_world_H_baseOmni.getOrigin();
                    posD[1 + VN_Y].getCenter() = data.m_world_H_baseOmni.getOrigin();
                    posD[1 + VN_Z].getCenter() = data.m_world_H_baseOmni.getOrigin();
                    posD[1 + VN_X].getOrientation() = (data.m_world_H_baseOmni).getOrientation()*q.axisToQuat(Vec3d(0.0, 0.0, 1.0), -M_PI / 2);
                    posD[1 + VN_Y].getOrientation() = (data.m_world_H_baseOmni).getOrientation()*q.axisToQuat(Vec3d(1.0, 0.0, 0.0), 0);
                    posD[1 + VN_Z].getOrientation() = (data.m_world_H_baseOmni).getOrientation()*q.axisToQuat(Vec3d(1.0, 0.0, 0.0), -M_PI / 2);

                    d_posDevice.endEdit();

                    if (DOFs != NULL)
                    {
                        sofa::helper::WriteAccessor<sofa::core::objectmodel::Data<VecCoord> > x = *DOFs->write(d_setRestShape.getValue() ? sofa::core::VecCoordId::restPosition() : sofa::core::VecCoordId::position());
                        sofa::helper::WriteAccessor<sofa::core::objectmodel::Data<VecCoord> > xfree = *DOFs->write(d_setRestShape.getValue() ? sofa::core::VecCoordId::restPosition() : sofa::core::VecCoordId::freePosition());

                        x[0].getCenter() = world_H_virtualTool.getOrigin();
                        xfree[0].getCenter() = world_H_virtualTool.getOrigin();
                        x[0].getOrientation() = world_H_virtualTool.getOrientation();
                        xfree[0].getOrientation() = world_H_virtualTool.getOrientation();
                    }
                    if (d_applyMappings.getValue())
                    {
                        sofa::simulation::Node *node = dynamic_cast<sofa::simulation::Node*> (this->getContext());
                        if (node)
                        {
                            sofa::simulation::MechanicalPropagatePositionAndVelocityVisitor mechaVisitor(sofa::core::MechanicalParams::defaultInstance()); mechaVisitor.execute(node);
                            sofa::simulation::UpdateMappingVisitor updateVisitor(sofa::core::ExecParams::defaultInstance()); updateVisitor.execute(node);
                        }
                    }
                    //button state
                    Vec1d& openT = (*d_openTool.beginEdit());
                    if (data.m_deviceData.m_buttonState & HD_DEVICE_BUTTON_1)
                    {
                        if (openT[0] > d_minTool.getValue())
                        {
                            openT[0] -= d_closeSpeedTool.getValue();
                        }
                        else
                        {
                            openT[0] = d_minTool.getValue();
                        }
                    }
                    else
                    {
                        if (openT[0] < d_maxTool.getValue())
                            openT[0] += d_openSpeedTool.getValue();
                        else
                            openT[0] = d_maxTool.getValue();
                    }
                    d_openTool.endEdit();

                    if (data.m_forceFeedback)
                    {
                        // store actual position of interface for the forcefeedback (as it will be used as soon as new LCP will be computed)
                        data.m_forceFeedback->setReferencePosition(world_H_virtualTool);
                    }
                    /// TODO : SHOULD INCLUDE VELOCITY !!
                }
                else
                {
                    sout << "data not ready \n" << sofa::core::objectmodel::Base::sendl;
                }
            }

            void OmniDriver::handleEvent(core::objectmodel::Event *event)
            {
                if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
                {
                    onAnimateBeginEvent();
                }
            }

            int OmniDriverClass = core::RegisterObject("Solver to test compliance computation for new articulated system objects")
                .add< OmniDriver >()
                .addAlias("DefaultHapticsDevice")
                ;

            SOFA_DECL_CLASS(OmniDriver)

        } // namespace controller

    } // namespace component

} // namespace sofa