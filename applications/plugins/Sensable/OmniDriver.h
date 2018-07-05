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
#ifndef SOFA_COMPONENT_OMNIDRIVER_H
#define SOFA_COMPONENT_OMNIDRIVER_H

//Sensable include
#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <sofa/helper/LCPcalc.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/core/behavior/BaseController.h>
#include <SofaUserInteraction/Controller.h>

//force feedback
#include <SofaHaptics/ForceFeedback.h>
#include <SofaHaptics/MechanicalStateForceFeedback.h>
#include <SofaHaptics/LCPForceFeedback.h>
#include <SofaHaptics/NullForceFeedbackT.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <boost/thread/thread.hpp>
#include <SofaBaseVisual/InteractiveCamera.h>

namespace sofa
{

    namespace component
    {

        namespace controller
        {

            class ForceFeedback;

            using namespace sofa::defaulttype;
            using core::objectmodel::Data;

            /** Holds data retrieved from HDAPI. */
            struct NewDeviceData
            {
                HHD m_id;
                int m_nupdates;
                int m_buttonState;					/* Has the device button has been pressed. */
                hduVector3Dd m_devicePosition;	/* Current device coordinates. */
                HDErrorInfo m_error;
                Vec3d m_pos;
                Quat m_quat;
                bool m_ready;
                bool m_stop;
            };

            struct NewOmniData
            {
                ForceFeedback::SPtr     m_forceFeedback;
                simulation::Node::SPtr  m_pContext;

                sofa::defaulttype::SolidTypes<double>::Transform m_endOmni_H_virtualTool;
                sofa::defaulttype::SolidTypes<double>::Transform m_world_H_baseOmni;
                double  m_forceScale;
                double  m_scale;
                bool    m_permanent_feedback;

                // API OMNI //
                NewDeviceData m_servoDeviceData;  // for the haptic loop
                NewDeviceData m_deviceData;		 // for the simulation loop

                double m_currentForce[3];

            };

            /**
            * OmniDriver driver
            */
            class OmniDriver : public Controller
            {
            public:
                SOFA_CLASS(OmniDriver, Controller);
                typedef RigidTypes::Coord Coord;
                typedef RigidTypes::VecCoord VecCoord;

                enum
                {
                    VN_stylus = 0,
                    VN_joint2 = 1,
                    VN_joint1 = 2,
                    VN_arm2 = 3,
                    VN_arm1 = 4,
                    VN_joint0 = 5,
                    VN_base = 6,
                    VN_X = 7,
                    VN_Y = 8,
                    VN_Z = 9,
                    NVISUALNODE = 10
                };

                OmniDriver();
                ~OmniDriver();

                Data<double>        d_forceScale;
                Data<double>        d_scale;
                Data<Vec3d>         d_positionBase;
                Data<Quat>          d_orientationBase;
                Data<Vec3d>         d_positionTool;
                Data<Quat>          d_orientationTool;
                Data<bool>          d_permanent;
                Data< VecCoord >    d_posDevice;
                Data< VecCoord >    d_posStylus;
                Data< std::string > d_locDOF;
                Data< std::string > d_deviceName;
                Data< int >         d_deviceIndex;
                Data<Vec1d>         d_openTool;
                Data<double>        d_maxTool;
                Data<double>        d_minTool;
                Data<double>        d_openSpeedTool;
                Data<double>        d_closeSpeedTool;
                Data<bool>          d_useScheduler;
                Data<bool>          d_setRestShape;
                Data<bool>          d_applyMappings;
                Data<bool>          d_alignOmniWithCamera;

                // Following are readOnly data to get information about
                Data<HHD>            d_id;
                Data<int>            d_nupdates;
                Data<int>            d_buttonState; /* Has the device button has been pressed. */
                Data<hduVector3Dd>   d_devicePosition;	/* Current device coordinates. */
                Data<Vec3d>          d_pos;
                Data<Quat>           d_quat;
                Data<bool>           d_ready;
                Data<bool>           d_stop;


                NewOmniData data;
                HDfloat angle1[3];
                HDfloat angle2[3];
                bool m_initialized;
                bool m_isFirstDevice;
                bool m_noDeviceDetected;
                sofa::component::container::MechanicalObject<sofa::defaulttype::Rigid3dTypes> *DOFs;
                sofa::component::visualmodel::BaseCamera::SPtr m_camera;

                static void printError(const HDErrorInfo *error, const char *message);

                int initDevice();
                void init();
                virtual void bwdInit();
                void setForceFeedback(ForceFeedback* ff);
                void cleanup();
                void setDataValue();
                void reset();
                void reinit();
                void onAnimateBeginEvent();
                void handleEvent(core::objectmodel::Event *event);
            protected:
                void updateDataValueVizualisation();
            };

        } // namespace controller

    } // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_OMNIDRIVER_H
