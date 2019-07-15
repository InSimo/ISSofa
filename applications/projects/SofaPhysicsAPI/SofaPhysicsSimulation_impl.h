#ifndef SOFAPHYSICSSIMULATION_IMPL_H
#define SOFAPHYSICSSIMULATION_IMPL_H

#include "SofaPhysicsAPI.h"
#include "SofaPhysicsOutputMesh_impl.h"
#include "SofaPhysicsOutputMesh_Tetrahedron_impl.h"
#include "SofaPhysicsDataMonitor_impl.h"
#include "SofaPhysicsDataController_impl.h"

#include <sofa/simulation/common/Simulation.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/visual/DrawToolGL.h>
#include <SofaBaseVisual/InteractiveCamera.h>
#include <sofa/helper/gl/Texture.h>

#include <map>

class SofaPhysicsSimulation::Impl
{
public:
    Impl(bool useGUI_ = false, int GUIFramerate_ = 0, void* shareRenderingContext = NULL);
    ~Impl();

    bool load(const char* filename);
    void start();
    void stop();
    bool step( double stepDuration = 0 );
    void reset();
    void resetView();
    void sendValue(const char* name, double value);
    void drawGL();

    unsigned int getNbOutputMeshes();
    unsigned int getNbOutputMeshTetrahedrons();
    SofaPhysicsOutputMesh** getOutputMeshes();
    SofaPhysicsOutputMeshTetrahedron** getOutputMeshTetrahedrons();

    bool isAnimated() const;
    void setAnimated(bool val);

    double getTimeStep() const;
    void   setTimeStep(double dt);
    double getTime() const;
    double getCurrentFPS() const;

    unsigned int getNbDataMonitors();
    SofaPhysicsDataMonitor** getDataMonitors();

    unsigned int getNbDataControllers();
    SofaPhysicsDataController** getDataControllers();

#ifdef SOFA_SOFAPHYSICSAPI_HAVE_COPYSCREEN
    bool getCopyScreenRequest(SofaPhysicsCopyScreen* info);
    void copyScreen(SofaPhysicsCopyScreen* info);
#endif

    typedef SofaPhysicsOutputMesh::Impl::SofaOutputMesh SofaOutputMesh;
    typedef SofaPhysicsDataMonitor::Impl::SofaDataMonitor SofaDataMonitor;
    typedef SofaPhysicsDataController::Impl::SofaDataController SofaDataController;
    typedef SofaPhysicsOutputMesh::Impl::SofaVisualOutputMesh SofaVisualOutputMesh;
    typedef SofaPhysicsOutputMeshTetrahedron::Impl::SofaOutputMeshTetrahedron SofaOutputMeshTetrahedron;
    //typedef SofaPhysicsOutputMesh::Impl::SofaOutputMeshTetra SofaOutputMeshTetra;

protected:

    sofa::simulation::Simulation* m_Simulation;
    sofa::simulation::Node::SPtr m_RootNode;
    std::string sceneFileName;
    sofa::component::visualmodel::BaseCamera::SPtr currentCamera;

    std::map<SofaOutputMesh*, SofaPhysicsOutputMesh*> outputMeshMap;
    std::map<SofaOutputMeshTetrahedron*, SofaPhysicsOutputMeshTetrahedron*> outputMeshMapTetrahedron;
    std::vector<SofaPhysicsOutputMesh*> outputMeshes;
    std::vector<SofaPhysicsOutputMeshTetrahedron*> outputMeshTetrahedrons;

    std::vector<SofaOutputMesh*> allSofaOutputMeshes;
    std::vector<SofaOutputMeshTetrahedron*> allSofaOutputMeshTetrahedrons;
    std::vector<SofaPhysicsOutputMesh*> allOutputMeshes;
    std::vector<SofaPhysicsOutputMeshTetrahedron*> allOutputMeshTetrahedrons;

    std::vector<SofaDataMonitor*> sofaDataMonitors;
    std::vector<SofaPhysicsDataMonitor*> dataMonitors;

    std::vector<SofaDataController*> sofaDataControllers;
    std::vector<SofaPhysicsDataController*> dataControllers;

    sofa::helper::gl::Texture *texLogo;
    double lastProjectionMatrix[16];
    double lastModelviewMatrix[16];
    GLint lastW, lastH;
    GLint lastViewport[4];
    bool initGLDone;
    bool initTexturesDone;
    bool useGUI;
    int GUIFramerate;
    void* GUIShareRenderingContext;

    sofa::core::visual::VisualParams* vparams;
    sofa::core::visual::DrawToolGL   drawTool;

    sofa::helper::system::thread::ctime_t stepTime[10];
    sofa::helper::system::thread::ctime_t timeTicks;
    sofa::helper::system::thread::ctime_t lastRedrawTime;
    int frameCounter;
    double currentFPS;

    void update();
    void updateOutputMeshes();
    void getAllOutputMeshes();
    void updateCurrentFPS();
    void beginStep();
    void endStep();
    void calcProjection();

public:

    const char* getSceneFileName() const
    {
        return sceneFileName.c_str();
    }
    sofa::simulation::Simulation* getSimulation() const
    {
        return m_Simulation;
    }
    sofa::simulation::Node* getScene() const
    {
        return m_RootNode.get();
    }

};

#endif // SOFAPHYSICSSIMULATION_IMPL_H
