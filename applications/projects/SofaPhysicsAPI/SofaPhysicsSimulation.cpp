#include "SofaPhysicsAPI.h"
#include "SofaPhysicsSimulation_impl.h"

#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glu.h>
#include <sofa/helper/io/ImageBMP.h>
#include <sofa/helper/gl/RAII.h>

#include <sofa/simulation/common/xml/initXml.h>
#include <sofa/simulation/tree/TreeSimulation.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/core/objectmodel/GUIEvent.h>

#include <sofa/helper/system/glut.h>
#include <sofa/simulation/common/GUIFactory.h>

#include <SofaComponentCommon/initComponentCommon.h>
#include <SofaComponentBase/initComponentBase.h>
#include <SofaComponentGeneral/initComponentGeneral.h>
#include <SofaComponentAdvanced/initComponentAdvanced.h>
#include <SofaComponentMisc/initComponentMisc.h>

#include <math.h>
#include <iostream>
#include <sstream>

SofaPhysicsSimulation::SofaPhysicsSimulation(bool useGUI, int GUIFramerate, void* shareRenderingContext)
    : impl(new Impl(useGUI, GUIFramerate, shareRenderingContext))
{
}

SofaPhysicsSimulation::~SofaPhysicsSimulation()
{
    delete impl;
}

bool SofaPhysicsSimulation::load(const char* filename)
{
    return impl->load(filename);
}

void SofaPhysicsSimulation::start()
{
    impl->start();
}

void SofaPhysicsSimulation::stop()
{
    impl->stop();
}

void SofaPhysicsSimulation::step()
{
    impl->step();
}

void SofaPhysicsSimulation::reset()
{
    impl->reset();
}

void SofaPhysicsSimulation::resetView()
{
    impl->resetView();
}

void SofaPhysicsSimulation::sendValue(const char* name, double value)
{
    impl->sendValue(name, value);
}

void SofaPhysicsSimulation::drawGL()
{
    impl->drawGL();
}

unsigned int SofaPhysicsSimulation::getNbOutputMeshes()
{
    return impl->getNbOutputMeshes();
}

unsigned int SofaPhysicsSimulation::getNbOutputMeshTetrahedrons()
{
    return impl->getNbOutputMeshTetrahedrons();
}

SofaPhysicsOutputMesh** SofaPhysicsSimulation::getOutputMeshes()
{
    return impl->getOutputMeshes();
}

SofaPhysicsOutputMeshTetrahedron** SofaPhysicsSimulation::getOutputMeshTetrahedrons()
{
    return impl->getOutputMeshTetrahedrons();
}

bool SofaPhysicsSimulation::isAnimated() const
{
    return impl->isAnimated();
}

void SofaPhysicsSimulation::setAnimated(bool val)
{
    impl->setAnimated(val);
}

double SofaPhysicsSimulation::getTimeStep() const
{
    return impl->getTimeStep();
}

void SofaPhysicsSimulation::setTimeStep(double dt)
{
    impl->setTimeStep(dt);
}

double SofaPhysicsSimulation::getTime() const
{
    return impl->getTime();
}

double SofaPhysicsSimulation::getCurrentFPS() const
{
    return impl->getCurrentFPS();
}

const char* SofaPhysicsSimulation::getSceneFileName() const
{
    return impl->getSceneFileName();
}

unsigned int SofaPhysicsSimulation::getNbDataMonitors()
{
    return impl->getNbDataMonitors();
}

SofaPhysicsDataMonitor** SofaPhysicsSimulation::getDataMonitors()
{
    return impl->getDataMonitors();
}

unsigned int SofaPhysicsSimulation::getNbDataControllers()
{
    return impl->getNbDataControllers();
}

SofaPhysicsDataController** SofaPhysicsSimulation::getDataControllers()
{
    return impl->getDataControllers();
}

#ifdef SOFA_SOFAPHYSICSAPI_HAVE_COPYSCREEN
bool SofaPhysicsSimulation::getCopyScreenRequest(SofaPhysicsCopyScreen* info)
{
    return impl->getCopyScreenRequest(info);
}

void SofaPhysicsSimulation::copyScreen(SofaPhysicsCopyScreen* info)
{
    impl->copyScreen(info);
}
#endif

////////////////////////////////////////
////////////////////////////////////////
////////////////////////////////////////

using namespace sofa::defaulttype;
using namespace sofa::helper::gl;
using namespace sofa::core::objectmodel;

static sofa::core::ObjectFactory::ClassEntry::SPtr classVisualModel;

SofaPhysicsSimulation::Impl::Impl(bool useGUI_, int GUIFramerate_, void* shareRenderingContext) :
useGUI(useGUI_), GUIFramerate(GUIFramerate_), GUIShareRenderingContext(shareRenderingContext)
{
    static bool first = true;
    if (first)
    {
        sofa::component::initComponentBase();
        sofa::component::initComponentCommon();
        sofa::component::initComponentGeneral();
        sofa::component::initComponentAdvanced();
        sofa::component::initComponentMisc();

        sofa::simulation::xml::initXml();

        if (useGUI)
        {
            int argc = 1;
            char* argv[] = { const_cast<char*>("a") };
            glutInit(&argc, argv);

            std::string guiName("SofaGuiQt");
            if (sofa::simulation::gui::initGUI(guiName))
            {
                std::vector<std::string> options;
                if (GUIShareRenderingContext)
                {
                    std::ostringstream o;
                    o << "shareRenderingContext=" << (unsigned long long)GUIShareRenderingContext;
                    std::cout << o.str() << std::endl;
                    options.push_back(o.str());
                }
                auto* gui = sofa::simulation::gui::createGUI(guiName, argv[0],options);
                if (gui)
                {
                    gui->initialize();
                    gui->setViewerResolution(600, 600);
                }
                else
                {
                    useGUI = false;
                }
            }
            else
            {
                useGUI = false;
            }
        }

        first = false;
    }

    m_RootNode = NULL;
    initGLDone = false;
    initTexturesDone = false;
    texLogo = NULL;
    lastW = 0;
    lastH = 0;
    vparams = sofa::core::visual::VisualParams::defaultInstance();
    vparams->drawTool() = &drawTool;

    m_Simulation = new sofa::simulation::tree::TreeSimulation();
    sofa::simulation::setSimulation(m_Simulation);

    sofa::core::ObjectFactory::AddAlias("VisualModel", "OglModel", true,
            &classVisualModel);

    sofa::helper::system::PluginManager::getInstance().init();

    timeTicks = sofa::helper::system::thread::CTime::getRefTicksPerSec();
    frameCounter = 0;
    currentFPS = 0.0;
    lastRedrawTime = 0;
}

SofaPhysicsSimulation::Impl::~Impl()
{
    for (std::map<SofaOutputMesh*, SofaPhysicsOutputMesh*>::const_iterator it = outputMeshMap.begin(), itend = outputMeshMap.end(); it != itend; ++it)
    {
        if (it->second) delete it->second;
    }
    outputMeshMap.clear();

    if ( m_RootNode.get() ) {
        m_Simulation->unload ( m_RootNode );
    }
    if (useGUI) {
        sofa::simulation::gui::closeGUI();
    }
}

bool SofaPhysicsSimulation::Impl::load(const char* cfilename)
{
    std::string filename = cfilename;
    std::cout << "FROM APP: SofaPhysicsSimulation::load(" << filename << ")" << std::endl;
    sofa::helper::BackTrace::autodump();

    //bool wasAnimated = isAnimated();
    bool success = true;
    sofa::helper::system::DataRepository.findFile(filename);
    m_RootNode = m_Simulation->load(filename.c_str());

    if (m_RootNode.get())
    {
        sceneFileName = filename;
        std::cout << "INIT" << std::endl;
        m_Simulation->init(m_RootNode.get());
        getAllOutputMeshes();
        updateOutputMeshes();

        if (useGUI) {
            auto* gui = sofa::simulation::gui::getCurrentGUI();
            gui->setScene(m_RootNode.get(), cfilename);
        }
    }
    else
    {
        m_RootNode = m_Simulation->createNewGraph("");
        success = false;
    }
    initTexturesDone = false;
    lastW = 0;
    lastH = 0;
    lastRedrawTime = sofa::helper::system::thread::CTime::getRefTime();

//    if (isAnimated() != wasAnimated)
//        animatedChanged();
    return success;
}


void SofaPhysicsSimulation::Impl::sendValue(const char* name, double value)
{
    // send a GUIEvent to the tree
    if (m_RootNode!=0)
    {
        std::ostringstream oss;
        oss << value;
        sofa::core::objectmodel::GUIEvent event("",name,oss.str().c_str());
        m_RootNode->propagateEvent(sofa::core::ExecParams::defaultInstance(), &event);
    }
    this->update();
}

bool SofaPhysicsSimulation::Impl::isAnimated() const
{
    if (getScene())
        return getScene()->getContext()->getAnimate();
    return false;
}

void SofaPhysicsSimulation::Impl::setAnimated(bool val)
{
    if (val) start();
    else stop();
}

double SofaPhysicsSimulation::Impl::getTimeStep() const
{
    if (getScene())
        return getScene()->getContext()->getDt();
    else
        return 0.0;
}

void SofaPhysicsSimulation::Impl::setTimeStep(double dt)
{
    if (getScene())
    {
        getScene()->getContext()->setDt(dt);
    }
}

double SofaPhysicsSimulation::Impl::getTime() const
{
    if (getScene())
        return getScene()->getContext()->getTime();
    else
        return 0.0;
}

double SofaPhysicsSimulation::Impl::getCurrentFPS() const
{
    return currentFPS;
}


void SofaPhysicsSimulation::Impl::start()
{
    std::cout << "FROM APP: start()" << std::endl;
    if (isAnimated()) return;
    if (getScene())
    {
        getScene()->getContext()->setAnimate(true);
        //animatedChanged();
    }
}

void SofaPhysicsSimulation::Impl::stop()
{
    std::cout << "FROM APP: stop()" << std::endl;
    if (!isAnimated()) return;
    if (getScene())
    {
        getScene()->getContext()->setAnimate(false);
        //animatedChanged();
    }
}


void SofaPhysicsSimulation::Impl::reset()
{
    std::cout << "FROM APP: reset()" << std::endl;
    if (getScene())
    {
        getSimulation()->reset(getScene());
        this->update();
    }
}

void SofaPhysicsSimulation::Impl::resetView()
{
    if (getScene() && currentCamera)
    {
        currentCamera->setDefaultView(getScene()->getGravity());
        bool fileRead = false;
        if (!sceneFileName.empty())
        {
            std::string viewFileName = sceneFileName + ".view";
            std::cout << "SofaPhysicsSimulation: Trying view file " << viewFileName << std::endl;
            fileRead = currentCamera->importParametersFromFile(viewFileName);
            if (!fileRead) // try default.scn.view
            {
                viewFileName = sofa::helper::system::SetDirectory::GetRelativeFromFile("default.scn.view", sceneFileName.c_str());
                std::cout << "SofaPhysicsSimulation: Trying view file " << viewFileName << std::endl;
                fileRead = currentCamera->importParametersFromFile(viewFileName);
            }
            //if there is no .view file , look at the center of the scene bounding box
            // and with a Up vector in the same axis as the gravity
            if (!fileRead)
            {
                std::cout << "SofaPhysicsSimulation: No view file" << std::endl;
                currentCamera->setDefaultView(getScene()->getGravity());
            }
        }
    }
}

void SofaPhysicsSimulation::Impl::update()
{
}

void SofaPhysicsSimulation::Impl::step()
{
    sofa::simulation::Node* groot = getScene();
    if (!groot) return;
    beginStep();
    getSimulation()->animate(groot);
    getSimulation()->updateVisual(groot);
    if (useGUI) {
        auto* gui = sofa::simulation::gui::getCurrentGUI();
        gui->stepMainLoop();
        if (GUIFramerate)
        {
            sofa::helper::system::thread::ctime_t curtime = sofa::helper::system::thread::CTime::getRefTime();
            if ((curtime-lastRedrawTime) > (double)timeTicks/GUIFramerate)
            {
                lastRedrawTime = curtime;
                gui->redraw();
            }
        }
    }
    endStep();
}

void SofaPhysicsSimulation::Impl::beginStep()
{
}

void SofaPhysicsSimulation::Impl::endStep()
{
    update();
    updateCurrentFPS();
    updateOutputMeshes();
}

void SofaPhysicsSimulation::Impl::updateCurrentFPS()
{
    if (frameCounter==0)
    {
        sofa::helper::system::thread::ctime_t t = sofa::helper::system::thread::CTime::getRefTime();
        for (int i=0; i<10; i++)
            stepTime[i] = t;
    }
    else
    {
        if ((frameCounter%10) == 0)
        {
            sofa::helper::system::thread::ctime_t curtime = sofa::helper::system::thread::CTime::getRefTime();
            int i = ((frameCounter/10)%10);
            currentFPS = ((double)timeTicks / (curtime - stepTime[i]))*(frameCounter<100?frameCounter:100);
            stepTime[i] = curtime;

            if (useGUI) {
                auto* gui = sofa::simulation::gui::getCurrentGUI();
                gui->showFPS(currentFPS);
            }
        }
    }
    ++frameCounter;
}

void SofaPhysicsSimulation::Impl::updateOutputMeshes()
{
    outputMeshes.clear();
    outputMeshTetrahedrons.clear();
    outputMeshes.reserve(allSofaOutputMeshes.size());
    outputMeshTetrahedrons.reserve(allSofaOutputMeshTetrahedrons.size());

    for (unsigned int i=0; i<allSofaOutputMeshes.size(); ++i)
    {
        SofaOutputMesh* sMesh = allSofaOutputMeshes[i];
        if (sMesh->getContext()->isActive())
        {
            SofaPhysicsOutputMesh* oMesh = allOutputMeshes[i];
            outputMeshes.emplace_back(oMesh);
        }
    }
    for( unsigned int i = 0; i<allSofaOutputMeshTetrahedrons.size();++i )
    {
        SofaOutputMeshTetrahedron* sMeshTetra = allSofaOutputMeshTetrahedrons[i];
        if (sMeshTetra->getContext()->isActive())
        {
            SofaPhysicsOutputMeshTetrahedron* oMeshTetra = allOutputMeshTetrahedrons[i];
            outputMeshTetrahedrons.emplace_back(oMeshTetra);
        }
    }
}

void SofaPhysicsSimulation::Impl::getAllOutputMeshes()
{
    sofa::simulation::Node* groot = getScene();
    if (!groot)
    {
        allSofaOutputMeshes.clear();
        allSofaOutputMeshTetrahedrons.clear();
        allOutputMeshes.clear();
        allOutputMeshTetrahedrons.clear();
        return;
    }
    allSofaOutputMeshes.clear();
    allSofaOutputMeshTetrahedrons.clear();

    groot->get<SofaOutputMesh>(&allSofaOutputMeshes, BaseContext::SearchDown);
    groot->get<SofaOutputMeshTetrahedron>(&allSofaOutputMeshTetrahedrons, BaseContext::SearchDown);


    allOutputMeshes.resize(allSofaOutputMeshes.size());
    allOutputMeshTetrahedrons.resize(allSofaOutputMeshTetrahedrons.size());

    for (unsigned int i = 0; i<allSofaOutputMeshes.size(); ++i)
    {
        SofaOutputMesh* sMesh = allSofaOutputMeshes[i];
        SofaPhysicsOutputMesh*& oMesh = outputMeshMap[sMesh];
        if (oMesh == NULL)
        {
            oMesh = new SofaPhysicsOutputMesh;
            oMesh->impl->setObject(sMesh);
        }
        allOutputMeshes[i] = oMesh;
    }
    for (unsigned int i = 0; i<allSofaOutputMeshTetrahedrons.size(); ++i)
    {
        SofaOutputMeshTetrahedron* sMeshTetra = allSofaOutputMeshTetrahedrons.at(i);
        SofaPhysicsOutputMeshTetrahedron*& oMeshTetra = outputMeshMapTetrahedron[sMeshTetra];
        if (oMeshTetra == NULL)
        {
            oMeshTetra = new SofaPhysicsOutputMeshTetrahedron;
            oMeshTetra->impl->setObject(sMeshTetra);
        }
        allOutputMeshTetrahedrons[i] = oMeshTetra;
    }
}

unsigned int SofaPhysicsSimulation::Impl::getNbOutputMeshes()
{
    return outputMeshes.size();
}

unsigned int SofaPhysicsSimulation::Impl::getNbOutputMeshTetrahedrons()
{
    return outputMeshTetrahedrons.size();
}

SofaPhysicsOutputMesh** SofaPhysicsSimulation::Impl::getOutputMeshes()
{
    if (outputMeshes.empty())
        return NULL;
    else
        return &(outputMeshes[0]);
}

SofaPhysicsOutputMeshTetrahedron** SofaPhysicsSimulation::Impl::getOutputMeshTetrahedrons()
{
    if( outputMeshTetrahedrons.empty() ) 
        return NULL;
    else
        return &(outputMeshTetrahedrons[0]);
}

unsigned int SofaPhysicsSimulation::Impl::getNbDataMonitors()
{
    return dataMonitors.size();
}

SofaPhysicsDataMonitor** SofaPhysicsSimulation::Impl::getDataMonitors()
{
    if (dataMonitors.empty())
    {
        sofa::simulation::Node* groot = getScene();
        if (!groot)
        {
            return NULL;
        }
        groot->get<SofaDataMonitor>(&sofaDataMonitors, BaseContext::SearchDown);
        dataMonitors.resize(sofaDataMonitors.size());
        for (unsigned int i=0; i<sofaDataMonitors.size(); ++i)
        {
            SofaDataMonitor* sData = sofaDataMonitors[i];
            SofaPhysicsDataMonitor* oData = new SofaPhysicsDataMonitor;
            oData->impl->setObject(sData);
            dataMonitors[i] = oData;
        }
    }
    return &(dataMonitors[0]);
}

unsigned int SofaPhysicsSimulation::Impl::getNbDataControllers()
{
    return dataControllers.size();
}

SofaPhysicsDataController** SofaPhysicsSimulation::Impl::getDataControllers()
{
    if (dataControllers.empty())
    {
        sofa::simulation::Node* groot = getScene();
        if (!groot)
        {
            return NULL;
        }
        groot->get<SofaDataController>(&sofaDataControllers, BaseContext::SearchDown);
        dataControllers.resize(sofaDataControllers.size());
        for (unsigned int i=0; i<sofaDataControllers.size(); ++i)
        {
            SofaDataController* sData = sofaDataControllers[i];
            SofaPhysicsDataController* oData = new SofaPhysicsDataController;
            oData->impl->setObject(sData);
            dataControllers[i] = oData;
        }
    }
    return &(dataControllers[0]);
}

void SofaPhysicsSimulation::Impl::drawGL()
{
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    if (!initGLDone)
    {
#ifdef SOFA_HAVE_GLEW
        glewInit();
#endif
        //Load texture for logo
        texLogo = new sofa::helper::gl::Texture(new sofa::helper::io::ImageBMP( sofa::helper::system::DataRepository.getFile("textures/SOFA_logo.bmp")));
        texLogo->init();

        initGLDone = true;
    }

    const int vWidth = viewport[2];
    const int vHeight = viewport[3];

    if (texLogo && texLogo->getImage())
    {
        int w = 0;
        int h = 0;
        h = texLogo->getImage()->getHeight();
        w = texLogo->getImage()->getWidth();

        Enable <GL_TEXTURE_2D> tex;
        glDisable(GL_DEPTH_TEST);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(-0.5, vWidth, -0.5, vHeight, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        if (texLogo)
            texLogo->bind();

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);

        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_QUADS);
        glTexCoord2d(0.0, 0.0);
        glVertex3d((vWidth-w)/2, (vHeight-h)/2, 0.0);

        glTexCoord2d(1.0, 0.0);
        glVertex3d( vWidth-(vWidth-w)/2, (vHeight-h)/2, 0.0);

        glTexCoord2d(1.0, 1.0);
        glVertex3d( vWidth-(vWidth-w)/2, vHeight-(vHeight-h)/2, 0.0);

        glTexCoord2d(0.0, 1.0);
        glVertex3d((vWidth-w)/2, vHeight-(vHeight-h)/2, 0.0);
        glEnd();

        glBindTexture(GL_TEXTURE_2D, 0);

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glDisable(GL_BLEND);
    }

    if (m_RootNode.get())
    {
        sofa::simulation::Node* groot = m_RootNode.get();
        if (!initTexturesDone)
        {
            std::cout << "INIT VISUAL" << std::endl;
            getSimulation()->initTextures(groot);
            bool setView = false;
            groot->get(currentCamera);
            if (!currentCamera)
            {
                currentCamera = sofa::core::objectmodel::New<sofa::component::visualmodel::InteractiveCamera>();
                currentCamera->setName(sofa::core::objectmodel::Base::shortName(currentCamera.get()));
                groot->addObject(currentCamera);
                currentCamera->p_position.forceSet();
                currentCamera->p_orientation.forceSet();
                currentCamera->bwdInit();
            }
            setView = true;
            //}

            vparams->sceneBBox() = groot->f_bbox.getValue();
            currentCamera->setBoundingBox(vparams->sceneBBox().minBBox(), vparams->sceneBBox().maxBBox());
            currentCamera->setViewport(vWidth, vHeight);
            if (setView)
                resetView();
            std::cout << "initTexturesDone" << std::endl;
            initTexturesDone = true;
        }

        glDepthFunc(GL_LEQUAL);
        glClearDepth(1.0);
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_NORMALIZE);
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        static GLfloat    lightPosition[4] = { -0.7f, 0.3f, 0.0f, 1.0f};
        static GLfloat    lmodel_ambient[]    = {0.0f, 0.0f, 0.0f, 0.0f};
        static GLfloat    ambientLight[4] = { 0.5f, 0.5f, 0.5f, 1.0f};
        static GLfloat    diffuseLight[4] = { 0.9f, 0.9f, 0.9f, 1.0f};
        static GLfloat    specularLight[4] = { 1.0f, 1.0f, 1.0f, 1.0f};
        static GLfloat    specularMat[4] = { 1.0f, 1.0f, 1.0f, 1.0f};
        glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

        // Setup 'light 0'
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
        glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
        glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

        // Enable color tracking
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

        // All materials hereafter have full specular reflectivity with a high shine
        glMaterialfv(GL_FRONT, GL_SPECULAR, specularMat);
        glMateriali(GL_FRONT, GL_SHININESS, 128);

        glShadeModel(GL_SMOOTH);

        // Define background color
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);

        // Turn on our light and enable color along with the light
        //glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glColor4f(1,1,1,1);
        glDisable(GL_COLOR_MATERIAL);
        glEnable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);

        vparams->sceneBBox() = groot->f_bbox.getValue();

        vparams->viewport() = sofa::helper::make_array(viewport[0], viewport[1], viewport[2], viewport[3]);

        if (vWidth != lastW || vHeight != lastH)
        {
            lastW = vWidth;
            lastH = vHeight;
            if (currentCamera)
                currentCamera->setViewport(vWidth, vHeight);
            calcProjection();
        }
        currentCamera->getOpenGLMatrix(lastModelviewMatrix);
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixd(lastProjectionMatrix);
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixd(lastModelviewMatrix);

        getSimulation()->draw(vparams,groot);

        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glDisableClientState(GL_NORMAL_ARRAY);

    }



    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}


// ---------------------------------------------------------
// --- Reshape of the window, reset the projection
// ---------------------------------------------------------
void SofaPhysicsSimulation::Impl::calcProjection()
{
    int width = lastW;
    int height = lastH;
    double xNear, yNear/*, xOrtho, yOrtho*/;
    double xFactor = 1.0, yFactor = 1.0;
    //double offset;
    //double xForeground, yForeground, xBackground, yBackground;
    //double zForeground, zBackground;
    Vector3 center;

    /// Camera part
    if (!currentCamera)
        return;
    sofa::simulation::Node* groot = getScene();
    if (groot && (!groot->f_bbox.getValue().isValid()))
    {
        vparams->sceneBBox() = groot->f_bbox.getValue();
        currentCamera->setBoundingBox(vparams->sceneBBox().minBBox(), vparams->sceneBBox().maxBBox());
    }
    currentCamera->computeZ();

    vparams->zNear() = currentCamera->getZNear();
    vparams->zFar() = currentCamera->getZFar();

    xNear = 0.35 * vparams->zNear();
    yNear = 0.35 * vparams->zNear();
    //offset = 0.001 * vparams->zNear(); // for foreground and background planes

    /*xOrtho = fabs(vparams->sceneTransform().translation[2]) * xNear
            / vparams->zNear();
    yOrtho = fabs(vparams->sceneTransform().translation[2]) * yNear
            / vparams->zNear();*/

    if ((height != 0) && (width != 0))
    {
        if (height > width)
        {
            xFactor = 1.0;
            yFactor = (double) height / (double) width;
        }
        else
        {
            xFactor = (double) width / (double) height;
            yFactor = 1.0;
        }
    }
    vparams->viewport() = sofa::helper::make_array(0,0,width,height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    xFactor *= 0.01;
    yFactor *= 0.01;

    //std::cout << xNear << " " << yNear << std::endl;

    //zForeground = -vparams->zNear() - offset;
    //zBackground = -vparams->zFar() + offset;

    if (currentCamera->getCameraType() == sofa::core::visual::VisualParams::PERSPECTIVE_TYPE)
        gluPerspective(currentCamera->getFieldOfView(), (double) width / (double) height, vparams->zNear(), vparams->zFar());
    else
    {
        double ratio = vparams->zFar() / (vparams->zNear() * 20);
        Vector3 tcenter = vparams->sceneTransform() * center;
        if (tcenter[2] < 0.0)
        {
            ratio = -300 * (tcenter.norm2()) / tcenter[2];
        }
        glOrtho((-xNear * xFactor) * ratio, (xNear * xFactor) * ratio, (-yNear
                * yFactor) * ratio, (yNear * yFactor) * ratio,
                vparams->zNear(), vparams->zFar());
    }

    //xForeground = -zForeground * xNear / vparams->zNear();
    //yForeground = -zForeground * yNear / vparams->zNear();
    //xBackground = -zBackground * xNear / vparams->zNear();
    //yBackground = -zBackground * yNear / vparams->zNear();

    //xForeground *= xFactor;
    //yForeground *= yFactor;
    //xBackground *= xFactor;
    //yBackground *= yFactor;

    glGetDoublev(GL_PROJECTION_MATRIX,lastProjectionMatrix);

    glMatrixMode(GL_MODELVIEW);
}

#ifdef SOFA_SOFAPHYSICSAPI_HAVE_COPYSCREEN
bool SofaPhysicsSimulation::Impl::getCopyScreenRequest(SofaPhysicsCopyScreen* info)
{
    bool res = false;
    if (useGUI)
    {
        auto* gui = sofa::simulation::gui::getCurrentGUI();
        sofa::simulation::gui::BaseGUI::CopyScreenInfo ginfo;
        ginfo.ctx = info->ctx;
        ginfo.name = info->name;
        ginfo.target = info->target;
        ginfo.srcX = info->srcX;
        ginfo.srcY = info->srcY;
        ginfo.dstX = info->dstX;
        ginfo.dstY = info->dstY;
        ginfo.width = info->width;
        ginfo.height = info->height;

        res = gui->getCopyScreenRequest(ginfo);
        if (res)
        {
            info->ctx = ginfo.ctx;
            info->name = ginfo.name;
            info->target = ginfo.target;
            info->level = 0;
            info->srcX = ginfo.srcX;
            info->srcY = ginfo.srcY;
            info->srcZ = 0;
            info->dstX = ginfo.dstX;
            info->dstY = ginfo.dstY;
            info->dstZ = 0;
            info->width = ginfo.width;
            info->height = ginfo.height;
            info->depth = 1;
        }
    }
    return res;
}

void SofaPhysicsSimulation::Impl::copyScreen(SofaPhysicsCopyScreen* info)
{
    if (useGUI)
    {
        auto* gui = sofa::simulation::gui::getCurrentGUI();
        sofa::simulation::gui::BaseGUI::CopyScreenInfo ginfo;
        ginfo.ctx = info->ctx;
        ginfo.name = info->name;
        ginfo.target = info->target;
        ginfo.srcX = info->srcX;
        ginfo.srcY = info->srcY;
        ginfo.dstX = info->dstX;
        ginfo.dstY = info->dstY;
        ginfo.width = info->width;
        ginfo.height = info->height;
        gui->useCopyScreen(ginfo);
    }
}
#endif
