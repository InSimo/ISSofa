/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <iostream>
#include <sstream>
#include <fstream>
#include <sofa/helper/ArgumentParser.h>
#include <sofa/simulation/common/xml/initXml.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/helper/system/PluginManager.h>
#ifdef SOFA_HAVE_DAG
#include <sofa/simulation/graph/DAGSimulation.h>
#endif
#ifdef SOFA_SMP
#include <sofa/simulation/tree/SMPSimulation.h>
#endif
#include <sofa/simulation/tree/TreeSimulation.h>


#include <SofaComponentCommon/initComponentCommon.h>
#include <SofaComponentBase/initComponentBase.h>
#include <SofaComponentGeneral/initComponentGeneral.h>
#include <SofaComponentAdvanced/initComponentAdvanced.h>
#include <SofaComponentMisc/initComponentMisc.h>

#include <SofaLoader/ReadState.h>
#include <SofaValidation/CompareState.h>
#include <sofa/helper/Factory.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/simulation/common/GUIFactory.h>
#include <sofa/simulation/common/SceneLoaderFactory.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>
#include <sofa/helper/system/atomic.h>
#include <cstdlib>
#ifdef SOFA_SMP
#include <athapascan-1>
#endif /* SOFA_SMP */
#ifdef WIN32
#include <windows.h>
#ifdef IS_TEST_FRAMEWORK
#include <crtdbg.h>
#endif
#endif
using std::cerr;
using std::endl;

void loadVerificationData(std::string& directory, std::string& filename, sofa::simulation::Node* node)
{
    std::cout << "loadVerificationData from " << directory << " and file " << filename << std::endl;

    std::string refFile;

    refFile += directory;
    refFile += '/';
    refFile += sofa::helper::system::SetDirectory::GetFileName(filename.c_str());

    std::cout << "loadVerificationData " << refFile << std::endl;


    sofa::component::misc::CompareStateCreator compareVisitor(sofa::core::ExecParams::defaultInstance());
    compareVisitor.setCreateInMapping(true);
    compareVisitor.setSceneName(refFile);
    compareVisitor.execute(node);

    sofa::component::misc::ReadStateActivator v_read(sofa::core::ExecParams::defaultInstance() /* PARAMS FIRST */, true);
    v_read.execute(node);
}

#ifdef WIN32
int onError( int reportType, char *message, int *returnValue )
{
	std::cout << "ERROR: type='" << reportType << "', message='" << message << "', returnValue='" << *returnValue << "'" << std::endl;
	
	return TRUE;
}
#endif

// ---------------------------------------------------------------------
// ---
// ---------------------------------------------------------------------
int main(int argc, char** argv)
{
    //std::cout << "Using " << sofa::helper::system::atomic<int>::getImplName()<<" atomics." << std::endl;

    sofa::helper::BackTrace::autodump();

    sofa::core::ExecParams::defaultInstance()->setAspectID(0);

#ifdef WIN32
    {
        HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
        COORD s;
        s.X = 160; s.Y = 10000;
        SetConsoleScreenBufferSize(hStdout, s);
        CONSOLE_SCREEN_BUFFER_INFO csbi;
        if (GetConsoleScreenBufferInfo(hStdout, &csbi))
        {
            SMALL_RECT winfo;
            winfo = csbi.srWindow;
            //winfo.Top = 0;
            winfo.Left = 0;
            //winfo.Bottom = csbi.dwSize.Y-1;
            winfo.Right = csbi.dwMaximumWindowSize.X-1;
            SetConsoleWindowInfo(hStdout, TRUE, &winfo);
        }
    }

#ifdef	IS_TEST_FRAMEWORK
	_CrtSetReportHook(&onError);
#endif

#endif

    std::string fileName ;
    bool        startAnim = false;
    bool        printFactory = false;
    bool        loadRecent = false;
    bool        temporaryFile = false;
	bool		testMode = false;
    int	        nbIterations = -1;
    unsigned    computationTimeSampling=0; ///< Frequency of display of the computation time statistics, in number of animation steps. 0 means never.
    std::vector<std::string> guiOptions;
    std::string viewerDimension = "800x600";

    std::string guiName = "SofaGuiQt";
    std::string verif = "";
#ifdef SOFA_SMP
    std::string simulationType = "smp";
#else
    std::string simulationType = "tree";
#endif
    std::vector<std::string> plugins;
    sofa::simulation::SceneLoader::SceneArguments args;
#ifdef SOFA_SMP
    std::string nProcs="";
    bool        disableStealing = false;
    bool        affinity = false;
#endif

    sofa::helper::parse(&args, "This is a SOFA application. Here are the command line arguments")
    // alphabetical order on short name
    .option(&startAnim,'a',"start","start the animation loop")
    .option(&computationTimeSampling,'c',"computationTimeSampling","Frequency of display of the computation time statistics, in number of animation steps. 0 means never.")
    .option(&guiName, 'g', "gui", "Name of the GUI or GUI plugin")
    .option(&guiOptions,'o',"guiOption","Options for the chosen GUI")
    .option(&viewerDimension,'d',"dimension","Set the GUI viewer dimension in WIDTHxHEIGHT format or with any other separator")
    .option(&plugins,'l',"load","load given plugins")
    .option(&nbIterations,'n',"nb_iterations","Number of iterations of the simulation")
    .option(&printFactory,'p',"factory","print factory logs")
    .option(&loadRecent,'r',"recent","load most recently opened file")
    .option(&simulationType,'s',"simu","select the type of simulation (bgl, dag, tree, smp)")
    .option(&temporaryFile,'t',"temporary","the loaded scene won't appear in history of opened files")
	.option(&testMode,'m',"test","select test mode with xml output after N iteration")
    .option(&verif,'v',"verification","load verification data for the scene")
#ifdef SOFA_SMP
    .option(&disableStealing,'w',"disableStealing","Disable Work Stealing")
    .option(&nProcs,'c',"nprocs","Number of processor")
    .option(&affinity,'f',"affinity","Enable aFfinity base Work Stealing")
#endif
    (argc,argv);

#ifdef SOFA_SMP
    int ac = 0;
    char **av = NULL;

    Util::KaapiComponentManager::prop["util.globalid"]="0";
    Util::KaapiComponentManager::prop["sched.strategy"]="I";
    if(!disableStealing)
        Util::KaapiComponentManager::prop["sched.stealing"]="true";
    if(nProcs!="")
        Util::KaapiComponentManager::prop["community.thread.poolsize"]=nProcs;
    if(affinity)
    {
        Util::KaapiComponentManager::prop["sched.stealing"]="true";
        Util::KaapiComponentManager::prop["sched.affinity"]="true";
    }

    a1::Community com = a1::System::join_community( ac, av);
#endif /* SOFA_SMP */

#ifndef SOFA_NO_OPENGL
    if(guiName != "batch") glutInit(&argc,argv);
#endif

#ifdef SOFA_HAVE_DAG
    if (simulationType == "dag")
        sofa::simulation::setSimulation(new sofa::simulation::graph::DAGSimulation());
    else
#endif
#ifdef SOFA_HAVE_BGL
        if (simulationType == "bgl")
            sofa::simulation::setSimulation(new sofa::simulation::bgl::BglSimulation());
        else
#endif
#ifdef SOFA_SMP
            if (simulationType == "smp")
                sofa::simulation::setSimulation(new sofa::simulation::tree::SMPSimulation());
            else
#endif
                sofa::simulation::setSimulation(new sofa::simulation::tree::TreeSimulation());
    if (testMode)
    {
#ifdef	WIN32
        _CrtSetReportHook(&onError);
#endif
    }

    sofa::component::initComponentBase();
    sofa::component::initComponentCommon();
    sofa::component::initComponentGeneral();
    sofa::component::initComponentAdvanced();
    sofa::component::initComponentMisc();

    sofa::simulation::xml::initXml();

    if (!args.empty())
        fileName = args[0];

    for (unsigned int i=0; i<plugins.size(); i++)
        sofa::helper::system::PluginManager::getInstance().loadPlugin(plugins[i]);

    if (!sofa::simulation::gui::initGUI(guiName))
    {
        std::cerr << "ERROR(SofaGUI): GUI \""<<guiName<<"\" initialization failed." << std::endl;
        return EXIT_FAILURE;
    }

    sofa::helper::system::PluginManager::getInstance().init();

    if(nbIterations > 0)
    {
        std::ostringstream oss ;
        oss << "nbIterations=";
        oss << nbIterations;

        guiOptions.push_back(oss.str());
    }

    auto *gui = sofa::simulation::gui::createGUI(guiName, argv[0], guiOptions);
    if (!gui)
    {
        std::cerr << "ERROR(SofaGUI): The GUI was not loaded." << std::endl;
        return EXIT_FAILURE;
    }

    gui->initialize();

    if (fileName.empty())
    {
        if (loadRecent) // try to reload the latest scene
        {
            std::string scenes = "share/config/Sofa.ini";
            scenes = sofa::helper::system::DataRepository.getFile( scenes );
            std::ifstream mrulist(scenes.c_str());
            std::getline(mrulist,fileName);
            mrulist.close();
        }
        else
            fileName = "Demos/caduceus.scn";

        fileName = sofa::helper::system::DataRepository.getFile(fileName);
    }

    //To set a specific resolution for the viewer, you can also use the component ViewerSetting in you scene graph
    if (!viewerDimension.empty())
    {
        std::size_t sep = viewerDimension.find_first_not_of("0123456789");
        if (sep != std::string::npos)
        {
            std::string widthStr(viewerDimension,0,sep);
            int width = atoi(widthStr.c_str());
            std::string heightStr(viewerDimension,sep+1);
            int height = atoi(heightStr.c_str());
            gui->setViewerResolution(width, height);
        }
    }

    sofa::simulation::Node::SPtr groot = sofa::simulation::getSimulation()->load(fileName.c_str(), args);
    if (!groot)
    {
        groot = sofa::simulation::getSimulation()->createNewGraph("");
    }

    if (!verif.empty())
    {
        loadVerificationData(verif, fileName, groot.get());
    }

    if (startAnim)
        groot->setAnimate(true);

    sofa::simulation::getSimulation()->init(groot.get());

    if (startAnim)
        groot->setAnimate(true);

    gui->setScene(groot,fileName.c_str(), temporaryFile);


    //=======================================
    //Apply Options

    if (printFactory)
    {
        std::cout << "////////// FACTORY //////////" << std::endl;
        sofa::helper::printFactoryLog();
        std::cout << "//////// END FACTORY ////////" << std::endl;
    }

    if( computationTimeSampling>0 )
    {
        sofa::helper::AdvancedTimer::setEnabled("Animate", true);
        sofa::helper::AdvancedTimer::setInterval("Animate", computationTimeSampling);
    }

    //=======================================
    // Run the main loop
    if (int err = gui->mainLoop())
        return err;
    groot = sofa::simulation::Node::DynamicCast(gui->getCurrentSimulation() );

    if (groot!=NULL)
        sofa::simulation::getSimulation()->unload(groot);

    sofa::simulation::gui::closeGUI();

    return 0;
}
