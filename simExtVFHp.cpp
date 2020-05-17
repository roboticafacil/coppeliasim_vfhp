#include "simExtVFHp.h"
#include "stackArray.h"
#include "stackMap.h"
#include "simLib.h"
#include "vfh/vfh_algorithm.h"
#include <iostream>

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif

#if defined(__linux) || defined(__APPLE__)
    #include <unistd.h>
    #include <string.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

#define PLUGIN_VERSION 5 // 2 since version 3.2.1, 3 since V3.3.1, 4 since V3.4.0, 5 since V3.4.1

static LIBRARY simLib; // the CoppelisSim library that we will dynamically load and bind

static VFH_Algorithm *m_vfh=NULL;

#define PARAM_VFH_SCAN_ANGLE 0
#define PARAM_VFH_CELL_SIZE 1
#define PARAM_VFH_WINDOW_SIZE 2
#define PARAM_VFH_SECTOR_ANGLE 3
#define PARAM_VFH_SAFETY_DIST 4
#define PARAM_VFH_WIDE_SECTOR_WIDTH 5
#define PARAM_VFH_TAU_LOW 6
#define PARAM_VFH_TAU_HIGH 7
#define PARAM_VFH_TARGET_WEIGHT 8
#define PARAM_VFH_PREVIOUS_WEIGHT 9
#define PARAM_VFH_CURRENT_WEIGHT 10

#define PARAM_MOTION_STOP_DISTANCE 0
#define PARAM_MOTION_OFF_CENTERED_DIST 1
#define PARAM_MOTION_WHEEL_RADIUS 2
#define PARAM_MOTION_WHEEL_BASE 3
// --------------------------------------------------------------------------------------
// simExtSkeleton_getData: an example of custom Lua command
// --------------------------------------------------------------------------------------
#define LUA_VFHp_HISTOGRAM_COMMAND "simVFHp.getHistogram" // the name of the new Lua command
#define LUA_VFHp_DIRECTION_COMMAND "simVFHp.getDirection" // the name of the new Lua command
#define LUA_VFHp_MOTION_COMMAND "simVFHp.getMotion" // the name of the new Lua command
#define LUA_VFHp_INIT_COMMAND "simVFHp.init" // the name of the new Lua command
#define LUA_VFHp_RELEASE_COMMAND "simVFHp.release" // the name of the new Lua command

void LUA_VFHp_HISTOGRAM_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    CStackArray* laserScan = new CStackArray();
    CStackArray* sonarScan = new CStackArray();
    //CStackArray* cell_hist = new CStackArray();
    CStackArray* primary_hist = new CStackArray();
    CStackArray* binary_hist = new CStackArray();
    CStackArray* masked_hist = new CStackArray();

    CStackArray* point = new CStackArray();
    CStackArray* arc = new CStackArray();
    size_t numPoints=0, numArcs=0;
    CStackArray inArguments;
    CStackArray outArguments;

    inArguments.buildFromStack(stack);

    if ((inArguments.getSize()==2)&&inArguments.isArray(0)&&inArguments.isArray(1))
    { // we expect 2 arguments: an array with laser points (x1,y1,x2,y2,...), sonar arcs (r1,th1,phi1,r2,th2,phi2,...)
        laserScan=inArguments.getArray(0);
        sonarScan=inArguments.getArray(1);
            numPoints=size_t(laserScan->getSize());
            std::vector<Point_t > laser_ranges;
            laser_ranges.clear();
            Point_t p;
            for (size_t i=0;i<numPoints;i++)
            {
                point=laserScan->getArray(i);
                p.x=point->getDouble(0);
                p.y=point->getDouble(1);
                laser_ranges.push_back(p);
            }
            std::vector<Arc_t > sonar_ranges;
            sonar_ranges.clear();
            Arc_t a;
            for (size_t i=0;i<numArcs;i++)
            {
                arc=sonarScan->getArray(i);
                a.r=arc->getDouble(0);
                a.th=arc->getDouble(1);
                a.beam_angle=arc->getDouble(2);
                sonar_ranges.push_back(a);
            }
            m_vfh->ComputeHistogram(laser_ranges,sonar_ranges);
            for (unsigned int i=0;i<m_vfh->primary_hist.size();i++)
                primary_hist->pushDouble(m_vfh->primary_hist[i]);
            outArguments.pushArray(primary_hist);
            for (unsigned int i=0;i<m_vfh->binary_hist.size();i++)
                binary_hist->pushBool(m_vfh->binary_hist[i]);
            outArguments.pushArray(binary_hist);
            for (unsigned int i=0;i<m_vfh->masked_hist.size();i++)
                masked_hist->pushBool(m_vfh->masked_hist[i]);
            outArguments.pushArray(masked_hist);
    }
    else
        simSetLastError(LUA_VFHp_HISTOGRAM_COMMAND,"Not enough arguments or wrong arguments.");
    outArguments.buildOntoStack(stack);
}

void LUA_VFHp_DIRECTION_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    CStackArray* masked_hist = new CStackArray();
    CStackArray* robotPose = new CStackArray();
    CStackArray* goalPostition = new CStackArray();
    CStackArray* candidates = new CStackArray();

    Pose_t robot;
    Point_t goal;
    size_t numSectors=0;
    CStackArray inArguments;
    CStackArray outArguments;

    inArguments.buildFromStack(stack);

    if ((inArguments.getSize()==3)&&inArguments.isArray(0)&&inArguments.isArray(1)&&inArguments.isArray(2))
    { // we expect 3 arguments: an array with masked histogram, robot pose (x,y,th) and goal position (x,y).

        masked_hist=inArguments.getArray(0);
        robotPose=inArguments.getArray(1);
        goalPostition=inArguments.getArray(2);
        if ((robotPose->getSize()==3)&&(goalPostition->getSize()>=2))
        {
            robot.x=robotPose->getDouble(0);
            robot.y=robotPose->getDouble(1);
            robot.th=robotPose->getDouble(2);
            goal.x=goalPostition->getDouble(0);
            goal.y=goalPostition->getDouble(1);
            numSectors=size_t(masked_hist->getSize());
            std::vector<bool> masked(numSectors);
            for (size_t i=0;i<numSectors;i++)
                masked[i]=masked_hist->getBool(i);
            m_vfh->ComputeDirection(masked,robot,goal);

            outArguments.pushDouble(m_vfh->getSelectedDirection());
            outArguments.pushDouble(m_vfh->getTargetDirection());
            for (unsigned int i=0;i<m_vfh->candidates.size();i++)
                candidates->pushDouble(m_vfh->candidates[i]);
            outArguments.pushArray(candidates);
        }
        else
            simSetLastError(LUA_VFHp_DIRECTION_COMMAND,"Wrong arguments.");
    }
    else
        simSetLastError(LUA_VFHp_DIRECTION_COMMAND,"Not enough arguments.");
    outArguments.buildOntoStack(stack);
}

void LUA_VFHp_MOTION_CALLBACK(SScriptCallBack* p)
{
    int stack=p->stackID;
    double velocity=0.0,direction=0.0;
    CStackArray* robotPose = new CStackArray();
    CStackArray* goalPosition = new CStackArray();
    CStackArray inArguments;
    CStackArray outArguments;
    Pose_t robot;
    Point_t goal;
    NHMotion_t motion;
    motion.wL=0.0;
    motion.wR=0.0;

    inArguments.buildFromStack(stack);


    if ((inArguments.getSize()==4)&&inArguments.isNumber(0)&&inArguments.isNumber(1)&&inArguments.isArray(2)&&inArguments.isArray(3))
    { // we expect 4 arguments: a reference velocity, a direction, robot pose (x,y,th) and goal position (x,y).
        velocity=inArguments.getDouble(0);
        direction=inArguments.getDouble(1);
        robotPose=inArguments.getArray(2);
        goalPosition=inArguments.getArray(3);
        if ((robotPose->getSize()==3)&&(goalPosition->getSize()>=2))
        {
            robot.x=robotPose->getDouble(0);
            robot.y=robotPose->getDouble(1);
            robot.th=robotPose->getDouble(2);
            goal.x=goalPosition->getDouble(0);
            goal.y=goalPosition->getDouble(1);
            motion=m_vfh->ComputeMotion(velocity,direction,robot,goal);
        }
        else
            simSetLastError(LUA_VFHp_MOTION_COMMAND,"Wrong arguments.");

    }
    else
        simSetLastError(LUA_VFHp_MOTION_COMMAND,"Not enough arguments or wrong arguments.");

    outArguments.pushDouble(motion.wL);
    outArguments.pushDouble(motion.wR);
    outArguments.buildOntoStack(stack);
}
// --------------------------------------------------------------------------------------


void LUA_VFHp_INIT_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;

    VFHParams_t vfhParams;
    MotionParams_t motionParams;
    vfhParams.scan_angle=120.0*M_PI/180.0;
    vfhParams.cell_size=0.1;
    vfhParams.windows_size=8;
    vfhParams.sector_angle=5*M_PI/180.0;
    vfhParams.safety_dist=0.55;
    vfhParams.wide_valley_width=5;
    vfhParams.tau_low=500.0;
    vfhParams.tau_high=2000.0;
    vfhParams.target_weight=3.0;
    vfhParams.previous_weight=2.0;
    vfhParams.current_weight=1.0;

    motionParams.stop_dist=0.2;
    motionParams.off_center_dist=0.3;
    motionParams.wheel_radius=0.0975;
    motionParams.wheel_base=0.1655;

    CStackArray* vfh_params = new CStackArray();
    CStackArray* motion_params = new CStackArray();
    int status=-1;
    CStackArray inArguments;
    inArguments.buildFromStack(stack);

    if ( (inArguments.getSize()==2)&&inArguments.isArray(0)&&inArguments.isArray(1))
    {
        vfh_params=inArguments.getArray(0);
        motion_params=inArguments.getArray(1);

        if (vfh_params->getSize()>PARAM_VFH_SCAN_ANGLE)
            vfhParams.scan_angle=vfh_params->getDouble(PARAM_VFH_SCAN_ANGLE);
        if (vfh_params->getSize()>PARAM_VFH_CELL_SIZE)
            vfhParams.cell_size=vfh_params->getDouble(PARAM_VFH_CELL_SIZE);
        if (vfh_params->getSize()>PARAM_VFH_WINDOW_SIZE)
            vfhParams.windows_size=vfh_params->getDouble(PARAM_VFH_WINDOW_SIZE);
        if (vfh_params->getSize()>PARAM_VFH_SECTOR_ANGLE)
            vfhParams.sector_angle=vfh_params->getDouble(PARAM_VFH_SECTOR_ANGLE);
        if (vfh_params->getSize()>PARAM_VFH_SAFETY_DIST)
            vfhParams.safety_dist=vfh_params->getDouble(PARAM_VFH_SAFETY_DIST);
        if (vfh_params->getSize()>PARAM_VFH_WIDE_SECTOR_WIDTH)
            vfhParams.wide_valley_width=(unsigned int)(vfh_params->getInt(PARAM_VFH_WIDE_SECTOR_WIDTH));
        if (vfh_params->getSize()>PARAM_VFH_TAU_LOW)
            vfhParams.tau_low=vfh_params->getDouble(PARAM_VFH_TAU_LOW);
        if (vfh_params->getSize()>PARAM_VFH_TAU_HIGH)
            vfhParams.tau_high=vfh_params->getDouble(PARAM_VFH_TAU_HIGH);
        if (vfh_params->getSize()>PARAM_VFH_TARGET_WEIGHT)
            vfhParams.target_weight=vfh_params->getDouble(PARAM_VFH_TARGET_WEIGHT);
        if (vfh_params->getSize()>PARAM_VFH_PREVIOUS_WEIGHT)
            vfhParams.previous_weight=vfh_params->getDouble(PARAM_VFH_PREVIOUS_WEIGHT);
        if (vfh_params->getSize()>PARAM_VFH_CURRENT_WEIGHT)
            vfhParams.current_weight=vfh_params->getDouble(PARAM_VFH_CURRENT_WEIGHT);

        if (motion_params->getSize()>PARAM_MOTION_STOP_DISTANCE)
            motionParams.stop_dist=motion_params->getDouble(PARAM_MOTION_STOP_DISTANCE);
        if (motion_params->getSize()>PARAM_MOTION_OFF_CENTERED_DIST)
            motionParams.off_center_dist=motion_params->getDouble(PARAM_MOTION_OFF_CENTERED_DIST);
        if (motion_params->getSize()>PARAM_MOTION_WHEEL_RADIUS)
            motionParams.wheel_radius=motion_params->getDouble(PARAM_MOTION_WHEEL_RADIUS);
        if (motion_params->getSize()>PARAM_MOTION_WHEEL_BASE)
            motionParams.wheel_base=motion_params->getDouble(PARAM_MOTION_WHEEL_BASE);

        m_vfh = new VFH_Algorithm(vfhParams,motionParams);
        m_vfh->Init();
        status=0;
    }
    else
        simSetLastError(LUA_VFHp_INIT_COMMAND,"Not enough arguments or wrong arguments.");
    // Now return a string and a map:
    CStackArray outArguments;
    outArguments.pushInt(status);
    outArguments.buildOntoStack(stack);
}

void LUA_VFHp_RELEASE_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    CStackArray inArguments;
    inArguments.buildFromStack(stack);

    if ( (inArguments.getSize()==0))
    { // we expect 1 argument: an array with laser points
        if (m_vfh!=NULL)
        {
            delete m_vfh;
            m_vfh=NULL;
        }
    }
    else
        simSetLastError(LUA_VFHp_RELEASE_COMMAND,"Not enough arguments or wrong arguments.");

    // Now return a string and a map:
    CStackArray outArguments;
    outArguments.buildOntoStack(stack);
}

// This is the plugin start routine (called just once, just after the plugin was loaded):
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
    // Dynamically load and bind CoppelisSim functions:
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#else
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the CoppelisSim library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp+="\\coppeliaSim.dll";
#elif defined (__linux)
    temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
    temp+="/libcoppeliaSim.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the CoppelisSim library:
    simLib=loadSimLibrary(temp.c_str());
    if (simLib==NULL)
    {
        std::cout << "Error, could not find or correctly load the CoppelisSim library. Cannot start 'PluginSkeleton' plugin.\n";
        return(0); // Means error, CoppelisSim will unload this plugin
    }
    if (getSimProcAddresses(simLib)==0)
    {
        std::cout << "Error, could not find all required functions in the CoppelisSim library. Cannot start 'PluginSkeleton' plugin.\n";
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppelisSim will unload this plugin
    }

    // Check the version of CoppelisSim:
    int simVer,simRev;
    simGetIntegerParameter(sim_intparam_program_version,&simVer);
    simGetIntegerParameter(sim_intparam_program_revision,&simRev);
    if( (simVer<30400) || ((simVer==30400)&&(simRev<9)) )
    {
        std::cout << "Sorry, your CoppelisSim copy is somewhat old, CoppelisSim 3.4.0 rev9 or higher is required. Cannot start 'PluginSkeleton' plugin.\n";
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppelisSim will unload this plugin
    }

    // Implicitely include the script lua/simExtPluginSkeleton.lua:
    simRegisterScriptVariable("simVFHp","require('simExtVFHp')",0);

    // Register the new function:
#ifdef VFHp_DEBUG
    std::cout << "Registering VFHp functions" << std::endl;
#endif
    simRegisterScriptCallbackFunction(strConCat(LUA_VFHp_MOTION_COMMAND,"@","VFHpComputeMotion"),strConCat("number wL,number wR=",LUA_VFHp_MOTION_COMMAND,"(number velocity, number direction, table pose, table goal)"),LUA_VFHp_MOTION_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_VFHp_DIRECTION_COMMAND,"@","VFHpComputeDirection"),strConCat("number selected, number target, table candidates=",LUA_VFHp_DIRECTION_COMMAND,"(table maskedHistogram, table pose, table goal)"),LUA_VFHp_DIRECTION_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_VFHp_HISTOGRAM_COMMAND,"@","VFHpComputeHistogram"),strConCat("table primary, table binary, table masked=",LUA_VFHp_HISTOGRAM_COMMAND,"(table laserScan, table sonarData)"),LUA_VFHp_HISTOGRAM_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_VFHp_INIT_COMMAND,"@","VFHpInit"),strConCat("number status=",LUA_VFHp_INIT_COMMAND,"(table vfhParams, table motionParams)"),LUA_VFHp_INIT_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(LUA_VFHp_RELEASE_COMMAND,"@","VFHpRelease"),strConCat("",LUA_VFHp_RELEASE_COMMAND,"()"),LUA_VFHp_RELEASE_CALLBACK);
    return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when CoppelisSim is ending, i.e. releasing this plugin):
SIM_DLLEXPORT void simEnd()
{
    // Here you could handle various clean-up tasks

    unloadSimLibrary(simLib); // release the library
}

// This is the plugin messaging routine (i.e. CoppelisSim calls this function very often, with various messages):
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag=true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    void* retVal=NULL;

    // Here we can intercept many messages from CoppelisSim (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the CoppelisSim user manual.

    if (message==sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag=true; // CoppelisSim dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message==sim_message_eventcallback_menuitemselected)
    { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message==sim_message_eventcallback_instancepass)
    {   // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in CoppelisSim. This message is the most convenient way to do so:

        int flags=auxiliaryData[0];
        bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message 
        bool instanceSwitched=((flags&64)!=0);

        if (instanceSwitched)
        {
            // React to an instance switch here!!
        }

        if (sceneContentChanged)
        { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message==sim_message_eventcallback_mainscriptabouttobecalled)
    { // The main script is about to be run (only called while a simulation is running (and not paused!))
        
    }

    if (message==sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start

    }

    if (message==sim_message_eventcallback_simulationended)
    { // Simulation just ended

    }

    if (message==sim_message_eventcallback_moduleopen)
    { // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)(customData))==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message==sim_message_eventcallback_modulehandle)
    { // A script called simHandleModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
    }

    if (message==sim_message_eventcallback_moduleclose)
    { // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message==sim_message_eventcallback_instanceswitch)
    { // We switched to a different scene. Such a switch can only happen while simulation is not running

    }

    if (message==sim_message_eventcallback_broadcast)
    { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message==sim_message_eventcallback_scenesave)
    { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
    { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag=false;
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}

