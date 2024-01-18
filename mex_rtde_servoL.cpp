/*
 * File : mex_rtde_servoL.cpp
 * Abstract:
 *       An CPP-file S-function for Simulink to control a UR Robot
 * Inputs: 1x6 <Double> TCPPose Target  (X,Y,Z,RX,RY,RZ)
 *         1x1 <Double> Lookahead in Range [0.03, 0.2]
 *         1x1 >Double> Gain - Servo Gain in Range [100, 2000]
 * Outputs: 1x1 <Double> Status of Connection to Robot 
 *
 * Parameters:
 *      <Double> Sample Time of s-Function
 *      string IP Address of Robot
 *
 *      Example Parameter List:     0.008 '192.168.255.128'
 *
 * Simulink Coder note:
 *   This file needs to be compiled against the ur-rtde and boost libraries
 *   Link for ur-rtde here:
 *      https://gitlab.com/sdurobotics/ur_rtde
 *   Link for boost here:
 *      https://www.boost.org/
 *
 *   Tested are the following versions:
 *      Microsoft Visual C++ 2019
 *      Matlab 2023b
 *      URSim Virtual 5.14.6
 *      Boost Version 1.84.0
 *      ur_rtde Branch Version: bbbd7a42
 *            IMPORTANT!!!!  In order to maintain Real-time performance, 
                             the branch needs to be modified in accordance
                             with Issue 151. 
                             https://gitlab.com/sdurobotics/ur_rtde/-/issues/151
                             Although the solution is in a submitted merge request, 
                             it is not solved in the current Master.
                             Solution: Comment out break at line 518 in rtde.cpp
                        }
                        robot_state->unlockUpdateStateMutex();

                        //break;
                    }                             
 * My Compiler function:
 *     mex -r2018a mex_rtde_servoL.cpp ...
            -IC:\Users\Gavin\source\repos\ur_rtde\include ...
            -IC:\dev\boost_1_84_0 ...
            -LC:\Users\Gavin\source\repos\ur_rtde-build\Release ...
            -lrtde.lib ...
            -LC:\dev\boost_1_84_0\bin.v2\libs\thread\build\msvc-14.3\release\link-static\threadapi-win32\threading-multi ...
            -LC:\dev\boost_1_84_0\bin.v2\libs\system\build\msvc-14.3\release\link-static\threading-multi ...
            -llibboost_system-vc143-mt-x64-1_84.lib ...
            -llibboost_thread-vc143-mt-x64-1_84.lib 


 * Copyright Gavin Kane 2023
 */


#define S_FUNCTION_NAME  mex_rtde_servoL
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <ur_rtde/rtde_control_interface.h>

#define TIME_IDX   0
#define TIME_PARAM(S) ssGetSFcnParam(S,TIME_IDX)

#define IP_IDX   1
#define IP_PARAM(S) ssGetSFcnParam(S,IP_IDX)

#define NPARAMS   2

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#include <thread>

using namespace ur_rtde;

ur_rtde::RTDEControlInterface *rtde_control;
int isInitialised = 0;
std::vector<double> tcpPose(6,0.0);

char_T ip_addr[sizeof("255.255.255.255")];

/*================*
 * Build checking *
 *================*/



/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    ssSetNumSFcnParams(S, NPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    ssSetSFcnParamTunable(S,TIME_IDX,false);  
    ssSetSFcnParamTunable(S,IP_IDX,false);  
    
    if (!mxIsChar(IP_PARAM(S)) ||
        mxGetNumberOfElements(IP_PARAM(S)) >= sizeof(ip_addr) ||
        mxGetString(IP_PARAM(S),ip_addr,sizeof(ip_addr)) != 0) {
          ssSetErrorStatus(S, "Fourth parameter to S-function must be a "
                           "string specifying IP Address of UR Robot");
          return;
      }


    if (!ssSetNumInputPorts(S, 5)) return;
    ssSetInputPortWidth(S, 0, 6);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortWidth(S, 2, 1);
    ssSetInputPortWidth(S, 3, 1);
    ssSetInputPortWidth(S, 4, 1);

    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);

    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
    rtde_control = NULL;
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct* S)
{
    ssSetSampleTime(S, 0, mxGetDoubles(TIME_PARAM(S))[0]);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
    rtde_control = NULL;
}





/* Function: mdlOutputs =======================================================
 * Abstract:
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    if (isInitialised == 0 || rtde_control == NULL)
    {
        mexPrintf("Connecting Controller to: ");
        mexPrintf(ip_addr);
        mexPrintf("\n");
        try
        {
            rtde_control = new ur_rtde::RTDEControlInterface(ip_addr);
            isInitialised = 1;
        }
        catch (const std::exception& e)
        {
            mexPrintf(e.what());
            mexPrintf("\n");
            isInitialised = 0;
        }
    }
    if (isInitialised == 1 && rtde_control != NULL)
    {


    
    try
    {
        int_T             i;
        InputRealPtrsType targetPoseInPtr = ssGetInputPortRealSignalPtrs(S,0);
        InputRealPtrsType LookaheadInPtr  = ssGetInputPortRealSignalPtrs(S,1);
        InputRealPtrsType GainInPtr       = ssGetInputPortRealSignalPtrs(S,2);
        InputRealPtrsType SpeedInPtr      = ssGetInputPortRealSignalPtrs(S,3);
        InputRealPtrsType AccelInPtr      = ssGetInputPortRealSignalPtrs(S,4);
 
        for (i=0; i<tcpPose.size(); i++) {
             tcpPose[i] = (*targetPoseInPtr[i]);
        }        
        real_T blockingTime  = 0.5*mxGetDoubles(TIME_PARAM(S))[0];
        real_T gain = *GainInPtr[0];
        real_T lookahead = *LookaheadInPtr[0];
        real_T speed = *SpeedInPtr[0];
        real_T accel = *AccelInPtr[0];        
        rtde_control->servoL(tcpPose, speed, accel, blockingTime, lookahead, gain);

        
        
    }
    catch (const std::exception& e)
    {
        
         mexPrintf(e.what());
         mexPrintf("\n");
         isInitialised = 0;
    }    
    }    
    real_T     *connectStateOutPtr    = ssGetOutputPortRealSignal(S,0);
    *connectStateOutPtr = isInitialised;
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    try
    {
        if (isInitialised == 1 && rtde_control != NULL)
        {
            rtde_control->servoStop(10.0);
            delete rtde_control;
        }
    }
    catch (const std::exception& e)
    {
        mexPrintf(e.what());
        mexPrintf("\n");
        isInitialised = 0;
    }
    isInitialised = 0;
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
