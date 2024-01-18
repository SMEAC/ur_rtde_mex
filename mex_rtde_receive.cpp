/*
 * File : mex_rtde_receive.cpp
 * Abstract:
 *       An CPP-file S-function for Simulink to get the current pose of a UR Robot
 * Inputs: -
 * Outputs: 1x6 <Double> TCPPose Actual  (X,Y,Z,RX,RY,RZ)
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
 *     mex -r2018a mex_rtde_receive.cpp ...
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


#define S_FUNCTION_NAME  mex_rtde_receive
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <ur_rtde/rtde_receive_interface.h>


#define TIME_IDX   0
#define TIME_PARAM(S) ssGetSFcnParam(S,TIME_IDX)

#define IP_IDX   1
#define IP_PARAM(S) ssGetSFcnParam(S,IP_IDX)

#define NPARAMS   2

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#include <thread>

using namespace ur_rtde;


ur_rtde::RTDEReceiveInterface *rtde_receive;
int isInitialised = 0;
std::vector<double> tcpPose(6,0.0);

// Parameters
char_T ip_addr[sizeof("255.255.255.255")];

/*================*
 * Build checking *
 *================*/

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS)  && defined(MATLAB_MEX_FILE)
/*
 * Check to make sure that each parameter is 1-d and positive
 */
static void mdlCheckParameters(SimStruct *S)
{

}
#endif

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
          ssSetErrorStatus(S, "Second parameter to S-function must be a "
                           "string specifying IP Address of UR Robot");
          return;
      }


    if (!ssSetNumInputPorts(S, 0)) return;


    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, 6);

    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
    rtde_receive = NULL;
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
    rtde_receive = NULL;
}




/* Function: mdlOutputs =======================================================
 * Abstract:
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{

    if (isInitialised == 0 || rtde_receive == NULL)
    {
        mexPrintf("Connecting Receiver to: ");
        mexPrintf(ip_addr);
        mexPrintf("\n");
        try
        {
            rtde_receive = new ur_rtde::RTDEReceiveInterface(ip_addr);
            isInitialised = 1;
        }
        catch (const std::exception& e)
        {
            mexPrintf(e.what());
            mexPrintf("\n");
            isInitialised = 0;
        }
    }
    if (isInitialised == 1 && rtde_receive != NULL)
    {
        try
        {
            int_T             i;
            real_T            *tcpPoseOutPtr    = ssGetOutputPortRealSignal(S,0);
    	    tcpPose = rtde_receive->getTargetTCPPose();
        
            for (i=0; i<tcpPose.size(); i++) {
                *tcpPoseOutPtr++ = tcpPose[i];
            }
        }
        catch (const std::exception& e)
        {
            mexPrintf(e.what());
            mexPrintf("\n");
            isInitialised = 0;
        }
    }
   
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    try
    {
        //delete rtde_receive;
        isInitialised = 0;
        rtde_receive = NULL;
    }
    catch (...)
    {}
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
