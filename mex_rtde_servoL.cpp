/*
 * File : mex_rtde_servoL.cpp
 * Abstract:
 *       An CPP-file S-function for Simulink to control a UR Robot
 * Inputs: 1x6 <Double> TCPPose Target  (X,Y,Z,RX,RY,RZ)
 *         1x1 <Double> Lookahead in Range [0.03, 0.2]
 *         1x1 >Double> Gain - Servo Gain in Range [100, 2000]
 * Outputs: 1x6 <Double> TCPPose Actual  (X,Y,Z,RX,RY,RZ)
 *          1x6 <Double> Q Actual  (Q1, Q2, Q3, Q4, Q5, Q6)
 * Parameters:
 *      <Double> Speed of Robot
 *      <Double> Accel of Robot
 *      <Double> Sample Time of s-Function
 *      string IP Address of Robot
 *
 *      Example Parameter List:     3 5 0.05 '192.168.255.128'
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
#include <ur_rtde/rtde_receive_interface.h>


#define SPEED_IDX  0
#define SPEED_PARAM(S) ssGetSFcnParam(S,SPEED_IDX)

#define ACCEL_IDX   1
#define ACCEL_PARAM(S) ssGetSFcnParam(S,ACCEL_IDX)

#define TIME_IDX   2
#define TIME_PARAM(S) ssGetSFcnParam(S,TIME_IDX)

#define IP_IDX   3
#define IP_PARAM(S) ssGetSFcnParam(S,IP_IDX)

#define NPARAMS   4

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono;

ur_rtde::RTDEReceiveInterface *rtde_receive;
ur_rtde::RTDEControlInterface *rtde_control;
int isInitialised = 0;
std::vector<double> tcpPose;
std::vector<double> q;
auto t_start = high_resolution_clock::now();
auto t_stop = high_resolution_clock::now();
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
      /* Check second parameter: gain to be applied to the sum */
      {
          if (!mxIsDouble(SPEED_PARAM(S)) ||
              mxGetNumberOfElements(SPEED_PARAM(S)) != 1) {
              ssSetErrorStatus(S,"Second parameter to S-function must be a "
                               "scalar \"gain\" value to be applied to "
                               "the sum of the inputs");
              return;
          }
      }
      /* Check second parameter: gain to be applied to the sum */
      {
          if (!mxIsDouble(ACCEL_PARAM(S)) ||
              mxGetNumberOfElements(ACCEL_PARAM(S)) != 1) {
              ssSetErrorStatus(S,"Second parameter to S-function must be a "
                               "scalar \"gain\" value to be applied to "
                               "the sum of the inputs");
              return;
          }
      }    
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
          ssSetErrorStatus(S, "Fourth parameter to S-function must be a "
                           "string specifying IP Address of UR Robot");
          return;
      }


    if (!ssSetNumInputPorts(S, 3)) return;
    ssSetInputPortWidth(S, 0, 6);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortWidth(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    if (!ssSetNumOutputPorts(S,3)) return;
    ssSetOutputPortWidth(S, 0, 6);
    ssSetOutputPortWidth(S, 1, 6);
    ssSetOutputPortWidth(S, 2, 2);

    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
    rtde_receive = NULL;
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
    rtde_receive = NULL;
    rtde_control = NULL;
}


#define MDL_SET_WORK_WIDTHS   /* Change to #undef to remove function */
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
/* Function: mdlSetWorkWidths ===============================================
 * Abstract:
 *      Set up run-time parameters.
 */
static void mdlSetWorkWidths(SimStruct *S)
{
    ssParamRec p;
    int        dlgP = SPEED_IDX;

    p.name             = "Speed";
    p.nDimensions      = 1;
    p.dimensions       = (int_T *) mxGetDimensions(SPEED_PARAM(S));
    p.dataTypeId       = SS_DOUBLE;
    p.complexSignal    = COMPLEX_NO;
    p.data             = (void *)mxGetPr(SPEED_PARAM(S));
    p.dataAttributes   = NULL;
    p.nDlgParamIndices = 1;
    p.dlgParamIndices  = &dlgP;
    p.transformed      = RTPARAM_NOT_TRANSFORMED;
    p.outputAsMatrix   = false;   

    ssParamRec m;
    int        dlgM = ACCEL_IDX;

    m.name             = "Accel";
    m.nDimensions      = 1;
    m.dimensions       = (int_T *) mxGetDimensions(ACCEL_PARAM(S));
    m.dataTypeId       = SS_DOUBLE;
    m.complexSignal    = COMPLEX_NO;
    m.data             = (void *)mxGetPr(ACCEL_PARAM(S));
    m.dataAttributes   = NULL;
    m.nDlgParamIndices = 1;
    m.dlgParamIndices  = &dlgM;
    m.transformed      = RTPARAM_NOT_TRANSFORMED;
    m.outputAsMatrix   = false;       
    
    if (!ssSetNumRunTimeParams(S, 2)) return;

    if (!ssSetRunTimeParamInfo(S, 0, &p)) return;
    if (!ssSetRunTimeParamInfo(S, 1, &m)) return;
}
#endif /* MDL_SET_WORK_WIDTHS */


#define MDL_PROCESS_PARAMETERS   /* Change to #undef to remove function */
#if defined(MDL_PROCESS_PARAMETERS) && defined(MATLAB_MEX_FILE)
/* Function: mdlProcessParameters ===========================================
 * Abstract:
 *      Update run-time parameters.
 */
static void mdlProcessParameters(SimStruct *S)
{
    /* Update Run-Time parameters */
    ssUpdateRunTimeParamData(S, 0, mxGetPr(SPEED_PARAM(S)));
    ssUpdateRunTimeParamData(S, 1, mxGetPr(ACCEL_PARAM(S)));
}
#endif /* MDL_PROCESS_PARAMETERS */




/* Function: mdlOutputs =======================================================
 * Abstract:
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    t_start = high_resolution_clock::now();
    auto duration2 = std::chrono::duration<double>(t_start - t_stop);
    if (isInitialised == 0 || rtde_receive == NULL || rtde_control == NULL)
    {
        mexPrintf("Initialising\nConnecting to: ");
        mexPrintf(ip_addr);

        rtde_receive = new ur_rtde::RTDEReceiveInterface(ip_addr);
        rtde_control = new ur_rtde::RTDEControlInterface(ip_addr);
        
        isInitialised = 1;
    }
    int_T             i;
    InputRealPtrsType targetPoseInPtr = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType LookaheadInPtr = ssGetInputPortRealSignalPtrs(S,1);
    InputRealPtrsType GainInPtr = ssGetInputPortRealSignalPtrs(S,2);
    real_T            *tcpPoseOutPtr    = ssGetOutputPortRealSignal(S,0);
    
    real_T            *qActualOutPtr    = ssGetOutputPortRealSignal(S,1);
    real_T            *timeInfoOutPtr    = ssGetOutputPortRealSignal(S,2);
    real_T speed         = *((real_T *)((ssGetRunTimeParamInfo(S,SPEED_IDX))->data));
    real_T accel         = *((real_T *)((ssGetRunTimeParamInfo(S,ACCEL_IDX))->data));
    real_T blockingTime  = mxGetDoubles(TIME_PARAM(S))[0];

	
    real_T gain = *GainInPtr[0];
    real_T lookahead = *LookaheadInPtr[0];

	tcpPose = rtde_receive->getTargetTCPPose();
    q = rtde_receive->getActualQ();

    for (i=0; i<6; i++) {
        *tcpPoseOutPtr++ = tcpPose[i];
        *qActualOutPtr++ = q[i];
        tcpPose[i] = (*targetPoseInPtr[i]);
    }


    rtde_control->servoL(tcpPose, speed, accel, blockingTime, lookahead, gain);
    t_stop = high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(t_stop - t_start);
    *timeInfoOutPtr++ = gain;
    *timeInfoOutPtr = lookahead;
    
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    delete rtde_receive;
    delete rtde_control;
    isInitialised = 0;
    rtde_receive = NULL;
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
