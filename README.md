[![View Simulink Mex File Integration of UR Control with RTDE on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://de.mathworks.com/matlabcentral/fileexchange/157111-simulink-mex-file-integration-of-ur-control-with-rtde)


# ur_rtde_mex
Simulink mex project for connecting a universal robot to matlab/simulink



 File : mex_rtde_moveL.cpp
 
 Abstract:
       An CPP-file S-function for Simulink to control a UR Robot
       
 Inputs: 
 * 1x6 <Double> TCPPose Target  (X,Y,Z,RX,RY,RZ)
 
 Outputs: 
 * 1x6 <Double> TCPPose Actual  (X,Y,Z,RX,RY,RZ)
 * 1x6 <Double> Q Actual  (Q1, Q2, Q3, Q4, Q5, Q6)
          
 Parameters:
 * <Double> Speed of Robot
 * <Double> Accel of Robot
 * <Double> Sample Time of s-Function
 * string IP Address of Robot
   
 Example Parameter List:     3 5 0.05 '192.168.255.128'

 Notes for compilation:
 
   This file needs to be compiled against the ur-rtde and boost libraries
   
   Link for ur-rtde here: https://gitlab.com/sdurobotics/ur_rtde
   Link for boost here:  https://www.boost.org/

   Tested are the following versions:
   * Microsoft Visual C++ 2019
   * Matlab 2023b
   * URSim Virtual 5.14.6
   * Boost Version 1.84.0
   * ur_rtde Branch Version: bbbd7a42
    IMPORTANT!!!!  In order to maintain Real-time performance, the branch needs to be modified in accordance with Issue 151. 
                             https://gitlab.com/sdurobotics/ur_rtde/-/issues/151
    Although the solution is in a submitted merge request, it is not solved in the current Master. Solution: Comment out break at line 518 in rtde.cpp

                        }
                        robot_state->unlockUpdateStateMutex();

                        //break;
                    }                             

 My Compiler function:
 
     mex -r2018a mex_rtde_moveL.cpp ...
            -IC:\Users\Gavin\source\repos\ur_rtde\include ...
            -IC:\dev\boost_1_84_0 ...
            -LC:\Users\Gavin\source\repos\ur_rtde-build\Release ...
            -lrtde.lib ...
            -LC:\dev\boost_1_84_0\bin.v2\libs\thread\build\msvc-14.3\release\link-static\threadapi-win32\threading-multi ...
            -LC:\dev\boost_1_84_0\bin.v2\libs\system\build\msvc-14.3\release\link-static\threading-multi ...
            -llibboost_system-vc143-mt-x64-1_84.lib ...
            -llibboost_thread-vc143-mt-x64-1_84.lib 


 Copyright Gavin Kane 2023

