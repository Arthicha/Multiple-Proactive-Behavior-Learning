/**********************************************************************
 *                                                                    *
 *                     robot control (main)                           *
 *                                                                    *
 *  create by: arthicha srisuchinnawong (arsri21@student.sdu.dk)      *      
 *  date: 21 Aug 2022                                                 * 
 *                                                                    *
 *  description: the program/ros-node combined the planning velocity  *
 *               command and neural control velocity command.         *
 *                                                                    *                                                         
 *********************************************************************/

/* --------------------------------------------------------------------
                          include libraries
-------------------------------------------------------------------- */

// additional standard library
#include <random>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>

// ros interface
#include "rosClass.h"


// define config parameters
#define MAXV 1.0 // maximum linear velocity
#define MAXW 0.6981 // maximum angular velocity

/* --------------------------------------------------------------------
                           auxillary functions
-------------------------------------------------------------------- */

float sat(float x,float sat)
{
  if (x < -sat) return -sat;
  if (x > sat) return sat;
  return x;
}

/* --------------------------------------------------------------------
                               global variables
-------------------------------------------------------------------- */

// operation state
bool running = true; // node state

/* --------------------------------------------------------------------
                               main loop
-------------------------------------------------------------------- */

int main(int argc, char * argv[]) {

  rosClass ros(argc,argv); // initialize ros class
  ros.getRosParam();

  while(running)
  {
    // planner twist command
    float vp = ros.getPlanningCommand(0);
    float wp = ros.getPlanningCommand(1);

    // neural twist command
    float vn = ros.getNeuralCommand(0);
    float wn = ros.getNeuralCommand(1);
    
    // combined command
    float v = sat(vp + vn,ros.max_speed);
    float w = sat(wp + wn,MAXW);
    if (abs(v) < 0.01) w = 0;

    // kinematic convertsion for simulation (v,w) -> (v1,v2,v3,alpha)
    float w_ = w*ros.robot_radius;
    float v1 = sat((1/ros.wheel_radius)*(v+w_),10);
    float v2 = sat((1/ros.wheel_radius)*(v-w_),10);
    float v3 = sat((1/ros.wheel_radius)*(v/(abs(v)+1e-10))*sqrtf((v*v)+(w_*w_)),10);
    float alpha = atan2(-w_,abs(v));
    if (v < 0){alpha = -alpha;}
    alpha = sat(alpha,1.570796);

    // publish 
    ros.publishWheelCommand(-v1,-v2,v3,alpha);
    ros.publishTwistCommand(v,w,ros.v_gain,ros.w_gain);
    running = ros.rosStep();
  }
  return 0;
}

