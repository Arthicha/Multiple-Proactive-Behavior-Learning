//
// created by zumo arthicha.
// date: 21 Aug 2022
// robot control-ros interface
//

#ifndef ROSCLASS_H
#define ROSCLASS_H

/* -------------------------------------------------------------------------------
								include libraries
 ------------------------------------------------------------------------------- */

// standard libraries
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fstream>
#include <math.h>
#include <string>
#include <map>
#include <vector>

// ros libraries
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>


// define name space
using namespace std;

/* -------------------------------------------------------------------------------
								ros class
 ------------------------------------------------------------------------------- */

class rosClass{
public:

	rosClass(int argc, char *argv[]); // constructor
	~rosClass(); // destuctor

	// get command 
	float getPlanningCommand(int idx);
	float getNeuralCommand(int idx);

	// publish methods
	void publishWheelCommand(float v1, float v2, float v3, float alpha); // simulation velocity command
	void publishTwistCommand(float v, float w, float v_gain, float w_gain); // twist command for interface

	// ros handle method
	bool rosStep();

	// update ros params
	void getRosParam();

	// ros public params
	float wheel_radius = 0.15; // wheel radius <meter>
	float robot_radius = 0.20; // robot radius <meter> - approximate robot as a circle
	float v_gain = 1.0; // linear velocity gain
	float w_gain = 1.0; // angular velocity gain
	float max_speed = 0.5; // meter/sec
	

private:

	// internal private variable
	bool terminate_status;
	float rosupdaterate = 1.0;

	// private data
	vector<float> planningCommand;
	vector<float> neuralCommand;

	// ros callback methods
	void terminateCB(const std_msgs::Bool& terminate_topic);
	void planningCB(const std_msgs::Float32MultiArray::ConstPtr& array); // get planning command
	void neuralControlCB(const std_msgs::Float32MultiArray::ConstPtr& array); // get neural control command

	

	// ros parameter
	ros::Rate* rosrate;
	ros::NodeHandle* node;

	// ros messages
	std_msgs::Float32MultiArray velocityCommand; // velocity command
	std_msgs::Float32MultiArray benchmarkCommand; // benchmark
	geometry_msgs::Twist twistCommand; // twist command

	// ros publisher
	ros::Publisher velocityCommandPub;
	ros::Publisher benchmarkCommandPub;
	ros::Publisher twistCommandPub;
	
	// ros subscriber
	ros::Subscriber terminatorSub;
	ros::Subscriber planningSub;
	ros::Subscriber neuralControlSub;

};

#endif //ROSCLASS_H
