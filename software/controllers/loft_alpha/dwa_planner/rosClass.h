//
// created by zumo arthicha.
// date: 22 Aug 2022
// planner-ros interface
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
#include "nav_2d_msgs/Polygon2DStamped.h"
#include "nav_2d_msgs/Polygon2D.h"
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
	float getLidarAngle(int idx);
	float getLidarDistance(int idx);
	int getLidarSize();
	float getRobotPose(int idx);
	float getGoal(int idx);

	// publish methods
	void publishVelocityCommand(float v, float w);
	void publishIteration();
	void publishExpectation(float x1, float y1, float yaw1,float x2, float y2, float yaw2);

	// ros handle method
	bool rosStep();

	// update ros params
	void getRosParam();


	// ros public params
	float max_speed = 0.5;
	float speed_resolution = 0.05;
	float yaw_resolution = 0.5;
	float predict_time = 3.0;
	float dt = 0.1;
	float clearance_cost = 1.0;
	float heading_cost = 10.0;
	float velocity_cost = 0.0;
	float detectionlength_gain = 1.0;
	float detectionangle_gain = 1.0;
	float detectionangle_offset = 0.0;
	float sensor_limit = 2.0; // meter
	int coarse_step = 2;

	// public ros data
	vector<float> detectionsx;
	vector<float> detectionsy;
	

private:

	// internal private variable
	bool terminate_status;
	float rosupdaterate = 1.0;

	// private data
	vector<float> lidarAngle;
	vector<float> lidarDistance;
	vector<float> robotPose;
	vector<float> goal;

	// ros callback methods
	void terminateCB(const std_msgs::Bool& terminate_topic);
	void lidarAngleCB(const std_msgs::Float32MultiArray::ConstPtr& array);
	void lidarDistanceCB(const std_msgs::Float32MultiArray::ConstPtr& array);
	void robotPoseCB(const std_msgs::Float32MultiArray::ConstPtr& array);
	void goalCB(const std_msgs::Float32MultiArray::ConstPtr& array);
	void hdetectionsCB(const nav_2d_msgs::Polygon2DStamped::ConstPtr& msgs);

	std_msgs::Float32MultiArray velocityCommand;
	std_msgs::Float32MultiArray realDetection;
	std_msgs::Float32MultiArray expectations;
	std_msgs::Int32 iteration;

	// ros parameter
	ros::Rate* rosrate;
	ros::NodeHandle* node;

	// ros publisher
	ros::Publisher velocityCommandPub;
	ros::Publisher iterationPub;
	ros::Publisher realDetectionPub;
	ros::Publisher expectationPub;
	
	// ros subscriber
	ros::Subscriber terminatorSub;
	ros::Subscriber lidarAngleSub;
	ros::Subscriber lidarDistanceSub;
	ros::Subscriber robotPoseSub;
	ros::Subscriber goalSub;
	ros::Subscriber hdetectionsSub;

};

#endif //ROSCLASS_H
