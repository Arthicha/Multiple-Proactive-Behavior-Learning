//
// created by zumo arthicha.
// date: 22 Aug 2022
// neural control-ros interface
//

#ifndef ROSCLASS_H
#define ROSCLASS_H

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

class rosClass{
public:
    rosClass(int argc, char *argv[]); // constructor
    ~rosClass(); // destuctor

    // get command 
    float getLidarAngle(int idx);
    float getLidarDistance(int idx);
    int getLidarSize();
    float getPlanningCommand(int idx);
    float getRobotPose(int idx);
    float getGoal(int idx);
    float getHit();
    int getLearningFlag();

    // publish methods
    void publishVelocityCommand(float v, float w);
    void publishSumWeight(vector<float> ws);

    // ros handle method
    bool rosStep();

    // update ros params
    void getRosParam();

    // ros public params
    float eta = 0.0;
    int n_sensor = 14; 
    float sensor_limit = 2.0; // meter
    float cover_range = 90; // degree
    float max_speed = 0.5; // meter/sec
    float robot_radius = 0.2; // meter
    int sleept = 500; // iteration
    float learned_weight = 0.0; // learned weight gain
    float detectionlength_gain = 1.0;
    float detectionangle_gain = 1.0;
    float detectionangle_offset = 0.0;

    // public data
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
    vector<float> planningCommand;
    vector<float> goal;
    float hit;
    int learningFlag = 0;

    // ros callback methods
	void terminateCB(const std_msgs::Bool& terminate_topic);
	void lidarAngleCB(const std_msgs::Float32MultiArray::ConstPtr& array);
    void lidarDistanceCB(const std_msgs::Float32MultiArray::ConstPtr& array);
    void robotPoseCB(const std_msgs::Float32MultiArray::ConstPtr& array);
    void planningCommandCB(const std_msgs::Float32MultiArray::ConstPtr& array);
    void goalCB(const std_msgs::Float32MultiArray::ConstPtr& array);
    void hitCB(const std_msgs::Float32MultiArray::ConstPtr& array);
    void learningFlagCB(const std_msgs::Int32& iter);
    void hdetectionsCB(const nav_2d_msgs::Polygon2DStamped::ConstPtr& msgs);

    

    // ros parameter
    ros::Rate* rosrate;
    ros::NodeHandle* node;

    // ros messages
	std_msgs::Float32MultiArray velocityCommand;
    std_msgs::Float32MultiArray sumWeight;

    // ros publisher
    ros::Publisher velocityCommandPub;
    ros::Publisher sumWeightPub;
    
    // ros subscriber
    ros::Subscriber terminatorSub;
    ros::Subscriber lidarAngleSub;
    ros::Subscriber lidarDistanceSub;
    ros::Subscriber robotPoseSub;
    ros::Subscriber planningCommandSub;
    ros::Subscriber goalSub;
    ros::Subscriber hdetectionsSub;
    ros::Subscriber hitSub;
    ros::Subscriber learningFlagSub;

};

#endif //ROSCLASS_H
