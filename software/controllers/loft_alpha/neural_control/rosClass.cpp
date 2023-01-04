//
// created by zumo arthicha.
//

#include "rosClass.h"


using namespace std;

rosClass::rosClass(int argc, char *argv[]) {

	/* -------------------------------------------------------------------------------

	constructor: rosClass
		- description: create a ros node, initialize data-related variable, and initialize
		ros publisher & subscriber.
		- input: none
		- output: none
		- parameters: ros rate

	------------------------------------------------------------------------------- */

    // create ros node
    std::string nodeName("neural_controller");
    ros::init(argc,argv,nodeName);
    node = new ros::NodeHandle("~");

    ros::param::param<float>("/robot_control_hz", rosupdaterate, 1.0);
    rosrate = new ros::Rate(rosupdaterate);

    // check robot operating system
    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");
    ROS_INFO("simROS just started!");
    
    // initialize ros publishers
	velocityCommandPub = node->advertise<std_msgs::Float32MultiArray>("/neuralcontrol/twistCommand",1);
	sumWeightPub = node->advertise<std_msgs::Float32MultiArray>("/neuralcontrol/weights",1);

	// initialize ros subscribers
    terminatorSub = node->subscribe("/terminate",1,&rosClass::terminateCB,this);
    lidarAngleSub = node->subscribe("/simulation/lidar/angle",1,&rosClass::lidarAngleCB,this);
    lidarDistanceSub = node->subscribe("/simulation/lidar/distance",1,&rosClass::lidarDistanceCB,this);
    robotPoseSub = node->subscribe("/simulation/pose",1,&rosClass::robotPoseCB,this);
    planningCommandSub = node->subscribe("/planning/twistCommand",1,&rosClass::planningCommandCB,this);
    goalSub = node->subscribe("/planning/goal",1,&rosClass::goalCB,this);
    hdetectionsSub = node->subscribe("/socket_receiver/detections",1,&rosClass::hdetectionsCB,this);
    hitSub = node->subscribe("/simulation/hit",1,&rosClass::hitCB,this);
    learningFlagSub = node->subscribe("/planning/iteration",1,&rosClass::learningFlagCB,this);

    // initialize data variable
    lidarAngle.clear();
    lidarDistance.clear();
    for(int i=0;i<360;i++)
    {
    	lidarAngle.push_back(i+1);
    	lidarDistance.push_back(0.0);
    }
    goal.clear();
    planningCommand.clear();
    robotPose.clear();
    for(int i=0;i<2;i++)
   	{
   		goal.push_back(0.0);
   		robotPose.push_back(0.0);
   	}
   	for(int i=0;i<4;i++)
   	{
   		planningCommand.push_back(0.0);
   	}
   	robotPose.push_back(0.0);

   	hit = 0.0;

	
} // end:constructor


bool rosClass::rosStep()
{
	/* -------------------------------------------------------------------------------

	public: rosStep
		- description: if ros::ok(), update ros parameter & update publisher/subscriber
		- input: none
		- output: ros::ok()

	------------------------------------------------------------------------------- */

	if (ros::ok())
	{
		getRosParam();
		ros::spinOnce();
		this->rosrate->sleep();
		return true;
	}else{
		cout << "ROS NOT OK" << endl;
		return false;
	}

} // end:rosStep()

void rosClass::getRosParam()
{
	/* -------------------------------------------------------------------------------

	private: getRosParam
		- description: update ros parameter, put values in the corresponding variables
		- input: none
		- output: none

	------------------------------------------------------------------------------- */

	// ros parameters
    ros::param::param<float>("/robot_radius", robot_radius, 0.20);
    ros::param::param<float>("/learning_rate", eta, 0.0);
    ros::param::param<float>("/sensor_limit", sensor_limit, 2.0);
    ros::param::param<int>("/n_sensor", n_sensor, 14);
    ros::param::param<float>("/cover_range", cover_range, 90);
    ros::param::param<float>("/max_speed", max_speed, 0.5);
    ros::param::param<int>("/sleept", sleept, 500);
    ros::param::param<float>("/learned_weight", learned_weight, 0.0);
    ros::param::param<float>("/detectionlength_gain", detectionlength_gain, 1.0);
    ros::param::param<float>("/detectionangle_gain", detectionangle_gain, 1.0);
    ros::param::param<float>("/detectionangle_offset", detectionangle_offset, 0.0);
}

void rosClass::publishVelocityCommand(float v,float w)
{
	if (ros::ok())
	{
		velocityCommand.data.clear();
		velocityCommand.data.push_back(v);
		velocityCommand.data.push_back(w);
		velocityCommandPub.publish(velocityCommand);
	}
	
}

void rosClass::publishSumWeight(vector<float> ws)
{
	if (ros::ok())
	{
		sumWeight.data.clear();
		for (int i=0;i<ws.size();i++)
		{
			sumWeight.data.push_back(ws.at(i));
		}
		sumWeightPub.publish(sumWeight);
	}
	
}

float rosClass::getLidarAngle(int idx)
{
	if (idx < lidarAngle.size())
	{
		return lidarAngle[idx];
	}
	return 0;
}

float rosClass::getLidarDistance(int idx)
{
	if (idx < lidarDistance.size())
	{
		return lidarDistance[idx];
	}
	return 10;
}

float rosClass::getRobotPose(int idx)
{
	if (idx < robotPose.size())
	{
		return robotPose[idx];
	}
	return 0;
}

float rosClass::getPlanningCommand(int idx)
{
	if (idx < planningCommand.size())
	{
		return planningCommand[idx];
	}
	return 0;
}

float rosClass::getGoal(int idx)
{
	if (idx < goal.size())
	{
		return goal[idx];
	}
	return 0;
}

float rosClass::getHit()
{
	return hit;
}

int rosClass::getLidarSize()
{
	return lidarAngle.size();
}

int rosClass::getLearningFlag()
{
	return learningFlag;
}


void rosClass::terminateCB(const std_msgs::Bool& terminate_topic)
{
    terminate_status=terminate_topic.data;
    if (terminate_status)
    {
    	ros::shutdown();
    }
}

void rosClass::lidarAngleCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	int i = 0;
	
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        lidarAngle[i] = *it;
        i++;
    }
    return;
}

void rosClass::lidarDistanceCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	int i = 0;
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        lidarDistance[i] = *it;
        i++;
    }
    return;
}

void rosClass::robotPoseCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	int i = 0;
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        robotPose[i] = *it;
        i++;
    }
    return;
}

void rosClass::planningCommandCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	int i = 0;
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        planningCommand[i] = *it;
        i++;
    }
    return;
}

void rosClass::goalCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	int i = 0;
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        goal[i] = *it;
        i++;
    }
    return;
}

void rosClass::hdetectionsCB(const nav_2d_msgs::Polygon2DStamped::ConstPtr& msgs)
{
	detectionsx.clear();
	detectionsy.clear();
	if (msgs->polygon.points.size() > 0)
	{
		for(int i =0 ; i<msgs->polygon.points.size(); i++)
	    {
	        detectionsx.push_back(msgs->polygon.points[i].x*detectionlength_gain);
	        detectionsy.push_back(msgs->polygon.points[i].y*detectionangle_gain+detectionangle_offset);
	    }
	}
}

void rosClass::hitCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	int i = 0;
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        hit = *it;
        i++;
    }
    return;
}

void rosClass::learningFlagCB(const std_msgs::Int32& iter)
{
    learningFlag = iter.data;
    return;
}

rosClass::~rosClass() {
    ROS_INFO("simROS terminated!");
}
