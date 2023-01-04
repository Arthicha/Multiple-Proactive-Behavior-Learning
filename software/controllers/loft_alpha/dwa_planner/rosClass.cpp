//
// created by zumo arthicha.
// date: 22 Aug 2022
// planner-ros interface
//

#include "rosClass.h"

/* -------------------------------------------------------------------------------
                                setup methods
 ------------------------------------------------------------------------------- */


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
    std::string nodeName("planner");
    ros::init(argc,argv,nodeName);
    node = new ros::NodeHandle("~");

    ros::param::param<float>("/planning_hz", rosupdaterate, 1.0);
	rosrate = new ros::Rate(rosupdaterate);

    // check robot operating system
    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");
    ROS_INFO("simROS just started!");

    // initialize ros publishers
	velocityCommandPub = node->advertise<std_msgs::Float32MultiArray>("/planning/twistCommand",1);
	iterationPub = node->advertise<std_msgs::Int32>("/planning/iteration",1);
	realDetectionPub = node->advertise<std_msgs::Float32MultiArray>("/simulation/humanxy",1);
	expectationPub = node->advertise<std_msgs::Float32MultiArray>("/planning/expectation",1);

	// initialize ros subscribers
    terminatorSub = node->subscribe("/terminate",1,&rosClass::terminateCB,this);
    lidarAngleSub = node->subscribe("/simulation/lidar/angle",1,&rosClass::lidarAngleCB,this);
    lidarDistanceSub = node->subscribe("/simulation/lidar/distance",1,&rosClass::lidarDistanceCB,this);
    robotPoseSub = node->subscribe("/simulation/pose",1,&rosClass::robotPoseCB,this);
    goalSub = node->subscribe("/planning/goal",1,&rosClass::goalCB,this);
    hdetectionsSub = node->subscribe("/socket_receiver/detections",1,&rosClass::hdetectionsCB,this);

    // initialize data variable
    lidarAngle.clear();
    lidarDistance.clear();
    for(int i=0;i<360;i++)
    {
    	lidarAngle.push_back(i+1);
    	lidarDistance.push_back(0.0);
    }
    robotPose.clear();
    goal.clear();
    for(int i=0;i<2;i++)
   	{
   		robotPose.push_back(0.0);
   		goal.push_back(0.0);
   	}
   	robotPose.push_back(0.0);

   	iteration.data = 0;

	
} // end:constructor

rosClass::~rosClass() {
	/* -------------------------------------------------------------------------------

	destructor: ~rosClass
		- description: print and terminate ros interface
		- input: none
		- output: none

	------------------------------------------------------------------------------- */

	ROS_INFO("simROS terminated!");
}


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


void rosClass::publishVelocityCommand(float v,float w)
{
	//float D = 0.34;
	//float v1 = 0.5*(2*v-tan(w)*D);
	//float v2 = 0.5*(2*v+tan(w)*D);
	if (ros::ok())
	{
		velocityCommand.data.clear();
		velocityCommand.data.push_back(v);
		velocityCommand.data.push_back(w);
		velocityCommandPub.publish(velocityCommand);
	}
	
}

void rosClass::publishIteration()
{
	if (ros::ok())
	{
		iteration.data += 1;
		iterationPub.publish(iteration);
	}
	
}

void rosClass::publishExpectation(float x1, float y1, float yaw1,float x2, float y2, float yaw2)
{
	if (ros::ok())
	{
		expectations.data.clear();
		expectations.data.push_back(x1);
		expectations.data.push_back(y1);
		expectations.data.push_back(yaw1);
		expectations.data.push_back(x2);
		expectations.data.push_back(y2);
		expectations.data.push_back(yaw2);
		expectationPub.publish(expectations);
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
	if (idx < lidarAngle.size())
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
	return 10;
}

float rosClass::getGoal(int idx)
{
	if (idx < goal.size())
	{
		return goal[idx];
	}
	return 0;
}

int rosClass::getLidarSize()
{
	return lidarAngle.size();
}

void rosClass::getRosParam()
{
	// ros parameters
    ros::param::param<float>("/max_speed", max_speed, 0.5);
    ros::param::param<float>("/speed_resolution", speed_resolution, 0.05);
    ros::param::param<float>("/yaw_resolution", yaw_resolution, 0.5);
    ros::param::param<float>("/predict_time", predict_time, 3.0);
    ros::param::param<float>("/dt", dt, 0.1);
    ros::param::param<float>("/clearance_gain", clearance_cost, 1.0);
    ros::param::param<float>("/heading_gain", heading_cost, 10.0);
    ros::param::param<float>("/velocity_gain", velocity_cost, 0.0);
    ros::param::param<float>("/detectionlength_gain", detectionlength_gain, 1.0);
    ros::param::param<float>("/detectionangle_gain", detectionangle_gain, 1.0);
    ros::param::param<float>("/detectionangle_offset", detectionangle_offset, 0.0);
    ros::param::param<float>("/sensor_limit", sensor_limit, 2.0);
    ros::param::param<int>("/coarse_step", coarse_step, 2);
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
	realDetection.data.clear();
	if (msgs->polygon.points.size() > 0)
	{
		float l = 0;
		float theta = 0;

		for(int i =0 ; i<msgs->polygon.points.size(); i++)
	    {
	    	l = detectionlength_gain*msgs->polygon.points[i].x;
	    	theta = detectionangle_gain*msgs->polygon.points[i].y+detectionangle_offset;

	        detectionsx.push_back(l);
	        detectionsy.push_back(theta);
	        realDetection.data.push_back(-1*l*cos(theta));
	        realDetection.data.push_back(l*sin(theta));
	    }
	}
	realDetectionPub.publish(realDetection);
	
}

