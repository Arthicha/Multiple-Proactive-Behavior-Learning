//
// created by zumo arthicha.
// date: 21 Aug 2022
// robot control-ros interface
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
	std::string nodeName("robot_controller");
	ros::init(argc,argv,nodeName);
	node = new ros::NodeHandle("~");

	ros::param::param<float>("/robot_control_hz", rosupdaterate, 1.0);
	rosrate = new ros::Rate(rosupdaterate);

	// check robot operating system
	if(!ros::master::check())
		ROS_ERROR("ros::master::check() did not pass!");
	ROS_INFO("simROS just started!");

	// initialize ros publishers
	velocityCommandPub = node->advertise<std_msgs::Float32MultiArray>("/loft/velocityCommand",1);
	benchmarkCommandPub = node->advertise<std_msgs::Float32MultiArray>("/loft/twistCommand",1);
	twistCommandPub = node->advertise<geometry_msgs::Twist>("/cmd_vel",1);

	// initialize ros subscribers
	terminatorSub = node->subscribe("/terminate",1,&rosClass::terminateCB,this);
	planningSub = node->subscribe("/planning/twistCommand",1,&rosClass::planningCB,this);
	neuralControlSub = node->subscribe("/neuralcontrol/twistCommand",1,&rosClass::neuralControlCB,this);
	
	// initialize data variable
	neuralCommand.clear();
	planningCommand.clear();
	for (int i=0;i<2; i++)
	{
		neuralCommand.push_back(0.0);
		planningCommand.push_back(0.0);
	}

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

rosClass::~rosClass() {
	/* -------------------------------------------------------------------------------

	destructor: ~rosClass
		- description: print and terminate ros interface
		- input: none
		- output: none

	------------------------------------------------------------------------------- */

	ROS_INFO("simROS terminated!");
}


void rosClass::getRosParam()
{
	/* -------------------------------------------------------------------------------

	private: getRosParam
		- description: update ros parameter, put values in the corresponding variables
		- input: none
		- output: none

	------------------------------------------------------------------------------- */

	// ros parameters
	ros::param::param<float>("/wheel_radius", wheel_radius, 0.15);
	ros::param::param<float>("/robot_radius", robot_radius, 0.20);
	ros::param::param<float>("/v_gain", v_gain, 1.0);
	ros::param::param<float>("/w_gain", w_gain, 1.0);
	ros::param::param<float>("/max_speed", max_speed, 0.5);
}


/* -------------------------------------------------------------------------------
                                get variable methods
 ------------------------------------------------------------------------------- */

float rosClass::getPlanningCommand(int idx)
{
	/* -------------------------------------------------------------------------------

	public: getPlanningCommand
		- description: return planning command.
		- input: index (0 = linear velocity, 1 = angular velocity)
		- output: corresponding velocity command

	------------------------------------------------------------------------------- */

	return planningCommand[idx];
}


float rosClass::getNeuralCommand(int idx)
{
	/* -------------------------------------------------------------------------------

	public: getNeuralCommand
		- description: return neural control command.
		- input: index (0 = linear velocity, 1 = angular velocity)
		- output: corresponding velocity command

	------------------------------------------------------------------------------- */

	return neuralCommand[idx];
}

/* -------------------------------------------------------------------------------
                                publish methods
 ------------------------------------------------------------------------------- */


void rosClass::publishWheelCommand(float v1, float v2,  float v3, float alpha)
{
	/* -------------------------------------------------------------------------------

	public: publishWheelCommand
		- description: publish wheel command (joint command)
		- input: simulation velocity command
			+ v1: left wheel speed
			+ v2: right wheel speed
			+ v3: front/rear wheel speed
			+ alpha: front/rear wheel angle
		- output: none

	------------------------------------------------------------------------------- */


	if (ros::ok())
	{
		velocityCommand.data.clear();

		velocityCommand.data.push_back(v1);
		velocityCommand.data.push_back(v2);
		velocityCommand.data.push_back(v3);
		velocityCommand.data.push_back(alpha);

		velocityCommandPub.publish(velocityCommand);
	}
	
}


void rosClass::publishTwistCommand(float v, float w, float v_gain, float w_gain)
{
	/* -------------------------------------------------------------------------------

	public: publishTwistCommand
		- description: publish twist command (linear & angular velocity)
		- input: twist command
			+ v: linear velocity
			+ w: angular velocity
			+ v_gain: linear velocity gain
			+ w_gain: angular velocity gain
		- output: none

	------------------------------------------------------------------------------- */

	if (ros::ok())
	{
		// simulated twist command
		benchmarkCommand.data.clear();

		benchmarkCommand.data.push_back(v);
		benchmarkCommand.data.push_back(w);

		benchmarkCommandPub.publish(benchmarkCommand);

		// real twist command
		twistCommand.linear.x = v*v_gain;
		twistCommand.linear.y = 0;
		twistCommand.linear.z = 0;
		twistCommand.angular.x = 0;
		twistCommand.angular.y = 0;
		twistCommand.angular.z = w*w_gain;

		twistCommandPub.publish(twistCommand);
	}
	
}

/* -------------------------------------------------------------------------------
                                subscriber callback methods
 ------------------------------------------------------------------------------- */


void rosClass::terminateCB(const std_msgs::Bool& terminate_topic)
{
	/* -------------------------------------------------------------------------------

	callback: terminateCB
		- description: store terminate status
		- input: terminate message
		- output: none

	------------------------------------------------------------------------------- */

	terminate_status=terminate_topic.data;
	if (terminate_status) ros::shutdown();
}

void rosClass::planningCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	/* -------------------------------------------------------------------------------

	callback: planningCB
		- description: store planning command
		- input: planning message
		- output: none

	------------------------------------------------------------------------------- */

	int i = 0;
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		planningCommand[i] = *it;
		i++;
	}
	return;
}

void rosClass::neuralControlCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	/* -------------------------------------------------------------------------------

	callback: neuralControlCB
		- description: store neural control command
		- input: neural control message
		- output: none

	------------------------------------------------------------------------------- */

	int i = 0;
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		neuralCommand[i] = *it;
		i++;
	}
	return;
}



