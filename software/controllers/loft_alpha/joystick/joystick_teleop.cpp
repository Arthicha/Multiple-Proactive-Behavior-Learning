
#include "joystick.h"
#include <chrono>
#include <unistd.h>
#include <cmath>
#include "ros/ros.h"
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;

#define NODE_NAME                       "joystick_teleop_node"
#define JOYNAME                         "/dev/input/js0"

#define POSITION_PRESENT_TOPIC          "dxl_prs_pos"
#define VELOCITY_PRESENT_TOPIC          "dxl_prs_vel"
#define CURRENT_PRESENT_TOPIC           "dxl_prs_cur"
#define POSITION_COMMAND_TOPIC          "dxl_cmd_pos"
#define VELOCITY_COMMAND_TOPIC          "dxl_cmd_vel"
#define CURRENT_COMMAND_TOPIC           "dxl_cmd_cur"
#define THETA_COMMAND_TOPIC             "dxl_cmd_theta"
#define DYNAMIXEL_COMMAND_TOPIC         "dxl_cmd"
#define JOYSTICK_TOPIC                  "/joyStick"
#define BUFFER_SIZE                     2
#define LOOP_RATE                       60

#define NUMBER_OF_ACT                   7
#define number_of_axis                  8
#define number_of_button                11


#define STATE_STICK_LEFT_HOR            0
#define STATE_STICK_LEFT_VER            1
#define STATE_LT                        2
#define STATE_STICK_RIGHT_HOR           3
#define STATE_STICK_RIGHT_VER           4
#define STATE_RT                        5
#define STATE_ARROW_HOR                 6
#define STATE_ARROW_VER                 7
#define STATE_A                         8
#define STATE_B                         9
#define STATE_X                         10
#define STATE_Y                         11
#define STATE_LB                        12
#define STATE_RB                        13
#define STATE_BACK                      14
#define STATE_START                     15
#define STATE_LOGITECH                  16
#define STATE_STICK_button_LEFT         17
#define STATE_STICK_button_RIGHT        18
//*********************************************************************//
//                          FUNCTION DECLARATION                       //
//*********************************************************************//

void joy2msgs(int button, int16_t joy_value);
void trans2joytable();
// void publishPositon(uint16_t array_position[]);
uint16_t joy2dxl(int16_t joy_value);
// void callBack_pos(const std_msgs::Int16MultiArray::ConstPtr &msg);



//*********************************************************************//
//                                VARIABLE                             //
//*********************************************************************//
bool is_command_pos = false;
bool is_command_vel = false;
int action_button = 0;


std_msgs::Int16MultiArray joy_msgs;
std_msgs::Float32MultiArray manualGoal;
std_msgs::Float32MultiArray planFlag;
std_msgs::Float32MultiArray obstacleFlag;
// std_msgs::Int16MultiArray pos_cmd_msg;
// std_msgs::Int16MultiArray vel_cmd_msg;
int16_t joyAxis[number_of_axis] = {0};
int16_t joyBott[number_of_button] = {0};
int16_t joy_X = 0;
int16_t joy_Y = 0;
int16_t joy_A = 0;
int16_t joy_B = 0;
int16_t joy_LB = 0;
int16_t joy_RB = 0;
int16_t joy_LT = 0;
int16_t joy_RT = 0;
int16_t joy_BACK = 0;
int16_t joy_START = 0;
int16_t joy_LOGITECH = 0;
int16_t joy_ARROW_HOR = 0;
int16_t joy_ARROW_VER = 0;
int16_t joy_STICK_LEFT_HOR = 0;
int16_t joy_STICK_LEFT_VER = 0;
int16_t joy_STICK_RIGHT_HOR = 0;
int16_t joy_STICK_RIGHT_VER = 0;
int16_t joy_STICK_button_LEFT = 0;
int16_t joy_STICK_button_RIGHT = 0;
int16_t STATE = 0;
int16_t joy_table_value[number_of_axis + number_of_button +1] = {
	joy_STICK_LEFT_HOR,
	joy_STICK_LEFT_VER,
	joy_LT,
	joy_STICK_RIGHT_HOR,
	joy_STICK_RIGHT_VER,
	joy_RT,
	joy_ARROW_HOR,
	joy_ARROW_VER,
	joy_A,
	joy_B,
	joy_X,
	joy_Y,
	joy_LB,
	joy_RB,
	joy_BACK,
	joy_START,
	joy_LOGITECH,
	joy_STICK_button_LEFT,
	joy_STICK_button_RIGHT,
	STATE
};

bool button_state[number_of_axis + number_of_button + 1] = {false};
float button_previous[number_of_axis + number_of_button + 1] = {0};

float obstacle = 0;
int transition = 0;






//*********************************************************************//
//                                  MAIN                               //
//*********************************************************************//

int main(int argc, char** argv)
{
	joy_msgs.data.resize(0);

	//ROS init
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh;
	ros::Rate loop_rate(LOOP_RATE);

	ros::Publisher JOY_PUB = nh.advertise<std_msgs::Int16MultiArray>(JOYSTICK_TOPIC, BUFFER_SIZE);
	ros::Publisher manualGoalPub = nh.advertise<std_msgs::Float32MultiArray>("/manualGoal",1);
	ros::Publisher planFlagPub = nh.advertise<std_msgs::Float32MultiArray>("joystick/planFlag",1);
	ros::Publisher obstacleFlagPub = nh.advertise<std_msgs::Float32MultiArray>("joystick/obstacleFlag",1);

	// Create an instance of Joystick
	Joystick joystick(JOYNAME);
	
	// Ensure that it was found and that we can use it
	if (!joystick.isFound())
	{
		printf("open failed.\n");
		exit(1);
	}
	
	printf("Start joystick........\n");
	button_state[STATE_LOGITECH] = true;

	auto start = chrono::steady_clock::now();

	while (ros::ok()){

		JoystickEvent event;
		// Attempt to sample an event from the joystick
		planFlag.data.clear();
		planFlag.data.push_back(0.0);
		if (joystick.sample(&event)){
			
			/*
			if (event.isButton()){
				joyBott[event.number] = event.value;
				action_button = number_of_axis + event.number;

			}*/

			if (event.isAxis()){
				joyAxis[event.number] = event.value;
				action_button = event.number;
			}

			
			//printf("action_button = %d : %d\n",action_button,event.value);
			trans2joytable();
			JOY_PUB.publish(joy_msgs);

			
			if (event.isButton())
			{
				if  (int(event.number) == 1) planFlag.data[0] = 1.0;
				if  (int(event.number) == 2) 
				{
					auto end = chrono::steady_clock::now();
					if (chrono::duration_cast<chrono::milliseconds>(end - start).count() > 500) 
					{
						obstacle = float(int(obstacle+1)%2);
						obstacleFlag.data.clear();
						obstacleFlag.data.push_back(obstacle);
						start = chrono::steady_clock::now();
					}
					

				}
			}
			manualGoal.data.clear();
			float gx = joy_msgs.data[0]/32768.0;
			float gy = joy_msgs.data[1]/32768.0;
			manualGoal.data.push_back(gx);
			manualGoal.data.push_back(gy);
			manualGoalPub.publish(manualGoal);
			
		}//ifjoystick event
		planFlagPub.publish(planFlag);
		obstacleFlagPub.publish(obstacleFlag);

			//nh.setParam("/AVIS_param/lift", 10*int(button_state[i]));
			
		

		ros::spinOnce();
		loop_rate.sleep();
	}//while
	return 0;
}//main



//*********************************************************************//
//                                FUNCTION                             //
//*********************************************************************//

// void callBack_pos(const std_msgs::Int16MultiArray::ConstPtr &msg)
// {
//   // printf("pos = %d %d %d %d %d\n",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4]);
//   printf("pos = ");
//   for (int i = 0; i < msg->data.size(); i++)
//   {
// 	DXL_present_position[i] = msg->data[i];
// 	printf(" %d ", DXL_present_position[i]);
//   } //for
//   printf("\n");
// //   command_pos = true;
// } //callBack


//____________________________________________________________//

uint16_t joy2dxl(int16_t joy_value){
	return (joy_value + 32767)/16;
}//joy2dxl

// void publishPositon(uint16_t array_position[]){
//     pos_cmd_msg.data.resize(0);
//     for(int i = 0; i < NUMBER_OF_ACT; i++){
//         pos_cmd_msg.data.push_back(array_position[i]);
//     }//for
//     is_command_pos = true;
// }//publishPositon

//____________________________________________________________//

void joy2msgs(int button, int16_t joy_value){
	joy_msgs.data[button] = joy_value;
}//joy2msgs

//____________________________________________________________//

void trans2joytable(){

	for(int i = 0; i < number_of_axis; i++){
		joy_table_value[i] = joyAxis[i];
	}
	/*for(int j = 0; j < number_of_button; j++){
		joy_table_value[j + number_of_axis] = joyBott[j];
	}
	joy_table_value[19] = action_button;
	*/
	joy_msgs.data.resize(0);
	for(int i = 0; i < number_of_axis + number_of_button + 1; i++){
		joy_msgs.data.push_back(joy_table_value[i]);
	}//for
	
}//trans2joytable
//____________________________________________________________//


// void actionFunction(int state){
//     switch (state)
//     {
//     case STATE_START:
//         if(joy_START == 1){
//             publishPositon(standard_posture);
//         }
//         state = -1;
//         break;
	
//     case STATE_BACK:
//         if(joy_BACK == 1){
//             publishPositon(standard_posture);
//         }
//         state = -1;
//         break;
	
//     default:
//         break;
//     }
// }//actionFunction

//____________________________________________________________//
