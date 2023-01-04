/**********************************************************************
 *                                                                    *
 *                 neural proactive control (main)                    *
 *                                                                    *
 *  create by: arthicha srisuchinnawong (arsri21@student.sdu.dk)      *      
 *  date: 22 Aug 2022                                                 * 
 *                                                                    *
 *  description: the program/ros-node generates an proactive twist    *
 *               command using reactive neural control with           *
 *               correlation-based learning                           *
 *                                                                    *                                                          
 *********************************************************************/

/* --------------------------------------------------------------------
                          include libraries
-------------------------------------------------------------------- */

// additional standard library
#include <iomanip>
#include <chrono>
#include <functional>
#include <memory>

// ros interface
#include "rosClass.h"

// neural control
#include "neural_control.h"

/* --------------------------------------------------------------------
                               global variables
-------------------------------------------------------------------- */

// operation state
bool running = true; // node state

float vn = 0.0; // neural linear velocity
float wn = 0.0; // neural angular velocity

vector<float> weights; // NPC weight

float d_goal = 0.0;
float dx = 0.0;
float dy = 0.0;

long tau = 0.0;
float avgw = 0.0;
float alpha = 0.0;

int planningIteration = 0;

/* --------------------------------------------------------------------
                               function
-------------------------------------------------------------------- */

float sign(float x)
{
	if (x >= 0) return 1;
	return -1;
}

/* --------------------------------------------------------------------
                               main loop
-------------------------------------------------------------------- */


int main(int argc, char * argv[]) {

	// initialize ros class & NPC
	rosClass ros(0,nullptr);
	ros.getRosParam();
	neuralControl nc(ros.n_sensor,ros.sensor_limit,ros.learned_weight);

	// initialize weight vector
	weights.clear();
	for(int i=0;i<(ros.n_sensor);i++) weights.push_back(0);

	while(running)
	{
		// update neural control
		nc.setLearningRate(ros.eta); // update learning rate
		nc.setRobotRadius(ros.robot_radius); // update robot radius
 
		/* --------------------------------------------------------------------
		                         get planning twist command
		-------------------------------------------------------------------- */

		// get planner twist command (v,w)
		float v = ros.getPlanningCommand(0);
		float w = ros.getPlanningCommand(1);

		// compute heading direction
		float theta = atan2(w*ros.robot_radius,v);
		while (theta >= 3.14) theta = theta-3.14*2;
		while (theta <= -3.14) theta = theta+3.14*2;

		/* --------------------------------------------------------------------
		                         set planning twist command
		-------------------------------------------------------------------- */

		// set planning input to the neural control
		nc.setControlInput(0,(v/ros.max_speed)*sign(v)*cos(theta)); // velocity 
		nc.setControlInput(1,theta); // heading angle
		
		/* --------------------------------------------------------------------
		                            set sensory feedback
		-------------------------------------------------------------------- */
		
		// compute sensor value
		for(int i=0;i<(ros.n_sensor);i++) // loop through all sensors & get min sensor value
		{

			// search for corresponding angle
			float range = 0;
			float l_key = 1000.0;
			float theta_key = 0.0;
			float angleidx = 0.0;
			range = ros.cover_range/(((ros.n_sensor-1)/2));

			for(int j= 0;j<range;j++)
			{
				if (i < (ros.n_sensor/2))
					angleidx = 0-(ros.cover_range/2)+(i*range)+j;
				else
					angleidx = 180-(ros.cover_range/2)+((i-(ros.n_sensor/2))*range)+j;	



				while (angleidx < 0) angleidx = angleidx+360;
				while (angleidx >= 360) angleidx = angleidx-360;

				float l = ros.getLidarDistance(angleidx); // distance/feedback
				if (l < l_key) // keep minimum (closest)
				{ 
					l_key = l;
					theta_key = ros.getLidarAngle(angleidx); // get angle
				}
			} 
			theta_key += 180.0;
			while(theta_key > 360.0) theta_key -= 360;
			theta_key = theta_key*3.14/180;

			if (l_key > ros.sensor_limit) l_key = ros.sensor_limit; // saturation

			nc.setSensorInput(i,l_key);
			nc.setAngleInput(i,-theta_key);

		}


		/* --------------------------------------------------------------------
		                            update neural control
		-------------------------------------------------------------------- */

		// perform sleep for xxxx iteration after start. 
		if (tau > ros.sleept)
		{
			if (planningIteration != ros.getLearningFlag())
			{
				nc.update(1);
				planningIteration = ros.getLearningFlag();
			}
			else{  nc.update(0);  }
		}else{
			tau += 1;
		}
		
		/* --------------------------------------------------------------------
		                      get and publish twist command
		-------------------------------------------------------------------- */

		// get NPC command
		vn = nc.getOutput(0);
		wn = nc.getOutput(1);


		// get NPC weight
		for(int i=0;i<(ros.n_sensor);i++) weights[i] = nc.getWeight(i);

		
		// publish twist command and weight
		ros.publishVelocityCommand(vn,wn);
		ros.publishSumWeight(weights);

		// step ros
		running = ros.rosStep();

	}

	return 0; // if program end
}


