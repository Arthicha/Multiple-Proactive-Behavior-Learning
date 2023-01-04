#ifndef NEURAL_CONTROL_H
#define NEURAL_CONTROL_H



#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <map>
#include <ros/ros.h>
#include <random>
#include "neuron.h"
#include "synapse.h"
#include "ann.h"
#include "interpolator2d.h"
#include "neural_unit.h"

#define PI 3.14159265
//#define D 0.2

class neuralControl: public ANN{
public:
	neuralControl(int size,float maxD,float learned_weight);

	void setSensorInput(int idx, float input);
	void setAngleInput(int idx, float input);
	void setControlInput(int idx, float input);
	void setLearningRate(float eta_);
	void setRobotRadius(float rd_);

	float update(bool learning);


	

	float getOutput(int idx);
	float getWeight(int i);

private:
	
	int num;
	int iter;
	float v_out;
	float w_out;

	float eta = 0.0; // 0.0 or 0.5
	float robot_radius = 0.5; 


	vector<neuralUnit*> units; // neural unit
	vector<float> weight; // weight update corresponding to the neural unit

	vector<float> xs; 
	vector<float> xs2; 
	vector<float> smoothsignal[2]; 
	
	vector<Neuron*> inputAngleNeurons;
	vector<Neuron*> inputNeurons;
	vector<Neuron*> controlNeurons;

	float relu(float x);
	float sign(float x);
	float sat(float x,float sat);
	float sigma(float x, float tau);
	

};

#endif //NEURAL_CONTROL_H