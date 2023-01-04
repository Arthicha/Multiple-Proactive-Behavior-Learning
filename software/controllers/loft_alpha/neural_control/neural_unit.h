//
// created by zumo arthicha.
// date: 22 Aug 2022
// neural control unit (corresponding to each angle)
//

#ifndef NEURAL_UNIT_H
#define NEURAL_UNIT_H

/* -------------------------------------------------------------------------------
								include libraries
 ------------------------------------------------------------------------------- */

// standard libraries
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <map>
#include <random>

// ann libraries
#include "neuron.h"
#include "synapse.h"
#include "ann.h"
#include "interpolator2d.h"

// define name space
using namespace std;

// define variable
#define INIT_W 0.0

/* -------------------------------------------------------------------------------
								ros class
 ------------------------------------------------------------------------------- */

class neuralUnit: public ANN{
public:
	neuralUnit(); // constructor

	// set neural unit input
	void setInput(float value);
	void setWeight(float value);

	// get neural unit signals
	float getOutput();
	Neuron* getOutputNeuron();
	float getInput();
	Neuron* getInputNeuron();
	float getWeight();

	// update weight
	void updateWeight(float dw);

private:

	int num; // number of unit
	vector<float> weights; // weight
};

#endif //NEURAL_UNIT_H