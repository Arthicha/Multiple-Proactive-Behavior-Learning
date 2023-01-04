//
// created by zumo arthicha.
// date: 22 Aug 2022
// neural control unit (corresponding to each angle)
//

#include "neural_unit.h"

/* -------------------------------------------------------------------------------
                                setup methods
 ------------------------------------------------------------------------------- */

neuralUnit::neuralUnit(){

	/* -------------------------------------------------------------------------------

	constructor: neuralUnit
		- description: initlize neuron unit
		- input: none
		- output: none
		- parameters: INIT_W

	------------------------------------------------------------------------------- */

	// create neurons
	num = 2;
	setNeuronNumber(num);
	for(int i=0;i<num;i++) setTransferFunction(i,identityFunction());
	
	// initialize connection
	w(1, 0, INIT_W);
	weights.clear();
	weights.push_back(INIT_W);
}


void neuralUnit::setInput(float value)
{
	/* -------------------------------------------------------------------------------

	public: setInput
		- description: set input value
		- input: input value
		- output: none
		- parameters: none

	------------------------------------------------------------------------------- */
	
	setActivity(0,value);
    setOutput(0,value);
}

float neuralUnit::getOutput()
{
	/* -------------------------------------------------------------------------------

	public: getOutput
		- description: get output value
		- input: none
		- output: output value
		- parameters: none

	------------------------------------------------------------------------------- */

	return getNeuron(1)->getOutput();
}

Neuron* neuralUnit::getOutputNeuron()
{
	/* -------------------------------------------------------------------------------

	public: getOutputNeuron
		- description: get output neuron
		- input: none
		- output: output neuron
		- parameters: none

	------------------------------------------------------------------------------- */

	return getNeuron(1);
}

float neuralUnit::getInput()
{
	/* -------------------------------------------------------------------------------

	public: getInput
		- description: get input value
		- input: none
		- output: input value
		- parameters: none

	------------------------------------------------------------------------------- */

	return getNeuron(0)->getOutput();
}

Neuron* neuralUnit::getInputNeuron()
{
	/* -------------------------------------------------------------------------------

	public: getInputNeuron
		- description: get input neuron
		- input: none
		- output: input neuron
		- parameters: none

	------------------------------------------------------------------------------- */

	return getNeuron(0);
}

float neuralUnit::getWeight()
{
	/* -------------------------------------------------------------------------------

	public: getWeight
		- description: get weight value
		- input: none
		- output: weight value
		- parameters: none

	------------------------------------------------------------------------------- */

	return weights[0];
}

void neuralUnit::setWeight(float value)
{
	/* -------------------------------------------------------------------------------

	public: setWeight
		- description: get weight value
		- input: weight value
		- output: none
		- parameters: none

	------------------------------------------------------------------------------- */

	weights[0] = value;
}


void neuralUnit::updateWeight(float dw)
{
	/* -------------------------------------------------------------------------------

	public: updateWeight
		- description: update/add weight by the given value
		- input: weight change
		- output: none
		- parameters: none

	------------------------------------------------------------------------------- */

	weights[0] = weights[0] + (dw); // update weight
	if (weights[0] < 0.0) weights[0] = 0.0; // saturation
	w(1, 0, weights[0]);
}





