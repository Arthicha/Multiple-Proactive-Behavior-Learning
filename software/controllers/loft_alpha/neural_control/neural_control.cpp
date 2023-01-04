#include "neural_control.h"

using namespace std;


neuralControl::neuralControl(int size,float maxD,float learned_weight){

	num = size;
	iter = 0;
	v_out =0;
	w_out =0;

	// initialization

	/* --------------------------------------------------------------------
		                   initialize neural signals/variables
	-------------------------------------------------------------------- */

	xs.clear();
	xs2.clear();
	weight.clear(); // reset weight update
	for(int k =0;k<2;k++) smoothsignal[k].clear();

	// append 0.0	
	for (int j=0;j<num;j++)
	{
		xs.push_back(0.0);
		xs2.push_back(0.0);
		weight.push_back(0.0);
		for(int k =0;k<4;k++) smoothsignal[k].push_back(0.0);
	}

	/* --------------------------------------------------------------------
		                       initialize neural structure
	-------------------------------------------------------------------- */

	// ----------------   initialize sensory input neuron (sensory feedback)
	inputNeurons.clear();
	inputAngleNeurons.clear();

	for (int j=0;j<num;j++)
	{
		// velocity input
		inputNeurons.push_back(addNeuron());
		inputNeurons.at(j)->setTransferFunction(identityFunction());
		w(inputNeurons.at(j),inputNeurons.at(j),1.0);

		// angle input
		inputAngleNeurons.push_back(addNeuron());
		inputAngleNeurons.at(j)->setTransferFunction(identityFunction());
		w(inputAngleNeurons.at(j),inputAngleNeurons.at(j),1.0);
	}

	// ----------------   initialize planning/control input neuron (planning command)
	controlNeurons.clear();

	for(int j=0;j<2;j++)
	{
		controlNeurons.push_back(addNeuron());
		controlNeurons.at(j)->setTransferFunction(identityFunction());
	}
	
	// ----------------   initialize neural unit
	units.clear();
	
	
	float weight_restore[14] = {10.738665580749512,5.253550052642822,3.1262898445129395,0.7919780611991882,
															3.1262898445129395,5.253550052642822,10.738665580749512,11.555553436279297,
															9.53602695465088,	 4.604489803314209,1.2357454299926758,4.604489803314209,
															9.53602695465088,11.555553436279297};
		

 	/* 	float weight_restore[12] = {2.671386957168579,	1.6904118061065674,	0.8246285915374756,	
 															1.0715118646621704,	1.5266811847686768,	1.8809876441955566,	
 															1.7709739208221436,	1.2171326875686646,	0.9487589597702026,	
 															0.8574652671813965,	1.9198253154754639,	2.793456792831421};
	*/
	for (int j=0;j<num;j++)
	{
		units.push_back(new neuralUnit());
		addSubnet(units.at(j));

		w(units.at(j)->getInputNeuron(),inputNeurons.at(j),-1.0/maxD);
		b(units.at(j)->getInputNeuron(),1.0);

		units.at(j)->setWeight((weight_restore[j])*learned_weight);
	}

}


float neuralControl::update(bool learning)
{
	float sumw = 0.0;



	/* --------------------------------------------------------------------
	                   compute neural network twist command
	-------------------------------------------------------------------- */

	float robot_speed = controlNeurons.at(0)->getOutput();

	vector<float> xes;
	xes.clear();
	for (int j=0;j<num;j++)
	{
		xes.push_back((controlNeurons.at(0)->getOutput())*cos(inputAngleNeurons.at(j)->getOutput()));
	}

	if (learning)
	{
		for (int j=0;j<num;j++) // for each unit
		{

			weight.at(j) = 0.0; // reset weight update to zero


			/* --------------------------------------------------------------------
			                   problem formulation (define x and y)
			-------------------------------------------------------------------- */

			// velocity component on corresponding angle (v cos(theta))
			float x = xes.at(j);

			// sensory feedback value
			float y = units.at(j)->getInput(); 
			/* --------------------------------------------------------------------
			              perform correlation-based learning on x and y
			-------------------------------------------------------------------- */
			
			if (1){//((y <= 0.5)) {
				
				smoothsignal[0].at(j) = 1*relu(y);
				
				
				if ((x >= 0.0) && ((xs2.at(j)) >= 0.0)) // detection
				{
					smoothsignal[1].at(j) = 1*relu(x);
				}else
				{
					smoothsignal[1].at(j) = 0;
				}

				if ((x > 0.0) && (xs2.at(j) > 0.0))
				{
					
					// regular correlation-based update
					// dw = di * v
					//weight.at(j) += 1*eta*(smoothsignal[0].at(j)-xs.at(j))*(smoothsignal[1].at(j));
					weight.at(j) += -1*eta*(smoothsignal[0].at(j))*( smoothsignal[1].at(j)-xs2.at(j));
					
					// stopping correlation-based update
					//weight.at(j) += -1*epsilon*eta*smoothsignal[0].at(j)*relu(abs(1.0*cos(inputAngleNeurons.at(j)->getOutput()))-abs(smoothsignal[1].at(j)));
					weight.at(j) += -0.001*(smoothsignal[0].at(j))*relu(abs(1.0*cos(inputAngleNeurons.at(j)->getOutput()))*(1-abs(smoothsignal[1].at(j))));
					
				}
				
				xs.at(j) = smoothsignal[0].at(j);
				xs2.at(j) = smoothsignal[1].at(j);
			}
			
			
		}

		// update weight
		for (int j=0;j<num;j++) 
		{
			int j_flip = 0;
			int diff = 0;
			int mid = 0;
			if (j < num/2)
			{
				mid = ((num/2)-1)/2;
				diff = j-mid;

			}else{
				mid = ((num/2)-1)/2+(num/2);
				diff = j-mid;
			}
			j_flip = mid-diff;
			units.at(j)->updateWeight(sat(weight.at(j)+weight.at(j_flip),0.002));
		}

	} // end: learning

	step();
	
	float ret = 0.0;
	float xi = 0.0;
	float theta = 0.0;
	float vn = 0.0;
	float wn = 0.0;
	//float d = D;
	//float r= WR;
	//cout << "----------------------" << endl;

	for (int j=0;j<num;j++)
	{
		xi = (units.at(j)->getOutput());
		theta = inputAngleNeurons.at(j)->getOutput();
		
		//if (((robot_speed <= 0) && (xi*ctheta < 0)) || ((robot_speed >=0) && (xi*ctheta > 0)))

		

		//cout << "in-class :  " << j << "\t" << theta << "\t" << x.at(j) << "\t" << robot_speed << endl;
		
		if (xes.at(j) > 0) // neuron and velocity are in the same direction
		{
			vn -= xi*cos(theta);
			//wn += -xi*sin(theta)*ctheta*(D);
			wn -= xi*sin(theta);//*robot_radius;
		}
		
	}
	//cout << "                                          " << vn << endl;
	v_out = vn;//sqrtf(vx*vx+vy*vy);
	//if (vx < 0) v_out *= -1;
	//w_out = wn;//atan2(vy,abs(vx));
	
	if (v_out <= 0)
	{
		w_out = wn;
	}else{
		w_out = -wn;
	}
	
	//if (vx < 0) w_out *= -1;


	/* --------------------------------------------------------------------
	                   perform correlation-based learning
	-------------------------------------------------------------------- */



	return sumw/num;
}

float neuralControl::getWeight(int i)
{
	return units.at(i)->getWeight();
}


void neuralControl::setSensorInput(int idx, float input)
{
	inputNeurons.at(idx)->setActivity(input);
	inputNeurons.at(idx)->setOutput(input);
}

void neuralControl::setAngleInput(int idx, float input)
{
	inputAngleNeurons.at(idx)->setActivity(input);
	inputAngleNeurons.at(idx)->setOutput(input);
}


void neuralControl::setControlInput(int idx, float input)
{
	controlNeurons.at(idx)->setActivity(input);
	controlNeurons.at(idx)->setOutput(input);
}

void neuralControl::setLearningRate(float eta_)
{
	eta = eta_;
}

void neuralControl::setRobotRadius(float rd_)
{
	robot_radius = rd_;
}

float neuralControl::getOutput(int idx)
{
	
	if (idx == 0)
	{
		return v_out;
	}else{
		return w_out;
	}
}

float neuralControl::relu(float x)
{
	if (x < 0){ return 0;}
	return x;
}

float neuralControl::sign(float x)
{
	if (x >= 0) return 1;
	return -1 ;
}

float neuralControl::sat(float x,float sat)
{
  if (x < -sat) return -sat;
  if (x > sat) return sat;
  return x;
}


float neuralControl::sigma(float x, float tau)
{
	if (x < tau)
	{
		return 1;
	}
	return 0;
}