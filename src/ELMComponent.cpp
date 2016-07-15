/*
 * ELMComponent.cpp
 *
 *  Created on: Jun 13, 2016
 *      Author: abalayn
 */

#include "ELMComponent.hpp"
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

#include <fstream>
#include <iostream>

#include "RealVector.h"
#include "ExtremeLearningMachine.h"

ELMComponent::ELMComponent(std::string const& name) : RTT::TaskContext(name),curr_mass(1) , elm_id(0) ,comp_wait(10000), nb_joints(6) ,currVelocity_Flow(RTT::NoData), meanJntTrq_Flow(RTT::NoData),  currJntPos_Flow(RTT::NoData) {	// TODO Auto-generated constructor stub



	this->addPort("cmdJntPos", cmdJntPos_Port);
	currPos = {0 , 0 ,0 ,0 ,0 , 0};
	dsrPos = {0 , 0 ,0 ,0 ,0 , 0};

	cmdJntPos_Port.setDataSample(currPos);

	this->addPort("currJntPos" , currJntPos_Port);

	this->addPort("meanJntTrq" , meanJntTrq_Port);

	this->addPort("currMass" , currMass_Port);

	this->addPort("currVelo" , currVelocity_Port);
}


bool ELMComponent::startHook() {
    return true;
}

void ELMComponent::stopHook() {
	return ;
}

void ELMComponent::cleanupHook() {
	return ;
}

bool ELMComponent::configureHook() {
	RTT::log(RTT::Error) << "Beginning ELMComponent configuration." << RTT::endlog();

	for (unsigned j = 0; j < nb_joints; j++)
		{
			torque_difference.push_back(0);
			torque_difference.push_back(0);
			add_trq.push_back(0);
			thresholds[0].push_back(0);
			thresholds[1].push_back(0);
			thresholds[2].push_back(0);
			thresholds[3].push_back(0);
			meanTrq.push_back(0);
			currVelo.push_back(0);
		}


	// 0 payload
	thresholds[0][0] = 4;//31;
	thresholds[0][1] = 1.6;//12; //12;
	thresholds[0][2] = 1.0;//159; //30; // see if smaller could be ok. (if the position difference is not high).
	thresholds[0][3] = 1;//30;//29;
	thresholds[0][4] = 1.0;//29;//29; // For mass 5.
	thresholds[0][5] = 1;//40;// 40;

	// 1 payload
	thresholds[1][0] = 5;//31;
	thresholds[1][1] = 1.6;//4.5;//7;//12; //12;
	thresholds[1][2] = 1.0;//4.5;//6.1;//159; //30; // see if smaller could be ok. (if the position difference is not high).
	thresholds[1][3] = 2.4;//30;//29;
	thresholds[1][4] = 1.0;//29;//29; // For mass 5.
	thresholds[1][5] = 1;//40;// 40;

	//3 payload
	thresholds[2][0] = 5;//31;
	thresholds[2][1] = 2;//12; //12;
	thresholds[2][2] = 2;//159; //30; // see if smaller could be ok. (if the position difference is not high).
	thresholds[2][3] = 1;//30;//29;
	thresholds[2][4] = 1.0;//29;//29; // For mass 5.
	thresholds[2][5] = 1;//40;// 40;

	// 5 payload
	thresholds[3][0] = 5;//31;
	thresholds[3][1] = 4.2;//12; //12;
	thresholds[3][2] = 2.0;//159; //30; // see if smaller could be ok. (if the position difference is not high).
	thresholds[3][3] = 1;//30;//29;
	thresholds[3][4] = 1.5;//29;//29; // For mass 5.
	thresholds[3][5] = 1;//40;// 40;

	add_trq[0] = 0.01/4.0;
	add_trq[1] = 0.008/4.0;
	add_trq[2] = 0.008/4.0;
	add_trq[3] = 0.01/4.0;
	add_trq[4] = 0.01/4.0;
	add_trq[5] = 0.01/4.0;


	RTT::log(RTT::Error) << "ELM 1 done." << RTT::endlog();


		// ELM model creation.

		std::string infile("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/models/elmmodel_mass_0/data");
		elm_0=ExtremeLearningMachine::create(infile);
		RTT::log(RTT::Warning)  << "Generating testdata with dimensionality: " << elm_0->getInputDimension() << RTT::endlog();
		RTT::log(RTT::Warning) << "Expecting results with dimensionality: " << elm_0->getOutputDimension() << RTT::endlog();
		RTT::log(RTT::Error) << "ELM loaded" << RTT::endlog();
		std::string infile1("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/models/elmmodel_mass_1/data");
		elm_1=ExtremeLearningMachine::create(infile1);
		std::string infile5("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/models/elmmodel_mass_5/data");
		elm_5=ExtremeLearningMachine::create(infile5);
		std::string infile3("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/models/elmmodel_mass_3/data");
		elm_3=ExtremeLearningMachine::create(infile3);


		RTT::log(RTT::Error) << "ELM Component configured." << RTT::endlog();

		return true;

}

void ELMComponent::updateHook(){
//	RTT::log(RTT::Error) << "Beginning ELMComponent update." << RTT::endlog();

	currJntPos_Flow = currJntPos_Port.read(currPos);
	meanJntTrq_Flow = meanJntTrq_Port.read(meanTrq);
	currMass_Flow = currMass_Port.read(curr_mass);
	currVelocity_Flow = currVelocity_Port.read(currVelo);



	/***************************************************************************************************************************************************/

	/*
	 * ELM part:
	 * Computing the difference between current torque and awaited torque for each joint.
	 */





	if (meanJntTrq_Flow == RTT::NewData || currJntPos_Flow == RTT::NewData)
	{
		RealVectorPtr inputdata = RealVector::create(elm_0->getInputDimension(), 0.0);
		for (int j=0; j<inputdata->getDimension(); j++) inputdata->setValueEquals(j,currPos[j]);
		RealVectorPtr result;
		if (curr_mass == 0)
		{
			result = elm_0->evaluate(inputdata);
		}
		else if (curr_mass == 1)
		{
			result = elm_1->evaluate(inputdata);
		}
		else if (curr_mass == 3)
		{
			result = elm_3->evaluate(inputdata);
		}
		else if (curr_mass == 5)
		{
			result = elm_5->evaluate(inputdata);
		}

		for (unsigned j = 0; j < meanTrq.size(); j++)
		{

			torque_difference[j] = result->getValue(j) - meanTrq[j];

	//		if ((sim_id >= 10000))
	//		{
					double curr_threshold;
					if (curr_mass - 0.0001 == 0)
					{
						curr_threshold = thresholds[0][j];
					}
					else if (curr_mass == 1)
					{
						curr_threshold = thresholds[1][j];
					}
					else if (curr_mass == 3)
					{
						curr_threshold = thresholds[2][j];
					}
					else if (curr_mass == 5)
					{
						curr_threshold = thresholds[3][j];
					}


					if ((abs(torque_difference[j]) > curr_threshold)&&(abs(currVelo[j])< 0.04))
					{

						if (torque_difference[j] > 0)
						{
							dsrPos[j] = dsrPos[j] + add_trq[j]*abs(torque_difference[j]);
						}
						else
						{
							dsrPos[j] = dsrPos[j] - add_trq[j]*abs(torque_difference[j]);
						}
					}



			//	}

		}


	}

	/***************************************************************************************************************************************************/




    if (cmdJntPos_Port.connected())
    {
    	cmdJntPos_Port.write(dsrPos);
    }


}

ORO_LIST_COMPONENT_TYPE(ELMComponent);
ORO_CREATE_COMPONENT_LIBRARY()
