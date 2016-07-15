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

ELMComponent::ELMComponent(std::string const& name) : RTT::TaskContext(name) , nb_joints(6) , currJntPos_Flow(RTT::NoData) , currJntTrq_Flow(RTT::NoData) {
	// TODO Auto-generated constructor stub



	this->addPort("cmdJntPos", cmdJntPos_Port);
	currPos = {0 , 0 ,0 ,0 ,0 , 0};
	dsrPos = {0 , 0 ,0 ,0 ,0 , 0};

	cmdJntPos_Port.setDataSample(currPos);

	this->addPort("currJntTrq", currJntTrq_Port);

	this->addPort("currJntPos" , currJntPos_Port);
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
			thresholds.push_back(0);
			torque_difference.push_back(0);
			currTrq.push_back(0);
		}
		thresholds[0] = 10;
		thresholds[1] = 10;
		thresholds[2] = 10;
		thresholds[3] = 10;
		thresholds[4] = 10;
		thresholds[5] = 10;

		// ELM model creation.
		std::string infile("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/elmmodel/data");
		elm=ExtremeLearningMachine::create(infile);
		RTT::log(RTT::Warning)  << "Generating testdata with dimensionality: " << elm->getInputDimension() << RTT::endlog();
		RTT::log(RTT::Warning) << "Expecting results with dimensionality: " << elm->getOutputDimension() << RTT::endlog();
		RTT::log(RTT::Error) << "ELM loaded" << RTT::endlog();


		error_file.open("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/error_data.txt");
		if (!error_file)
			RTT::log(RTT::Error) << "The file could not be open." << RTT::endlog();
		RTT::log(RTT::Error) << "The error file is open." << RTT::endlog();


		RTT::log(RTT::Error) << "ELM Component configured." << RTT::endlog();

		return true;

}

void ELMComponent::updateHook(){
//	RTT::log(RTT::Error) << "Beginning ELMComponent update." << RTT::endlog();

	currJntPos_Flow = currJntPos_Port.read(currPos);
	currJntTrq_Flow = currJntTrq_Port.read(currTrq);


	/***************************************************************************************************************************************************/

	/*
	 * ELM part:
	 * Computing the difference between current torque and awaited torque for each joint.
	 */


	// Create vector containing awaited position.
	RealVectorPtr inputdata = RealVector::create(elm->getInputDimension(), 0.0);
	for (int j=0; j<inputdata->getDimension(); j++) inputdata->setValueEquals(j,currPos[j]);

	// Create vector containing the torque which should be applied.
	RealVectorPtr result = elm->evaluate(inputdata);
	error_file << "{" ;
	for (unsigned j = 0; j < nb_joints; j++)
	{
		torque_difference[j] = result->getValue(j) - currTrq[j];
		error_file << "joint " << j << ": " << torque_difference[j] << " dsrTrq: " << result->getValue(j) << " realTrq: " << currTrq[j] << " ;";
	}
	error_file << "}" << std::endl;
	/***************************************************************************************************************************************************/



	/***************************************************************************************************************************************************/
	/*
	 * Compliance of the robot.
	 */

	/*
	for (unsigned j = 0; j < nb_joints; j++)
	{
		if (torque_difference[j] > thresholds[j])
			dsrPos[j] = currPos[j];
	}
	*/
	/***************************************************************************************************************************************************/




    if (cmdJntPos_Port.connected()) {
    	cmdJntPos_Port.write(dsrPos);
    }
	//RTT::log(RTT::Error) << "P: Torque command sent." << RTT::endlog();


//    RTT::log(RTT::Error) << "ELMComponent updated." << RTT::endlog();
}

ORO_LIST_COMPONENT_TYPE(ELMComponent);
ORO_CREATE_COMPONENT_LIBRARY()
