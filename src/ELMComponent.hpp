/*
 * ELMComponent.hpp
 *
 *  Created on: Jun 13, 2016
 *      Author: abalayn
 */

#ifndef SRC_ELMCOMPONENT_HPP_
#define SRC_ELMCOMPONENT_HPP_

#include <fstream>
#include <iostream>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

#include "RealVector.h"
#include "ExtremeLearningMachine.h"

class ELMComponent: public RTT::TaskContext {
public:
	ELMComponent(std::string const& name);
	 bool configureHook();
	 bool startHook();
	 void updateHook();
	 void stopHook();
	 void cleanupHook();

	 int nb_joints;

	 RTT::OutputPort<std::vector<double>> cmdJntPos_Port;

	 RTT::InputPort<std::vector<double>> trgtJntPos_Port; // Target value.
	 RTT::FlowStatus trgtJntPos_Flow;

	 RTT::InputPort<std::vector<double>> currJntPos_Port; // Target value.
	 RTT::FlowStatus currJntPos_Flow;

	 RTT::InputPort<std::vector<double>> meanJntTrq_Port; // Target value.
	 RTT::FlowStatus meanJntTrq_Flow;

	 RTT::InputPort<std::vector<double>> currVelocity_Port; // Target value.
	 RTT::FlowStatus currVelocity_Flow;

	 // ELM Learner
	 ExtremeLearningMachinePtr elm;
	 std::vector<double> torque_difference;
	 std::string infile;
	 std::ofstream error_file;

	 // Compliancy
	 std::vector<double> trgtPos;
	 std::vector<double> dsrPos;
	 std::vector<double> currPos;
	 std::vector<double> meanTrq;
	 std::vector<double> currVelo;


	 // ELM Learner

	 	// Several ELM initialized to consider different payload at the end effector

	 	ExtremeLearningMachinePtr elm_0;
	 	ExtremeLearningMachinePtr elm_1;
	 	ExtremeLearningMachinePtr elm_3;
	 	ExtremeLearningMachinePtr elm_5;

	 	std::vector<double> payload_error; // To compare the differences between current torque and awaited torque or different payloads.
	 	std::vector<double> payload_index; // contains payloads in correct order to find current payload.


	 	// Variable to save intermediate robot position - to decide if the data will be written in the file.
	 	std::vector< std::vector<double> > inter_torque_elm;

	 	// Compliance
	 	std::vector< std::vector<double> > thresholds;
	 	std::vector<double> jnt_effort;
	 	std::vector<double> add_trq; // Torque which is added when difference overshoots the threshold.
	 	bool wait;
	 	bool temp_wait; // Used because all joints must be updated before waiting.
	 	int nb_wait;
	 	int wait_step;
	 	int elm_step;
	 	int mean_trq_step;
	 	int comp_wait;
		int elm_id;
		int nb_step;
		bool new_pos;

		double curr_mass;
		RTT::InputPort<double> currMass_Port;
		RTT::FlowStatus currMass_Flow;




};

#endif /* SRC_ELMCOMPONENT_HPP_ */
