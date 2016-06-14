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

	 RTT::InputPort<std::vector<double>> currJntPos_Port; // Target value.
	 RTT::FlowStatus currJntPos_Flow;

	 RTT::InputPort<std::vector<double>> currJntTrq_Port;
	 RTT::FlowStatus currJntTrq_Flow;
	 std::vector<double> currTrq;

	 // ELM Learner
	 ExtremeLearningMachinePtr elm;
	 std::vector<double> torque_difference;
	 std::string infile;
	 std::ofstream error_file;

	 // Compliancy
	 std::vector<double> thresholds;
	 std::vector<double> currPos;
	 std::vector<double> dsrPos;


};

#endif /* SRC_ELMCOMPONENT_HPP_ */
