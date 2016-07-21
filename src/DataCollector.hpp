/*
 * DataCollector.hpp
 *
 *  Created on: Jul 21, 2016
 *      Author: abalayn
 */

#ifndef SRC_DATACOLLECTOR_HPP_
#define SRC_DATACOLLECTOR_HPP_


#include <fstream>
#include <iostream>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

class DataCollector: public RTT::TaskContext{
public:
	DataCollector(std::string const& name);
	bool configureHook();
	void updateHook();

private:
	double new_mass;
	std::vector<double> currPos;
	std::vector<double> currTrq;
	RTT::OutputPort<double> newMass_Port;
	RTT::OutputPort<std::vector<double>> trgtJntPos_Port;
	RTT::InputPort<std::vector<double>> currJntPos_Port;
	RTT::FlowStatus currJntPos_Flow;
	RTT::InputPort<std::vector<double>> meanCurrJntTrq_Port;
	RTT::FlowStatus meanCurrJntTrq_Flow;
	int nb_joints;
	std::ofstream data_file;

	int sim_id;
	std::vector<double> jnt_it;
	std::vector<double> jnt_width; // Shouldn't forget to modify it for joint 2!!
	std::vector<double> target_value;
	std::vector<double> nb_recording;

	double l1;
	double l2;

	bool random_pos;
};

#endif /* SRC_DATACOLLECTOR_HPP_ */
