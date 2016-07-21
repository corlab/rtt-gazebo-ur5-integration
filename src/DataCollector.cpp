/*
 * DataCollector.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: abalayn
 */

#include "DataCollector.hpp"
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <math.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace std;

DataCollector::DataCollector(std::string const& name): RTT::TaskContext(name), sim_id(0), nb_recording(0), jnt_it(0), random_pos(true), nb_joints(6), new_mass(0), currJntPos_Flow(RTT::NoData) , meanCurrJntTrq_Flow(RTT::NoData) , currPos(0) , currTrq(0) {

	l1 = 0.7;//0.42500;//0.7; // find real values later !!
	l2 = 0.9;//0.39225;//0.9;// find real values later !!


	this->addPort("newMass", newMass_Port);
	new_mass = 0.0;
	newMass_Port.setDataSample(new_mass);
	this->addPort("trgtJntPos" , trgtJntPos_Port);
	target_value = { 0 , -0.1 , 3.14 - (+ -0.1 + acos(sin(0.1)*l1/l2) + 1.57) - 0.3 -0.4 , -3.14 , -1.4 , -1.57};
	trgtJntPos_Port.setDataSample(target_value);

	this->addPort("currJntPos" , currJntPos_Port);
	this->addPort("meanCurrJntTrq" , meanCurrJntTrq_Port);

}


bool DataCollector::configureHook(){
	RTT::log(RTT::Error) << "Beginning DataCollector configuration." << RTT::endlog();

	new_mass = 1; // Set the same as the rttgazebo componenent !



	for (unsigned j = 0; j < nb_joints; j++)
	{
		currPos.push_back(0);
		currTrq.push_back(0);
		nb_recording.push_back(0);
		jnt_it.push_back(1);
		jnt_width.push_back(0);
	}


	nb_recording[0] = 4.0;
	nb_recording[1] = 4.0;
	nb_recording[2] = 4.0;
	nb_recording[3] = 4.0;
	nb_recording[4] = 4.0;
	nb_recording[5] = 4.0;

	jnt_width[0] = 6.28;
	jnt_width[1] = abs(-2.3 - -0.1);
	jnt_width[2] = abs(3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));//abs(3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));
	jnt_width[3] = 0.7 - -3.14;
	jnt_width[4] = 1.57 - -1.4;
	jnt_width[5] = 3.14 - -1.57;

	jnt_it[1] = -1;
	jnt_it[2] = -1;

	data_file.open("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/test_data.txt");
	if (!data_file)
		RTT::log(RTT::Error) << "The file could not be open." << RTT::endlog();

	RTT::log(RTT::Error) << "DataCollector configured." << RTT::endlog();

	return true;
}

void DataCollector::updateHook(){
//	RTT::log(RTT::Error) << target_value[2] << RTT::endlog();

	sim_id ++;
	if (currJntPos_Port.connected())
		currJntPos_Flow = currJntPos_Port.read(currPos);
	if (meanCurrJntTrq_Port.connected())
		meanCurrJntTrq_Flow = meanCurrJntTrq_Port.read(currTrq);

	if (meanCurrJntTrq_Flow == RTT::NewData)
	{
		RTT::log(RTT::Error) << "New data received" << RTT::endlog();

		// Data writing

		data_file << "{ sim_id = " << sim_id << " ; mass = " << new_mass << " ; ";
		for (unsigned j = 0; j < nb_joints; j++)
		{
			data_file << "jnt " << j << " ; ";
			data_file << "trq "<< currTrq[j] << " ; ";
			data_file << "agl "	<< currPos[j] << " ; ";
			data_file << "trg_agl "	<<0 << " ; "; // Target angle is useless here (therefore set to 0 because of the file parser set in matlab)
		}
		data_file << " }" << std::endl;


		// Payload and new positions send


		new_mass = 5 + ((double)rand() / RAND_MAX)*( 5); // Between 5 and 10 kg.
		if (newMass_Port.connected())
		{
			newMass_Port.write(new_mass);
			RTT::log(RTT::Error) << "New mass set" << RTT::endlog();
		}

		if ((jnt_it[5]) < nb_recording[5])
		{
			jnt_it[5] = jnt_it[5]+1;

			if (random_pos)
			{
				target_value[5] = jnt_it[5]*jnt_width[5]/nb_recording[5] + (jnt_width[5]/nb_recording[5]) * ((((float) rand()) / (float) RAND_MAX)) -1.57;
			}
			else
			{
				target_value[5] = jnt_it[5]*(jnt_width[5]/nb_recording[5]) -1.57;
			}
		}
		else
		{

			if ((jnt_it[4]) < nb_recording[4])
			{
				if (random_pos)
					target_value[4] = jnt_it[4]*jnt_width[4]/nb_recording[4] + (jnt_width[5]/nb_recording[4]) * ((((float) rand()) / (float) RAND_MAX)) -1.4;
				else
					target_value[4] = jnt_it[4]*jnt_width[4]/nb_recording[4] -1.4;

				jnt_it[4]++;

			}
			else
			{
				if ( jnt_it[3] <nb_recording[3])
				{
					if (random_pos)
						target_value[3] = jnt_it[3]*jnt_width[3]/nb_recording[3] + (jnt_width[3]/nb_recording[3]) * ((((float) rand()) / (float) RAND_MAX)) -3.14;
					else
						target_value[3] = jnt_it[3]*jnt_width[3]/nb_recording[3] -3.14;
					jnt_it[3]++;
				}
				else
				{
					if (( jnt_it[2] > -(nb_recording[2]))&&(target_value[1] < 1.57))
					{
						if (random_pos)
							target_value[2] = jnt_it[2]*jnt_width[2]/nb_recording[2] + (jnt_width[2]/nb_recording[2]) * ((((float) rand()) / (float) RAND_MAX)) + 3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
						else
							target_value[2] = jnt_it[2]*jnt_width[2]/nb_recording[2]  + 3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
						jnt_it[2]--;
					}
					else if (( jnt_it[2] > -(nb_recording[2]))&&(target_value[1] > 1.57))
					{
						if (random_pos)
							target_value[2] = -jnt_it[2]*jnt_width[2]/nb_recording[2] - (jnt_width[2]/nb_recording[2]) * ((((float) rand()) / (float) RAND_MAX)) -3.14 + (- target_value[1] + 1.7 - acos(cos(-target_value[1]+1.7)*l1/l2)) + 0.3 +0.4;
						else
							target_value[2] = -jnt_it[2]*jnt_width[2]/nb_recording[2]  -3.14 + (- target_value[1] + 1.7 - acos(cos(-target_value[1]+1.7)*l1/l2)) + 0.3 +0.4;
						jnt_it[2]--;
					}
					else
					{
						if ((jnt_it[1] ) > -(nb_recording[1]))
						{
							if (random_pos)
								target_value[1] = jnt_it[1]*jnt_width[1]/nb_recording[1] - (jnt_width[1]/nb_recording[1]) * ((((float) rand()) / (float) RAND_MAX)) -0.1;
							else
								target_value[1] = jnt_it[1]*jnt_width[1]/nb_recording[1] -0.1;

							jnt_width[2] = abs(3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));
							jnt_it[1]--;
						}
						else
						{
							target_value[1] = -0.1;
							jnt_it[1] = -1;
							jnt_width[2] = abs(3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));

							if (jnt_it[0] >= nb_recording[0])
							{
								target_value[0] = 0;
								jnt_it[0] = 1;


							}
							else
							{
								if (random_pos)
									target_value[0] = jnt_it[0]*jnt_width[0]/nb_recording[0] + (jnt_width[0]/nb_recording[0]) * ((((float) rand()) / (float) RAND_MAX));
								else
									target_value[0] = jnt_it[0]*jnt_width[0]/nb_recording[0];
								jnt_it[0]++;
							}
						}
						if (target_value[1] < 1.57)
							target_value[2] = 3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
						else
							target_value[2] = -3.14 + (- target_value[1] + 1.7 - acos(cos(-target_value[1]+1.7)*l1/l2)) + 0.3 +0.4;
						jnt_it[2] = -1;
					}
					target_value[3] = -3.14;
					jnt_it[3] = 1;
				}
				target_value[4] = -1.4;
				jnt_it[4] = 1;
			}
			target_value[5] = -1.57;
			jnt_it[5] = 1;
		}





		if (trgtJntPos_Port.connected())
		{
			trgtJntPos_Port.write(target_value);
			RTT::log(RTT::Error) << "New data sent" << RTT::endlog();
		}

	}


}

ORO_LIST_COMPONENT_TYPE(DataCollector)
ORO_CREATE_COMPONENT_LIBRARY();
