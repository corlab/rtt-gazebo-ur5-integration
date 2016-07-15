#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

//#include <Eigen/Dense>
#include <boost/graph/graph_concepts.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt/os/TimeService.hpp>
#include <boost/thread/mutex.hpp>

#include <rci/dto/JointAngles.h>
#include <rci/dto/JointTorques.h>
#include <rci/dto/JointVelocities.h>

#include <nemo/Vector.h>
#include <nemo/Mapping.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <math.h>
#include <numeric>

#include "RealVector.h"
#include "ExtremeLearningMachine.h"

using namespace std;

#define l(lvl) RTT::log(lvl) << "[" << this->getName() << "] "



class UR5RttGazeboComponent: public RTT::TaskContext {
public:



	// BEST  Kp({2700 , 2700  , 2700 , 2700 , 2700 , 2700 }) , Ki({8.7 , 8.7  , 8.7  , 8.7  , 8.7  , 8.7 }) , Kd({209250 ,209250 , 209250 , 209250 , 209250 , 209250})

	UR5RttGazeboComponent(std::string const& name) :
	/*	RTT::TaskContext(name), nb_static_joints(0) , jnt_it(0) , jnt_width(0) , nb_links(0), inter_torque({{0} , {0} , {0} , {0} , {0} , {0}}) , jnt_effort(0),  torque_difference(0), control_value(0) , target_value(0), error_value(0),errorI(0), cumulative_error(0), error_derivative(0), last_error(0), dynStepSize(5) , pid_it(5) ,Kp({2000 , 3000  , 3000 , 540 , 540 , 1000 }) , Ki({0.4 , 0.4 , 0.4 , 0.2 , 0.2 , 0.4 }) , Kd({41850 ,41850 , 41850 , 41850 , 41850 , 37850}) , Ks({0,0,0,0,0,0}) */// HACK: The urdf has static tag for base_link, which makes it appear in gazebo as a joint.
			RTT::TaskContext(name), nb_static_joints(0), trq_err_1(0) , trq_err_2(0) , trq1(0) , trq2(0), nearest_mass(0) , comp_wait(10000), curr_mass(0.0) ,wait(false),temp_wait(false), nb_wait(0), nb_measure_trq(20), wait_step(600), mean_trq_step(200), elm_step(200), ee_mass({5, 0.00001 , 1 , 3 , 4 , 2}) , mass_id(0) , elm_id(0) , random_pos(true) , jnt_it(0) , jnt_width(0) , nb_links(0), inter_torque({{0} , {0} , {0} , {0} , {0} , {0}}) ,add_trq(0), inter_torque_elm({{0} , {0} , {0} , {0} , {0} , {0}}) ,thresholds({{0} , {0} , {0} , {0} }) ,nb_recording(0), jnt_effort(0),  torque_difference(0) , torque_difference_0(0) , torque_difference_1(0), torque_difference_3(0), torque_difference_5(0), payload_index({0.00001 , 1 , 3 , 5}), payload_error(0), control_value(0) , target_value(0), error_value(0),errorI(0), cumulative_error(0), error_derivative(0), last_error(0), dynStepSize(5) , pid_it(5) ,Kp({2000 , 3000  , 3000 , 540 , 540 , 70 }) , Ki({0.4 , 0.4 , 0.4 , 0.0 , 0.0 , 0.4 }) , Kd({41850 ,41850 , 11850 , 41850 , 51850 , 2850}) , Ks({0,0,0,0,0,0}) // HACK: The urdf has static tag for base_link, which makes it appear in gazebo as a joint.
			 {
		// Add required gazebo interfaces.
		this->provides("gazebo")->addOperation("configure",
				&UR5RttGazeboComponent::gazeboConfigureHook, this,
				RTT::ClientThread);
		this->provides("gazebo")->addOperation("update",
				&UR5RttGazeboComponent::gazeboUpdateHook, this, RTT::ClientThread);

		nb_iteration = 0;
		sim_id = 1;

		l1 = 0.7;//0.42500;//0.7; // find real values later !!
		l2 = 0.9;//0.39225;//0.9;// find real values later !!

		}


	double constrainCommand(double eff_max , double eff_min, double command)
		{
			if (command >= eff_max)
				return eff_max;
			else if (command <= eff_min)
				return eff_min;
			else
				return command;
		}

	void eeMass(double mass , gazebo::physics::ModelPtr model)
	{
		// Code to change mass and inertia tensor at the end effector during the simulation. // Here mass set to 1 for data recording.
		RTT::log(RTT::Error) << "Model modification." << RTT::endlog();
		auto inertial = model->GetLinks()[links_idx[nb_links-1]]->GetInertial();
		RTT::log(RTT::Error) << "Inertia pointer prepared." << RTT::endlog();
		inertial->SetMass(mass);
		double inertia_value;
		inertia_value = (mass*0.05*0.05)/6;
		RTT::log(RTT::Error) << "Mass set to " << mass << RTT::endlog();
		inertial->SetInertiaMatrix(inertia_value, inertia_value, inertia_value, 0, 0, 0);
		RTT::log(RTT::Error) << "Inertia matrix prepared." << RTT::endlog();
		model_links_[links_idx[nb_links-1]]->SetInertial(inertial);
		RTT::log(RTT::Error) << "Inertia matrix set." << RTT::endlog();
		model_links_[links_idx[nb_links-1]]->UpdateMass();
		RTT::log(RTT::Error) << "Inertia set to model. " << RTT::endlog();
	}


	//! Called from gazebo
	virtual bool gazeboConfigureHook(gazebo::physics::ModelPtr model) {
		if (model.get() == NULL) {
			RTT::log(RTT::Error) << "No model could be loaded" << RTT::endlog();
			std::cout << "No model could be loaded" << RTT::endlog();
			return false;
		}


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


		// Get the joints
		gazebo_joints_ = model->GetJoints();
		model_links_ = model->GetLinks(); // Only working when starting gzserver and gzclient separately!

		RTT::log(RTT::Warning) << "Model has " << gazebo_joints_.size()
				<< " joints and " << model_links_.size()<< " links"<< RTT::endlog();

		//NOTE: Get the joint names and store their indices
		// Because we have base_joint (fixed), j0...j6, ati_joint (fixed)
		int idx = 0;
		for (gazebo::physics::Joint_V::iterator jit = gazebo_joints_.begin();
				jit != gazebo_joints_.end(); ++jit, ++idx) {

			const std::string name = (*jit)->GetName();
			// NOTE: Remove fake fixed joints (revolute with upper==lower==0
			// NOTE: This is not used anymore thanks to <disableFixedJointLumping>
			// Gazebo option (ati_joint is fixed but gazebo can use it )

			if ((*jit)->GetLowerLimit(0u) == (*jit)->GetUpperLimit(0u)) {
				RTT::log(RTT::Warning) << "Not adding (fake) fixed joint ["
						<< name << "] idx:" << idx << RTT::endlog();
				continue;
			}
			joints_idx.push_back(idx);
			joint_names_.push_back(name);
			RTT::log(RTT::Warning) << "Adding joint [" << name << "] idx:"
					<< idx << RTT::endlog();
			std::cout << "Adding joint [" << name << "] idx:" << idx
					<< RTT::endlog();
		}

		if (joints_idx.size() == 0) {
			RTT::log(RTT::Error) << "No Joints could be added, exiting"
					<< RTT::endlog();
			return false;
		}



		idx = 0;
		for (gazebo::physics::Link_V::iterator lit = model_links_.begin();
				lit != model_links_.end(); ++lit, ++idx) {

			const std::string name = (*lit)->GetName();
			links_idx.push_back(idx);
			link_names_.push_back(name);
			RTT::log(RTT::Warning) << "Adding link [" << name << "] idx:"
					<< idx << RTT::endlog();
			nb_links++;
		}



		if (links_idx.size() == 0) {
			RTT::log(RTT::Error) << "No links could be added, exiting"
					<< RTT::endlog();
			return false;
		}




		RTT::log(RTT::Warning) << "Done configuring gazebo" << RTT::endlog();

		for (unsigned j = 0; j < joints_idx.size(); j++)
		{
			gazebo_joints_[joints_idx[j]]->SetProvideFeedback(true);

			error_value.push_back(0);
			cumulative_error.push_back(0);
			last_error.push_back(0);
			errorI.push_back(0);
			error_derivative.push_back(0);
			control_value.push_back(0);
			target_value.push_back(0);
			supp_target_value.push_back(0);
			torque_difference.push_back(0);
			torque_difference_0.push_back(0);
			torque_difference_1.push_back(0);
			torque_difference_3.push_back(0);
			torque_difference_5.push_back(0);
			jnt_it.push_back(1);
			jnt_width.push_back(0);
			thresholds[0].push_back(0);
			thresholds[1].push_back(0);
			thresholds[2].push_back(0);
			thresholds[3].push_back(0);
			jnt_effort.push_back(0);
			nb_recording.push_back(0);
			add_trq.push_back(0);


		}
		RTT::log(RTT::Warning) << "Done configuring PIDs" << RTT::endlog();

		data_file.open("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/test_data.txt");
		if (!data_file)
			RTT::log(RTT::Error) << "The file could not be open." << RTT::endlog();
		error_file.open("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/error_data.txt");
		if (!error_file)
			RTT::log(RTT::Error) << "The file could not be open." << RTT::endlog();

		sensibility_file.open("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/sensibility_data.txt");
		if (!sensibility_file)
			RTT::log(RTT::Error) << "The file could not be open." << RTT::endlog();


//For recording data
//	target_value[0] = 5.6;//0;
//	target_value[1] = -0.1;
//	target_value[2] =  3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
//	target_value[3] = -3.14;
//	target_value[4] = -1.4;
//	target_value[5] = -1.57;

//For testing positions
	target_value[0] = 0.1;
	target_value[1] = -0.1;//-0.1;//-1.15;//-0.3;
	target_value[2] = 3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;//0.2;//0.7;//0.2;
	target_value[3] = -1.4064;
	target_value[4] = -1.5318;
	target_value[5] = -0.3283;

//		//For testing other positions
//			target_value[0] = 0.1;
//			target_value[1] = -1.15;//-0.3;
//			target_value[2] = 0.2;//0.7;//0.2;
//			target_value[3] = -1.4064;
//			target_value[4] = -1.5318;
//			target_value[5] = -0.3283;
//

	//For recording data
		supp_target_value[0] = 0.1;
		supp_target_value[1] = -0.1;//-0.1;//-1.15;//-0.3;
		supp_target_value[2] =  3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;//0.2;//0.7;//0.2;
		supp_target_value[3] = -1.4064;
		supp_target_value[4] = -1.5318;
		supp_target_value[5] = -0.3283;




		jnt_width[0] = 6.28;
		jnt_width[1] = abs(-2.3 - -0.1);
		jnt_width[2] = abs(3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));//abs(3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));
		jnt_width[3] = 0.7 - -3.14;
		jnt_width[4] = 1.57 - -1.4;
		jnt_width[5] = 3.14 - -1.57;
		RTT::log(RTT::Warning) << "Done configuring robot position" << RTT::endlog();


		/*// To launch data recording from a different start point.
		jnt_it[0] = 2;

			target_value[0] =  jnt_it[0]*jnt_width[0]/5 + (jnt_width[0]/5) * ((((float) rand()) / (float) RAND_MAX)); //0 ;
			RTT::log(RTT::Warning) << "Test trgt value 0: " <<  target_value[0] << RTT::endlog();
		 */

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


		jnt_effort[0] = 150;
		jnt_effort[1] = 150;
		jnt_effort[2] = 150;
		jnt_effort[3] = 28;
		jnt_effort[4] = 28;
		jnt_effort[5] = 28;

		nb_recording[0] = 4.0;
		nb_recording[1] = 4.0;
		nb_recording[2] = 4.0;
		nb_recording[3] = 4.0;
		nb_recording[4] = 4.0;
		nb_recording[5] = 4.0;

		jnt_it[1] = -1;
		jnt_it[2] = -1;

	//	jnt_it[1] = -5;
		jnt_it[0] = 5;

		nb_wait = wait_step;

		curr_mass = 0.0001;
		// To test guessing of the payload.
		eeMass(curr_mass, model);

		//eeMass(0.00001 , model);
		mass_id = 4; //  For data recording.

		nearest_mass = 0;

		RTT::log(RTT::Warning) << "Configure hook finished." << RTT::endlog();

		return true;
	}

	//! Called from Gazebo
	virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model) {
		if (model.get() == NULL) {
			return;
		}
		nb_iteration++;


		/***************************************************************************************************************************************************/

		/*
		 * Data recording part
		*/

//	if ((nb_iteration == 3410) || (nb_iteration == 3420) || (nb_iteration == 3430) || (nb_iteration == 3440) || (nb_iteration == 3450) || (nb_iteration == 3460) || (nb_iteration == 3470) || (nb_iteration == 3480) || (nb_iteration == 3490) || (nb_iteration == 3500) ) // To check if position is stable.
//	{
//		for (unsigned j = 0; j < joints_idx.size(); j++)
//		{
//			gazebo::physics::JointWrench w1 = gazebo_joints_[joints_idx[j]]->GetForceTorque(0u);
//			gazebo::math::Vector3 a1 = gazebo_joints_[joints_idx[j]]->GetLocalAxis(0u);
//			inter_torque[j].push_back(a1.Dot(w1.body1Torque));
//		}
//	}
//
//
//
//	if (nb_iteration >= 3500) // For stabilisation of the torque.
//	{
//
//
//			data_file << "{ sim_id = " << sim_id << " ; mass = " << curr_mass << " ; ";
//			double mean_of_torques = 0;
//			for (unsigned j = 0; j < joints_idx.size(); j++)
//			{
//				data_file << "jnt " << j << " ; ";
//
//				// Computing the mean of the torques.
//
//				mean_of_torques = (std::accumulate((inter_torque[j]).begin(),(inter_torque[j]).end(), 0.0))/10.0;
//
//				data_file << "trq "<< mean_of_torques << " ; ";
//
//				data_file << "agl "	<< model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian() << " ; ";
//				data_file << "trg_agl "	<<target_value[j] << " ; ";
//				mean_of_torques = 0;
//				inter_torque[j] = {0};
//			}
//			data_file << " }" << std::endl;
//
//		nb_iteration = 0;
//
//
//
//			curr_mass = 5 + ((double)rand() / RAND_MAX)*( 5); // Between 5 and 10 kg.
//			eeMass(curr_mass, model);
//
//	// Changes desired position  of each joint.
//			if ((jnt_it[5]) < nb_recording[5])
//			{
//				jnt_it[5] = jnt_it[5]+1;
//
//				if (random_pos)
//				{
//					target_value[5] = jnt_it[5]*jnt_width[5]/nb_recording[5] + (jnt_width[5]/nb_recording[5]) * ((((float) rand()) / (float) RAND_MAX)) -1.57;
//				}
//				else
//				{
//					target_value[5] = jnt_it[5]*(jnt_width[5]/nb_recording[5]) -1.57;
//				}
//			}
//			else
//			{
//
//				if ((jnt_it[4]) < nb_recording[4])
//				{
//					if (random_pos)
//						target_value[4] = jnt_it[4]*jnt_width[4]/nb_recording[4] + (jnt_width[5]/nb_recording[4]) * ((((float) rand()) / (float) RAND_MAX)) -1.4;
//					else
//						target_value[4] = jnt_it[4]*jnt_width[4]/nb_recording[4] -1.4;
//
//					jnt_it[4]++;
//
//				}
//				else
//				{
//					if ( jnt_it[3] <nb_recording[3])
//					{
//						if (random_pos)
//							target_value[3] = jnt_it[3]*jnt_width[3]/nb_recording[3] + (jnt_width[3]/nb_recording[3]) * ((((float) rand()) / (float) RAND_MAX)) -3.14;
//						else
//							target_value[3] = jnt_it[3]*jnt_width[3]/nb_recording[3] -3.14;
//						jnt_it[3]++;
//					}
//					else
//					{
//						if (( jnt_it[2] > -(nb_recording[2]))&&(target_value[1] < 1.57))
//						{
//							if (random_pos)
//								target_value[2] = jnt_it[2]*jnt_width[2]/nb_recording[2] + (jnt_width[2]/nb_recording[2]) * ((((float) rand()) / (float) RAND_MAX)) + 3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
//							else
//								target_value[2] = jnt_it[2]*jnt_width[2]/nb_recording[2]  + 3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
//							jnt_it[2]--;
//						}
//						else if (( jnt_it[2] > -(nb_recording[2]))&&(target_value[1] > 1.57))
//						{
//							if (random_pos)
//								target_value[2] = -jnt_it[2]*jnt_width[2]/nb_recording[2] - (jnt_width[2]/nb_recording[2]) * ((((float) rand()) / (float) RAND_MAX)) -3.14 + (- target_value[1] + 1.7 - acos(cos(-target_value[1]+1.7)*l1/l2)) + 0.3 +0.4;
//							else
//								target_value[2] = -jnt_it[2]*jnt_width[2]/nb_recording[2]  -3.14 + (- target_value[1] + 1.7 - acos(cos(-target_value[1]+1.7)*l1/l2)) + 0.3 +0.4;
//							jnt_it[2]--;
//						}
//						else
//						{
//							if ((jnt_it[1] ) > -(nb_recording[1]))
//							{
//								if (random_pos)
//									target_value[1] = jnt_it[1]*jnt_width[1]/nb_recording[1] - (jnt_width[1]/nb_recording[1]) * ((((float) rand()) / (float) RAND_MAX)) -0.1;
//								else
//									target_value[1] = jnt_it[1]*jnt_width[1]/nb_recording[1] -0.1;
//
//								jnt_width[2] = abs(3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));
//								jnt_it[1]--;
//							}
//							else
//							{
//								target_value[1] = -0.1;
//								jnt_it[1] = -1;
//								jnt_width[2] = abs(3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));
//
//								if (jnt_it[0] >= nb_recording[0])
//								{
//									target_value[0] = 0;
//									jnt_it[0] = 1;
//                                //
//								//	mass_id++;
//								//	if (mass_id >= ee_mass.size())
//								//	{
//								//		mass_id = 0;
//								//	}
//								//	eeMass(ee_mass[mass_id] , model);
//								//	RTT::log(RTT::Error) << "Mass set to " << ee_mass[mass_id] << " kg. " << RTT::endlog();
//
//								}
//								else
//								{
//									if (random_pos)
//										target_value[0] = jnt_it[0]*jnt_width[0]/nb_recording[0] + (jnt_width[0]/nb_recording[0]) * ((((float) rand()) / (float) RAND_MAX));
//									else
//										target_value[0] = jnt_it[0]*jnt_width[0]/nb_recording[0];
//									jnt_it[0]++;
//								}
//							}
//							if (target_value[1] < 1.57)
//								target_value[2] = 3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
//							else
//								target_value[2] = -3.14 + (- target_value[1] + 1.7 - acos(cos(-target_value[1]+1.7)*l1/l2)) + 0.3 +0.4;
//							jnt_it[2] = -1;
//						}
//						target_value[3] = -3.14;
//						jnt_it[3] = 1;
//					}
//					target_value[4] = -1.4;
//					jnt_it[4] = 1;
//				}
//				target_value[5] = -1.57;
//				jnt_it[5] = 1;
//			}
//
//
//
//	}




		/***************************************************************************************************************************************************/


		/***************************************************************************************************************************************************/


	//	/*
	//	 * Video recording part
	//	 */
	//	/*
	//	if (sim_id < 5000)
	//	{
	//		target_value[0] = -1;
	//		target_value[1] = -0.7;
	//		target_value[2] = 1.3;
	//		target_value[3] = -2;
	//		target_value[4] = -1.4;
	//		target_value[5] = -1.57;
	//	}
	//	else if ((sim_id  > 5000) && (sim_id < 6500))
	//	{
	//
	//		target_value[2] = target_value[2] - 0.0006;
	//
	//	}
	//	else if ((sim_id  > 6500) && (sim_id < 10200))
	//	{
	//		target_value[0] = target_value[0] - 0.0006;
	//
	//	}
	//	else if ((sim_id  > 10200) && (sim_id < 11700))
	//	{
	//		target_value[2] = target_value[2] + 0.0006;
	//	}
	//
	//
	//	target_value[0] = -1;
	//	target_value[1] = -1.57;
	//	target_value[2] = 1.57;
	//	target_value[3] = -1.57;
	//	target_value[4] = -1.57;
	//	target_value[5] = -1.57;
	//	 */

//		/***************************************************************************************************************************************************/
//
//
//
//		/***************************************************************************************************************************************************/
//
	/*
	 * ELM part:
	 * Computing the difference between current torque and awaited torque for each joint.
	 */


	comp_wait = comp_wait - 1;
	elm_id++;

	if ((elm_id == (mean_trq_step-19*mean_trq_step/nb_measure_trq)) ||(elm_id == (mean_trq_step-18*mean_trq_step/nb_measure_trq)) ||(elm_id == (mean_trq_step-17*mean_trq_step/nb_measure_trq)) ||(elm_id == (mean_trq_step-16*mean_trq_step/nb_measure_trq)) ||(elm_id == (mean_trq_step-15*mean_trq_step/nb_measure_trq)) ||(elm_id == (mean_trq_step-14*mean_trq_step/nb_measure_trq)) ||(elm_id == (mean_trq_step-13*mean_trq_step/nb_measure_trq)) ||(elm_id == (mean_trq_step-12*mean_trq_step/nb_measure_trq)) ||(elm_id == (mean_trq_step-11*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-10*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-9*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-8*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-7*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-6*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-5*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-4*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-3*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-2*mean_trq_step/nb_measure_trq)) || (elm_id == (mean_trq_step-mean_trq_step/nb_measure_trq)) || (elm_id == mean_trq_step) ) // To check if position is stable.
	{
		for (unsigned j = 0; j < joints_idx.size(); j++)
		{
			gazebo::physics::JointWrench w1 = gazebo_joints_[joints_idx[j]]->GetForceTorque(0u);
			gazebo::math::Vector3 a1 = gazebo_joints_[joints_idx[j]]->GetLocalAxis(0u);
			inter_torque_elm[j].push_back(a1.Dot(w1.body1Torque));
		}
	}




	/***************************************************************************************************************************************************/



	/***************************************************************************************************************************************************/
	/*
	 * Compliance of the robot.
	 */









		//if ((sim_id % 2000)&&(elm_id >= 1000)&&(sim_id>14000))
	if (elm_id == mean_trq_step)
	{
		elm_id = 0;




					// ********************************************************************************************************************************
				// * ELM part:
				// * Computing the difference between current torque and awaited torque for each joint.
				 // Create vector containing awaited position.

		RealVectorPtr inputdata = RealVector::create(elm_0->getInputDimension(), 0.0);
		for (int j=0; j<inputdata->getDimension(); j++) inputdata->setValueEquals(j,target_value[j]/*model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian()*/);
		RealVectorPtr result;
		if (nearest_mass == 0)
		{
			// Create vector containing the torque which should be applied.
			result = elm_0->evaluate(inputdata);
		}
		else if (nearest_mass == 1)
		{
			// Create vector containing the torque which should be applied.
			result = elm_1->evaluate(inputdata);
		}
		else if (nearest_mass == 3)
		{
			// Create vector containing the torque which should be applied.
			result = elm_3->evaluate(inputdata);
		}
		else if (nearest_mass == 5)
		{
			// Create vector containing the torque which should be applied.
			result = elm_5->evaluate(inputdata);
		}


		error_file << "{" ;

		for (unsigned j = 0; j < joints_idx.size(); j++)
		{
			double mean_of_torques_elm = 0;
			mean_of_torques_elm = (std::accumulate((inter_torque_elm[j]).begin(),(inter_torque_elm[j]).end(), 0.0))/nb_measure_trq;

			torque_difference[j] = result->getValue(j) - mean_of_torques_elm;
			//RTT::log(RTT::Warning) << "Torque difference: " << torque_difference[j] << RTT::endlog();
			error_file << "joint " << j << ": " << torque_difference[j] << " dsrTrq: " << result->getValue(j) << " realTrq: " <<  mean_of_torques_elm << " ;";

				// ********************************************************************************************************************************


			// ********************************************************************************************************************************
				// * Compliance part: only every T seconds - to be decided.

			if ((sim_id >= 10000)&&(sim_id%elm_step == 0))
			{
			//	if (!wait)
			//	{
					double curr_threshold;
					if (nearest_mass == 0)
					{
						curr_threshold = thresholds[0][j];
					}
					else if (nearest_mass == 1)
					{
						curr_threshold = thresholds[1][j];
					}
					else if (nearest_mass == 3)
					{
						curr_threshold = thresholds[2][j];
					}
					else if (nearest_mass == 5)
					{
						curr_threshold = thresholds[3][j];
					}
				//	RTT::log(RTT::Warning) << "Joint " << j << "velocity " << model->GetJoints()[joints_idx[j]]->GetVelocity(0) << RTT::endlog();

					if (comp_wait == 0)
					{
						trq_err_1 = torque_difference[1];
						trq_err_2 = torque_difference[2];
						trq1 = control_value[1];
						trq2 = control_value[2];
					}

					if ((abs(torque_difference[j]) > curr_threshold)&&(abs(model->GetJoints()[joints_idx[j]]->GetVelocity(0))< 0.04)&&(comp_wait <= 0))
					{
					//	gazebo::physics::JointWrench w1 = gazebo_joints_[joints_idx[j]]->GetForceTorque(0u);
					//	gazebo::math::Vector3 a1 = gazebo_joints_[joints_idx[j]]->GetLocalAxis(0u);
					//	error_file << "{" ;
					//	error_file << "joint " << j << ": " << torque_difference[j] << " dsrTrq: " << result->getValue(j) << " realTrq: " << (a1.Dot(w1.body1Torque)) << " ;";
					//	error_file << "tgtPos" << target_value[j];
					// Test: the new target value is the current position. 	To be evaluated because there is no deformation compared to soft robots!
					//	target_value[j] = model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian();
					//	error_file << "newPos" << target_value[j];
					//	error_file << "}" << std::endl;



						// Test: the new target value: we add an angle: has to be set for each joint>
						if (torque_difference[j] > 0)
						{
							target_value[j] = target_value[j] + add_trq[j]*abs(torque_difference[j]);
						}
						else
						{
							target_value[j] = target_value[j] - add_trq[j]*abs(torque_difference[j]);
						}
						error_file << "joint " << j << "modified! ";
						error_file << "trgt_val " << target_value[j] << " prv_val " << supp_target_value[j] ;
						temp_wait = true;
						RTT::log(RTT::Warning) << "Joint " << j << "set to " << target_value[j] << RTT::endlog();
					}



			//	}



			}
			inter_torque_elm[j] = {0};
			mean_of_torques_elm = 0;
		}
		if (temp_wait)
		{
			wait = true;
			temp_wait = false;
		}
		error_file << "trgt_val5 " << target_value[5] << " prv_val " << supp_target_value[2] ;
		error_file << "trgt_val4 " << target_value[4] << " prv_val " << supp_target_value[1] ;
		error_file << "trgt_val3 " << target_value[3] << " prv_val " << supp_target_value[0] ;
		error_file << "trgt_val2 " << target_value[2] << " prv_val " << supp_target_value[2] ;
		error_file << "trgt_val1 " << target_value[1] << " prv_val " << supp_target_value[1] ;
		error_file << "trgt_val0 " << target_value[0] << " prv_val " << supp_target_value[0] ;
				error_file << "}" << std::endl;

	}

	if (wait)
	{
		nb_wait = nb_wait-1;
		if (nb_wait<=0)
		{
			wait = false;
			nb_wait = wait_step;
		}
}


		/***************************************************************************************************************************************************/



		/***************************************************************************************************************************************************/

	/*
	 *  Try to guess which payload is currently at the end-effector.
	 */


//
//	if ((sim_id == 3410) || (sim_id == 3420) || (sim_id == 3430) || (sim_id == 3440) || (sim_id == 3450) || (sim_id == 3460) || (sim_id == 3470) || (sim_id == 3480) || (sim_id == 3490) || (sim_id == 3500) ) // To check if position is stable.
//	{
//		for (unsigned j = 0; j < joints_idx.size(); j++)
//		{
//			gazebo::physics::JointWrench w1 = gazebo_joints_[joints_idx[j]]->GetForceTorque(0u);
//			gazebo::math::Vector3 a1 = gazebo_joints_[joints_idx[j]]->GetLocalAxis(0u);
//			inter_torque[j].push_back(a1.Dot(w1.body1Torque));
//		}
//	}
//
//	if (sim_id == 3500)
//	{
//		RealVectorPtr inputdata = RealVector::create(elm_0->getInputDimension(), 0.0);
//		for (int j=0; j<inputdata->getDimension(); j++) inputdata->setValueEquals(j,model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian());
//		// Create vector containing the torque which should be applied.
//		RealVectorPtr result_0 = elm_0->evaluate(inputdata);
//		RealVectorPtr result_1 = elm_1->evaluate(inputdata);
//		RealVectorPtr result_3 = elm_3->evaluate(inputdata);
//		RealVectorPtr result_5 = elm_5->evaluate(inputdata);
//
//
//		for (unsigned j = 0; j < joints_idx.size(); j++)
//		{
//			double mean_of_torques = 0;
//			mean_of_torques= (std::accumulate((inter_torque[j]).begin(),(inter_torque[j]).end(), 0))/10.0;
//			torque_difference_0[j] = result_0->getValue(j) - mean_of_torques;
//			torque_difference_1[j] = result_1->getValue(j) - mean_of_torques;
//			torque_difference_3[j] = result_3->getValue(j) - mean_of_torques;
//			torque_difference_5[j] = result_5->getValue(j) - mean_of_torques;
//
//			mean_of_torques = 0;
//			inter_torque[j] = {0};
//		}
//		double error_0 =  std::inner_product( torque_difference_0.begin(), torque_difference_0.end(), torque_difference_0.begin(), 0.0 );
//		double error_1 =  std::inner_product( torque_difference_1.begin(), torque_difference_1.end(), torque_difference_1.begin(), 0.0 );
//		double error_3 =  std::inner_product( torque_difference_3.begin(), torque_difference_3.end(), torque_difference_3.begin(), 0.0 );
//		double error_5 =  std::inner_product( torque_difference_5.begin(), torque_difference_5.end(), torque_difference_5.begin(), 0.0 );
//
//		payload_error.push_back(error_0);
//		payload_error.push_back(error_1);
//		payload_error.push_back(error_3);
//		payload_error.push_back(error_5);
//
//
//		// Find index of min value of the vector
//		std::size_t min_mass_idx = std::distance(payload_error.begin(), std::min_element(payload_error.begin(), payload_error.end()));
//		RTT::log(RTT::Warning) << "Payload is the nearest to " << payload_index[min_mass_idx] << RTT::endlog();
//
//	/*
//		if (std::min(error_0 , error_1) == error_0)
//		{
//			if (std::min(error_0 , error_5) == error_0)
//			{
//				RTT::log(RTT::Warning) << "Payload is the nearest to " << 0 << RTT::endlog();
//			}
//			else
//			{
//				RTT::log(RTT::Warning) << "Payload is the nearest to " << 5 << RTT::endlog();
//			}
//		}
//		else
//		{
//			if (std::min(error_1 , error_5) == error_1)
//			{
//				RTT::log(RTT::Warning) << "Payload is the nearest to " << 1 << RTT::endlog();
//			}
//			else
//			{
//				RTT::log(RTT::Warning) << "Payload is the nearest to " << 5 << RTT::endlog();
//			}
//		}
//	*/
//	}
//

		/***************************************************************************************************************************************************/




	/***************************************************************************************************************************************************/
/*
 * Sensibility computation
 * Mass set to 1 for the model used. Test from which mass it moves.
 */

//
//	if (sim_id%6000 == 0)
//	{
//		comp_wait = 3000;
//		bool modif = false;
//		for (unsigned j = 0; j < 3; j++)
//		{
//			if (supp_target_value[j] != target_value[j])
//			{
//				modif = true;
//			}
//		}
//		if (modif)
//		{
//			sensibility_file << "{ jnt 1: " << supp_target_value[1] << " ; jnt 2: " << supp_target_value[2] << " ; mass: " << curr_mass << " ; jnt1_trq: " << trq1 << " ; jnt2_trq: "<< trq2  << " ; jnt1_error: "<< trq_err_1 << " ; jnt2_error: "<< trq_err_2 << " }" << std::endl;
//
//
//			if (( jnt_it[2] > -(nb_recording[2]))&&(supp_target_value[1] < 1.57))
//			{
//				supp_target_value[2] = jnt_it[2]*jnt_width[2]/nb_recording[2]  + 3.14 - (+ supp_target_value[1] + acos(sin(-supp_target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
//							error_file 	 << "Joint 2 set to " << supp_target_value[2] << std::endl;
//
//				jnt_it[2]--;
//			}
//			else if (( jnt_it[2] > -(nb_recording[2]))&&(supp_target_value[1] > 1.57))
//			{
//				supp_target_value[2] = -jnt_it[2]*jnt_width[2]/nb_recording[2]  -3.14 + (- supp_target_value[1] + 1.7 - acos(cos(-supp_target_value[1]+1.7)*l1/l2)) + 0.3 +0.4;
//							error_file 	 << "Joint 2 set to " << supp_target_value[2] << std::endl;
//
//				jnt_it[2]--;
//			}
//			else
//			{
//				if ((jnt_it[1] ) > -(nb_recording[1]))
//				{
//					supp_target_value[1] = jnt_it[1]*jnt_width[1]/nb_recording[1] -0.1;
//
//					jnt_width[2] = abs(3.14 - (+ supp_target_value[1] + acos(sin(-supp_target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+supp_target_value[1] + acos(sin(-supp_target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));
//					jnt_it[1]--;
//				}
//				else
//				{
//					supp_target_value[1] = -0.1;
//					jnt_it[1] = -1;
//					jnt_width[2] = abs(3.14 - (+ supp_target_value[1] + acos(sin(-supp_target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -  (3.14 - (+supp_target_value[1] + acos(sin(-supp_target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8));
//				}
//				if (supp_target_value[1] < 1.57)
//					supp_target_value[2] = 3.14 - (+ supp_target_value[1] + acos(sin(-supp_target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
//				else
//					supp_target_value[2] = -3.14 + (- supp_target_value[1] + 1.7 - acos(cos(-supp_target_value[1]+1.7)*l1/l2)) + 0.3 +0.4;
//				jnt_it[2] = -1;
//							error_file 	 << "Joint 1 set to " << supp_target_value[1] << std::endl;
//										error_file 	 << "Joint 2 set to " << supp_target_value[2] << std::endl;
//
//			}
//
//		//		if (supp_target_value[2] > -1.3)
//		//		{
//		//			supp_target_value[2] = supp_target_value[2] - 0.3;
//		//		//	supp_target_value[2] = 3.14 - (+ supp_target_value[1] + acos(sin(-supp_target_value[1])*l1/l2) + 1.57) - 0.3 -0.4 -0.3;
//		//			RTT::log(RTT::Warning) << "Joint 2 set to " << supp_target_value[2] << RTT::endlog();
//		//			error_file 	 << "Joint 2 set to " << supp_target_value[2] << std::endl;
//		//		}
//		//		else
//		//		{
//		//
//		//			if (supp_target_value[1] > -1.8)
//		//			{
//		//				supp_target_value[1] = supp_target_value[1] - 0.3;
//		//			}
//		//			else
//		//			{
//		//				supp_target_value[1] = -0.1;
//		//			}
//		//				supp_target_value[2] = 0.2;
//		//			//			supp_target_value[2] =3.14 - (+ supp_target_value[1] + acos(sin(-supp_target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;//0.2;//0.7;//0.2;
//		//
//		//			RTT::log(RTT::Warning) << "Joint 1 set to " << supp_target_value[1] << RTT::endlog();
//		//			error_file 	 << "Joint 1 set to " << supp_target_value[1] << std::endl;
//		//		}
//
//		target_value[2] = supp_target_value[2];
//		target_value[1] = supp_target_value[1];
//		supp_target_value[0] = 0.1;
//		target_value[0] = supp_target_value[0];
//		supp_target_value[3] = -1.4064;
//		target_value[3] = supp_target_value[3];
//		supp_target_value[4] = -1.5318;
//		target_value[4] = supp_target_value[4];
//		supp_target_value[5] = -0.3283;
//		target_value[5] = supp_target_value[5];
//		curr_mass = 5;
//		eeMass(curr_mass , model);
//
//		}
//		else
//		{
//			curr_mass = curr_mass + 0.05;
//		//	curr_mass = curr_mass + 0.1;
//			eeMass(curr_mass,model);
//			RTT::log(RTT::Warning) << "Mass set to "  << curr_mass << RTT::endlog();
//			error_file 	 << "Mass set to " << curr_mass << std::endl;
//
//
//		}
//	}
//






		/***************************************************************************************************************************************************/
		/*
		 * PID Component part
		 */

		pid_it++;

		// PID control of position with torque
		if (pid_it >= dynStepSize)
		{
			for (unsigned j = 0; j < joints_idx.size(); j++)
			{
				//Regular PID
			/*
				error_value[j] = target_value[j] -  model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian();
				error_derivative[j] = error_value[j]-last_error[j];
				cumulative_error[j] = cumulative_error[j] + error_value[j];
				control_value[j] = constrainCommand(jnt_effort[j] , -jnt_effort[j] , error_value[j]*Kp[j] + cumulative_error[j]*Ki[j]*dynStepSize + error_derivative[j]*(Kd[j]/dynStepSize));
				last_error[j] = error_value[j];
				pid_it = 0;
			*/

				// PID anti wind-up

				error_value[j] = target_value[j] -  model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian();
				error_derivative[j] = error_value[j]-last_error[j];
				control_value[j] = constrainCommand( jnt_effort[j], -jnt_effort[j] , Kp[j]*error_value[j] + (Kd[j]/dynStepSize)*error_derivative[j] + errorI[j]);
				errorI[j] = errorI[j] + dynStepSize*(Ki[j]*error_value[j] + Ks[j]*(control_value[j] - Kp[j]*error_value[j] - (Kd[j]/dynStepSize)*error_derivative[j] - errorI[j] ));
				last_error[j] = error_value[j];
				pid_it = 0;

			//	RTT::log(RTT::Error) << j << " " << error_value[j] << "  " << control_value[j] << "  " << errorI[j] << RTT::endlog() ;
			}

			//	error_file << "cmd 1 " << control_value[1]  << std::endl ;


		// For tuning PID.
		//RTT::log(RTT::Error) << "Ki " << Kd[1]  << " agl0 "	<< model->GetJoints()[joints_idx[0]]->GetAngle(0).Radian() <<" trg_agl1 "	<<target_value[1] <<  " agl1 "	<< model->GetJoints()[joints_idx[1]]->GetAngle(0).Radian() <<  " trg_agl2 "	<<target_value[2] << " agl2 "	<< model->GetJoints()[joints_idx[2]]->GetAngle(0).Radian() << RTT::endlog();

		}

		for (unsigned j = 0; j < joints_idx.size(); j++)
		{
			gazebo_joints_[joints_idx[j]]->SetForce(0 , control_value[j]);
		}

		/***************************************************************************************************************************************************/


		/*
		 * Test for mass modification at the end-effector.
		 */

		/*
		if (sim_id == 8000)
		{
			eeMass(5.0 , model);
			RTT::log(RTT::Error) << "Mass set to 5 " << RTT::endlog() ;
		}
		if (sim_id == 12000)
		{
			RTT::log(RTT::Error) << "Mass set to 1 " << RTT::endlog() ;
			eeMass(1.0 , model);
		}
		if (sim_id == 16000)
		{
			RTT::log(RTT::Error) << "Mass set to 0 " << RTT::endlog() ;
			eeMass(0.001 , model);
		}
		*/

		sim_id ++;


	}



	virtual bool configureHook() {
		return true;
	}


	virtual void updateHook() {
		return;
	}

protected:

	//! Synchronization ??

	// File where data are written.
	std::ofstream data_file;
	std::ofstream error_file;
	std::ofstream sensibility_file;
	int comp_wait;

	bool random_pos;
	std::vector<double> nb_recording;
	std::vector<double> ee_mass;
	int mass_id;
	int nearest_mass; // To indicate the model used for compliance mode.
	double curr_mass;

	int nb_iteration; // number of hook iterations for one tested position.
	int sim_id; // number of angle positions tested.

	int elm_id;

	std::vector<int> joints_idx;
	std::vector<int> links_idx;


	std::vector<gazebo::physics::JointPtr> gazebo_joints_;
	gazebo::physics::Link_V model_links_;
	std::vector<std::string> joint_names_;
	std::vector<std::string> link_names_;
	int nb_links;

	int nb_static_joints;

	double l1; // length of link1
	double l2;  // length of link2


	// For recording data randomly.
	std::vector<double> jnt_it;
	std::vector<double> jnt_width; // Shouldn't forget to modify it for joint 2!!

	// Variable to save intermediate robot position - to decide if the data will be written in the file.
	std::vector< std::vector<double> > inter_torque;

	// Variables for PID controller : transform to vector for several joints.
	std::vector<double> error_value;
	std::vector<double> cumulative_error;
	std::vector<double> last_error;
	double dynStepSize;
	std::vector<double> Kp;
	std::vector<double> Kd;
	std::vector<double> Ki;
	std::vector<double> Ks;
	std::vector<double> control_value;
	std::vector<double> target_value;
	std::vector<double> errorI;
	std::vector<double> error_derivative;
	std::vector<double> supp_target_value;


	int pid_it;


	// ELM Learner

	// Several ELM initialized to consider different payload at the end effector

	ExtremeLearningMachinePtr elm_0;
	ExtremeLearningMachinePtr elm_1;
	ExtremeLearningMachinePtr elm_3;
	ExtremeLearningMachinePtr elm_5;

	std::vector<double> torque_difference;
	std::vector<double> torque_difference_0;
	std::vector<double> torque_difference_1;
	std::vector<double> torque_difference_3;
	std::vector<double> torque_difference_5;
	std::vector<double> payload_error; // To compare the differences between current torque and awaited torque or different payloads.
	std::vector<double> payload_index; // contains payloads in correct order to find current payload.
	double trq_err_1;
	double trq_err_2;
	double trq1;
	double trq2;


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
	int nb_measure_trq;

};

ORO_LIST_COMPONENT_TYPE(UR5RttGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();


