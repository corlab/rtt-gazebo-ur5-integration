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
#include <math.h>

#include "ur5_rtt_gazebo_component.hpp"

using namespace std;

#define l(lvl) RTT::log(lvl) << "[" << this->getName() << "] "




UR5RttGazeboComponent::UR5RttGazeboComponent(std::string const& name) :
			RTT::TaskContext(name), nb_static_joints(0),nb_links(0), meanCollTrq(0), newMass_Flow(RTT::NoData), elm_id(0), compPos_Flow(RTT::NoData), inter_torque_elm({{0} , {0} , {0} , {0} , {0} , {0}}) , curr_mass(0), inter_torque({{0} , {0} , {0} , {0} , {0} , {0}}),  trqCmdOutput(0) , targetPosition(0) , currPosition(0) , cmdJntTrq_Flow(RTT::FlowStatus(0)) , trgtPos_Flow(RTT::FlowStatus(0)), PIDStepSize(5) // Frequency of PID component
	{
		// Add required gazebo interfaces.
		this->provides("gazebo")->addOperation("configure", &UR5RttGazeboComponent::gazeboConfigureHook, this, RTT::ClientThread);
		this->provides("gazebo")->addOperation("update", &UR5RttGazeboComponent::gazeboUpdateHook, this, RTT::ClientThread);

		nb_iteration = 0;
		sim_id = 1;

		l1 = 0.7;// find real values later !!
		l2 = 0.9;// find real values later !!

	}

	//! Called from gazebo.
	bool UR5RttGazeboComponent::gazeboConfigureHook(gazebo::physics::ModelPtr model) {

		RTT::log(RTT::Error) << "Beginning RttGazeboComponent configuration." << RTT::endlog();


		if (model.get() == NULL) {
			RTT::log(RTT::Error) << "No model could be loaded" << RTT::endlog();
			std::cout << "No model could be loaded" << RTT::endlog();
			return false;
		}



		// Get the joints
		gazebo_joints_ = model->GetJoints();
		model_links_ = model->GetLinks(); // Only working when starting gzserver and gzclient separately!

		RTT::log(RTT::Warning) << "Model has " << gazebo_joints_.size() << " joints" << RTT::endlog();

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

		curr_mass = 1;
			RTT::log(RTT::Error) << "Beginning mass model" << RTT::endlog();

					// To test guessing of the payload.
			eeMass(curr_mass, model);

			RTT::log(RTT::Error) << "Mass model set" << RTT::endlog();


		RTT::log(RTT::Warning) << "Gazebo model found " << joints_idx.size()
				<< " joints " << RTT::endlog();
		RTT::log(RTT::Warning) << "Done configuring gazebo" << RTT::endlog();

		for (unsigned j = 0; j < joints_idx.size(); j++)
		{
			gazebo_joints_[joints_idx[j]]->SetProvideFeedback(true);
			currPosition.push_back(j);
			meanTrq.push_back(0);
			currVelo.push_back(0);
			meanCollTrq.push_back(0);
		}


		this->addPort("cmdJntTrq", cmdJntTrq_Port);
		trqCmdOutput = {0.0 , 0.0 ,0.0 ,0.0 ,0.0 , 0.0};
		this->addPort("trgtPos", trgtPos_Port);
		this->addPort("compPos" , compPos_Port);
		this->addPort("newMass" , newMass_Port);


		this->addPort("currJntPos", currJntPos_Port);
		currJntPos_Port.setDataSample(trqCmdOutput);
		this->addPort("refJntPos", refJntPos_Port);
		refJntPos_Port.setDataSample(trqCmdOutput);
		this->addPort("currJntTrq", currJntTrq_Port);
		currJntTrq_Port.setDataSample(trqCmdOutput);
		this->addPort("meanJntTrq", meanJntTrq_Port);
		meanJntTrq_Port.setDataSample(trqCmdOutput);
		this->addPort("currVelo" , currVelocity_Port);
		currVelocity_Port.setDataSample(trqCmdOutput);
		this->addPort("meanCollTrq" , meanCollTrq_Port);
		meanCollTrq_Port.setDataSample(trqCmdOutput);

		this->addPort("currMass" , currMass_Port);
		currMass_Port.setDataSample(0);

		RTT::log(RTT::Warning) << "Port added. " << RTT::endlog();


		targetPosition = { 0 , -0.1 , 3.14 - (+ -0.1 + acos(sin(0.1)*l1/l2) + 1.57) - 0.3 -0.4 , -3.14 , -1.4 , -1.57};


		RTT::log(RTT::Warning) << "Configure hook finished. " << RTT::endlog();
		RTT::log(RTT::Error) << "RttGazeboComponent configured." << RTT::endlog();

		return true;
	}

	//! Called from Gazebo
	void UR5RttGazeboComponent::gazeboUpdateHook(gazebo::physics::ModelPtr model) {
//		RTT::log(RTT::Warning) << "Beginning GazeboComponent update. " << RTT::endlog();
		if (model.get() == NULL) {
			return;
		}



		if (cmdJntTrq_Port.connected())
			cmdJntTrq_Flow = cmdJntTrq_Port.read(trqCmdOutput);
		if (trgtPos_Port.connected())
			trgtPos_Flow = trgtPos_Port.read(targetPosition);
		if (compPos_Port.connected())
			compPos_Flow = compPos_Port.read(targetPosition);



		if (newMass_Port.connected())
		{
			newMass_Flow = newMass_Port.read(new_mass);
		}


		if (newMass_Flow == RTT::NewData)
		{
			curr_mass = new_mass;
			eeMass(curr_mass, model);
			RTT::log(RTT::Error) << "New mass set to model" << RTT::endlog();
		}

		if (trgtPos_Flow == RTT::NewData)
		{
			RTT::log(RTT::Error) << "New target position" << RTT::endlog();

			nb_iteration = 0;
		}

		for (unsigned j = 0; j < joints_idx.size(); j++)
		{
			gazebo_joints_[joints_idx[j]]->SetForce(0 , trqCmdOutput[j]);
		}


		sim_id ++;

		nb_iteration++;
		elm_id ++;


			if ((elm_id == (10)) ||(elm_id == (20)) ||(elm_id == (30)) ||(elm_id == (40)) ||(elm_id == (50)) ||(elm_id == (60)) ||(elm_id == (70)) ||(elm_id == (80)) ||(elm_id == (90)) || (elm_id == (100)) || (elm_id == (110)) || (elm_id == (120)) || (elm_id == (130)) || (elm_id == (140)) || (elm_id == (150)) || (sim_id == (160)) || (elm_id == (170)) || (elm_id == (180)) || (elm_id == (190)) || (elm_id == 200) ) // To check if position is stable.
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

			if (elm_id == 200)
			{
				elm_id = 0;


						// ********************************************************************************************************************************
						// * ELM part:
						// * Computing the difference between current torque and awaited torque for each joint.
						// Create vector containing awaited position.

				for (unsigned j = 0; j < joints_idx.size(); j++)
				{
					double mean_of_torques_elm = 0;
					mean_of_torques_elm = (std::accumulate((inter_torque_elm[j]).begin(),(inter_torque_elm[j]).end(), 0.0))/20;
					meanTrq[j] = mean_of_torques_elm;
					inter_torque_elm[j] = {0};
					mean_of_torques_elm = 0;
					currVelo[j] = model->GetJoints()[joints_idx[j]]->GetVelocity(0);
				}
				if (meanJntTrq_Port.connected())
				{
					meanJntTrq_Port.write(meanTrq);
				}
				if (currVelocity_Port.connected())
				{
					currVelocity_Port.write(currVelo);
				}
			}

		for (unsigned j = 0; j < joints_idx.size(); j++)
		{
			currPosition[j] = model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian();
		}

		if ((nb_iteration == 3410) || (nb_iteration == 3420) || (nb_iteration == 3430) || (nb_iteration == 3440) || (nb_iteration == 3450) || (nb_iteration == 3460) || (nb_iteration == 3470) || (nb_iteration == 3480) || (nb_iteration == 3490) || (nb_iteration == 3500) ) // To check if position is stable.
		{
			for (unsigned j = 0; j < joints_idx.size(); j++)
			{
				gazebo::physics::JointWrench w1 = gazebo_joints_[joints_idx[j]]->GetForceTorque(0u);
				gazebo::math::Vector3 a1 = gazebo_joints_[joints_idx[j]]->GetLocalAxis(0u);
				inter_torque[j].push_back(a1.Dot(w1.body1Torque));
			}
		}


		if (nb_iteration >= 3500) // For stabilisation of the torque.
		{
			nb_iteration = 0;
			for (unsigned j = 0; j < joints_idx.size(); j++)
			{
				// Computing the mean of the torques.
				meanCollTrq[j] = (std::accumulate((inter_torque[j]).begin(),(inter_torque[j]).end(), 0.0))/10.0;
				inter_torque[j] = {0};
			}
			if (meanCollTrq_Port.connected())
				meanCollTrq_Port.write(meanCollTrq);
		}





		if (currJntPos_Port.connected()) {
			currJntPos_Port.write(currPosition);
		}

		if (refJntPos_Port.connected()) {
			refJntPos_Port.write(targetPosition);
		}

		if (currJntTrq_Port.connected()) {
			currJntTrq_Port.write(trqCmdOutput);
		}

	}


	bool UR5RttGazeboComponent::startHook() {
	    return true;
	}

	bool UR5RttGazeboComponent::configureHook() {
		return true;
	}


	void UR5RttGazeboComponent::updateHook() {
		return;
	}

	void UR5RttGazeboComponent::stopHook() {
		return ;
	}

	void UR5RttGazeboComponent::cleanupHook() {
		return ;
	}

	void UR5RttGazeboComponent::eeMass(double mass , gazebo::physics::ModelPtr model)
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
			curr_mass = mass;
			if (currMass_Port.connected()) {
				currMass_Port.write(mass);
			}
			RTT::log(RTT::Error) << "end model modification" << RTT::endlog();

	}

ORO_LIST_COMPONENT_TYPE(UR5RttGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();


