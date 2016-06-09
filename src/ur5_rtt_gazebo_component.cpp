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
			RTT::TaskContext(name), nb_static_joints(0) , inter_torque({{0} , {0} , {0} , {0} , {0} , {0}}),  trqCmdOutput(0) , targetPosition(0) , cmdJntTrq_Flow(RTT::FlowStatus(0)) , PIDStepSize(0.0005) // Frequency of PID component
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

		RTT::log(RTT::Warning) << "Model has " << gazebo_joints_.size()
				<< " joints" << RTT::endlog();

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

		RTT::log(RTT::Warning) << "Gazebo model found " << joints_idx.size()
				<< " joints " << RTT::endlog();
		RTT::log(RTT::Warning) << "Done configuring gazebo" << RTT::endlog();

		for (unsigned j = 0; j < joints_idx.size(); j++)
		{
			gazebo_joints_[joints_idx[j]]->SetProvideFeedback(true);
		}

		data_file.open("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/test_data.txt");
		if (!data_file)
			RTT::log(RTT::Error) << "The file could not be opened." << RTT::endlog();



		this->addPort("cmdJntTrq", cmdJntTrq_Port);
		trqCmdOutput = {0 , 0 ,0 ,0 ,0 , 0};

		this->addPort("currJntPos", currJntPos_Port);
		currJntPos_Port.setDataSample(trqCmdOutput);
		this->addPort("refJntPos", refJntPos_Port);
		refJntPos_Port.setDataSample(trqCmdOutput);
		RTT::log(RTT::Warning) << "Port added. " << RTT::endlog();

		targetPosition = { 0 , -0.1 , 3.14 - (+ -0.1 + acos(sin(0.1)*l1/l2) + 1.57) - 0.3 -0.4 , -3.14 , -1.4 , -1.57};


		RTT::log(RTT::Warning) << "Configure hook finished. " << RTT::endlog();
		RTT::log(RTT::Error) << "RttGazeboComponent configured." << RTT::endlog();

		return true;
	}

	//! Called from Gazebo
	void UR5RttGazeboComponent::gazeboUpdateHook(gazebo::physics::ModelPtr model) {
		RTT::log(RTT::Warning) << "Beginning GazeboComponent update. " << RTT::endlog();
		if (model.get() == NULL) {
			return;
		}



			cmdJntTrq_Flow = cmdJntTrq_Port.read(trqCmdOutput);
		//	RTT::log(RTT::Warning) << "G: Torque command read." << RTT::endlog();


			for (unsigned j = 0; j < joints_idx.size(); j++)
			{
				gazebo_joints_[joints_idx[j]]->SetForce(0 , trqCmdOutput[j]/PIDStepSize);
			}
		//	RTT::log(RTT::Warning) << "G: Torque command set." << RTT::endlog();

			sim_id ++;

		nb_iteration++;


		if ((nb_iteration == 2950) || (nb_iteration == 2960) || (nb_iteration == 2970) || (nb_iteration == 2980) || (nb_iteration == 2990)) // To check if position is stable.
		{
			for (unsigned j = 0; j < joints_idx.size(); j++)
			{
				gazebo::physics::JointWrench w1 = gazebo_joints_[joints_idx[j]]->GetForceTorque(0u);
				gazebo::math::Vector3 a1 = gazebo_joints_[joints_idx[j]]->GetLocalAxis(0u);
				inter_torque[j].push_back(a1.Dot(w1.body1Torque));
			}
		}


		if (nb_iteration >= 3000) // For stabilisation of the torque.
		{
			// Data recording.

			for (unsigned j = 0; j < joints_idx.size(); j++)
			{
				data_file << "{ sim_id = " << sim_id << " ; ";

				for (unsigned j = 0; j < joints_idx.size(); j++)
				{
					data_file << "jnt " << j << " ; ";
					gazebo::physics::JointWrench w1 = gazebo_joints_[joints_idx[j]]->GetForceTorque(0u);
					gazebo::math::Vector3 a1 = gazebo_joints_[joints_idx[j]]->GetLocalAxis(0u);
					data_file << "trq "<< *std::min_element((inter_torque[j]).begin(),(inter_torque[j]).end()) << " ; "; // See torque computation !!
					data_file << "agl "	<< model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian() << " ; ";
					data_file << "trg_agl "	<<targetPosition[j] << " ; ";
				}

				for (unsigned j = 0; j < joints_idx.size(); j++)
				{
					data_file << "ctrl "	<< trqCmdOutput[j] << " ; ";
				}
				data_file << " }" << std::endl;
			}
		//	RTT::log(RTT::Warning) << "G: Data recorded." << RTT::endlog();


			nb_iteration = 0;

			// Changes desired position  of each joint.
/*
			// To make the target values of the joints change.
			if (targetPosition[5] < 3.14)
			{
				targetPosition[5] = targetPosition[5] + 1.17;
			}
			else
			{
				if (targetPosition[4] < 1.57)
				{
					targetPosition[4] = targetPosition[4] + 0.7;
				}
				else
				{
					if ( (targetPosition[3] <(0.7)))
					{
						targetPosition[3] = targetPosition[3] + 0.9;
					}
					else
					{
						if (targetPosition[1] < 1.57 && targetPosition[2] > (3.14 - (+targetPosition[1] + acos(sin(-targetPosition[1])*l1/l2) + 1.57) - 3.14 + 0.8))
						{
							targetPosition[2] = targetPosition[2] - 0.3;
						}
						else if (targetPosition[1] > 1.57 && targetPosition[2] > (-3.14 + (- targetPosition[1] + 1.7 - acos(cos(-targetPosition[1]+1.7)*l1/l2))+3.14 - 0.8))
						{
							targetPosition[2] = targetPosition[2] + 0.3;
						}
						else
						{
							if (targetPosition[1] > -2.3)
							{
								targetPosition[1] = targetPosition[1] - 0.4;
							}
							else
							{
								targetPosition[1] = -0.1;


								if (targetPosition[0] >= 6.28)
								{
									targetPosition[0] = 0;
								}
								else
								{
									targetPosition[0] = targetPosition[0] + 1.5;
								}

							}
									if (targetPosition[1] < 1.57)
							{
								targetPosition[2] = 3.14 - (+ targetPosition[1] + acos(sin(-targetPosition[1])*l1/l2) + 1.57) - 0.3 -0.4;
							}
							else
							{
								targetPosition[2] = -3.14 + (- targetPosition[1] + 1.7 - acos(cos(-targetPosition[1]+1.7)*l1/l2)) + 0.3 +0.4;
							}
						}
						targetPosition[3] = -3.14;
					}
					targetPosition[4] = -1.4;
				}
				targetPosition[5] = -1.57;
			}
*/


		}





		if (currJntPos_Port.connected()) {
			currJntPos_Port.write(currPosition);
		}
		//RTT::log(RTT::Warning) << "G: Current position written." << RTT::endlog();

		if (refJntPos_Port.connected()) {
			refJntPos_Port.write(targetPosition);
		}
		//RTT::log(RTT::Warning) << "G: Target position written." << RTT::endlog();


				RTT::log(RTT::Warning) << "GazeboComponent updated. " << RTT::endlog();

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

ORO_LIST_COMPONENT_TYPE(UR5RttGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();


