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


//#include "ur5_rtt_gazebo_component.hpp"

using namespace std;

#define l(lvl) RTT::log(lvl) << "[" << this->getName() << "] "



class UR5RttGazeboComponent: public RTT::TaskContext {
public:



	// BEST  Kp({2700 , 2700  , 2700 , 2700 , 2700 , 2700 }) , Ki({8.7 , 8.7  , 8.7  , 8.7  , 8.7  , 8.7 }) , Kd({209250 ,209250 , 209250 , 209250 , 209250 , 209250})

	UR5RttGazeboComponent(std::string const& name) :
			RTT::TaskContext(name), nb_static_joints(0) , nb_links(0), inter_torque({{0} , {0} , {0} , {0} , {0} , {0}}) , control_value(0) , target_value(0), error_value(0), cumulative_error(0), last_error(0), dynStepSize(5) , pid_it(5) ,Kp({10000 , 15000  , 15000 , 2700 , 2700 , 5000 }) , Ki({2 , 2 , 2 , 1 , 1 , 2 }) , Kd({209250 ,209250 , 209250 , 209250 , 209250 , 189250})  // HACK: The urdf has static tag for base_link, which makes it appear in gazebo as a joint.
			 {
		// Add required gazebo interfaces.
		this->provides("gazebo")->addOperation("configure",
				&UR5RttGazeboComponent::gazeboConfigureHook, this,
				RTT::ClientThread);
		this->provides("gazebo")->addOperation("update",
				&UR5RttGazeboComponent::gazeboUpdateHook, this, RTT::ClientThread);

		nb_iteration = 0;
		sim_id = 1;

		l1 = 0.7; // find real values later !!
		l2 = 0.9;// find real values later !!

		}

	//! Called from gazebo
	virtual bool gazeboConfigureHook(gazebo::physics::ModelPtr model) {
		if (model.get() == NULL) {
			RTT::log(RTT::Error) << "No model could be loaded" << RTT::endlog();
			std::cout << "No model could be loaded" << RTT::endlog();
			return false;
		}


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


		// TODO Read links. Test link parameters modification virtual
		//void gazebo::physics::Link::SetInertial 	( 	const InertialPtr &  	_inertial	)
		//void gazebo::physics::Link::UpdateMass 	( 		)

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
			control_value.push_back(0);
			target_value.push_back(0);

		}

		data_file.open("/homes/abalayn/workspace/rtt-gazebo-ur5-integration/test_data.txt");
		if (!data_file)
			RTT::log(RTT::Error) << "The file could not be open." << RTT::endlog();


		target_value[0] = 0 ;
		target_value[1] = -0.1 ;
		target_value[2] =  3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
		target_value[3] = -3.14;
		target_value[4] = -1.4;
		target_value[5] = -1.57;

		return true;
	}

	//! Called from Gazebo
	virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model) {
		if (model.get() == NULL) {
			return;
		}
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


				data_file << "{ sim_id = " << sim_id << " ; ";
				double mean_of_torques = 0;
				for (unsigned j = 0; j < joints_idx.size(); j++)
				{
					data_file << "jnt " << j << " ; ";

					// Computing the mean of the torques.

					mean_of_torques = (std::accumulate((inter_torque[j]).begin(),(inter_torque[j]).end(), 0))/5.0;

					data_file << "trq "<< mean_of_torques << " ; ";
					data_file << "agl "	<< model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian() << " ; ";
					data_file << "trg_agl "	<<target_value[j] << " ; ";
					mean_of_torques = 0;
					inter_torque[j] = {0};
				}
				data_file << " }" << std::endl;

			nb_iteration = 0;


			// Changes desired position  of each joint.

			if ((target_value[5]+1.17) < 3.14)
			{
				target_value[5] = target_value[5] + 1.17;
			}
			else
			{
				if ((target_value[4]+0.7) < 1.57)
				{
					target_value[4] = target_value[4] + 0.7;
				}
				else
				{
					if ( (target_value[3]+0.9) <0.7)
					{
						target_value[3] = target_value[3] + 0.9;
					}
					else
					{
						if (target_value[1] < 1.57 && (target_value[2]-0.4) > (3.14 - (+target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 3.14 + 0.8))
						{
							target_value[2] = target_value[2] - 0.4;
						}
						else if (target_value[1] > 1.57 && (target_value[2]+0.4) < (-3.14 + (- target_value[1] + 1.7 - acos(cos(-target_value[1]+1.7)*l1/l2))+3.14 - 0.8))
						{
							target_value[2] = target_value[2] + 0.4;
						}
						else
						{
							if ((target_value[1] -0.5) > -2.3)
							{
								target_value[1] = target_value[1] - 0.5;
							}
							else
							{
								target_value[1] = -0.1;


								if ((target_value[0]+1.5) >= 6.28)
								{
									target_value[0] = 0;
									 // Code to change mass and inertia tensor at the end effector during the simulation. // Here mass set to 1 for data recording.

															RTT::log(RTT::Error) << "Model modification." << RTT::endlog();
															auto inertial = model->GetLinks()[links_idx[nb_links-1]]->GetInertial();
															RTT::log(RTT::Error) << "Inertia pointer prepared." << RTT::endlog();
															inertial->SetMass(1.0);
															RTT::log(RTT::Error) << "Mass prepared." << RTT::endlog();
															inertial->SetInertiaMatrix(0.000416667, 0.000416667, 0.000416667, 0, 0, 0);
															RTT::log(RTT::Error) << "Inertia matrix prepared." << RTT::endlog();
															model_links_[links_idx[nb_links-1]]->SetInertial(inertial);
															RTT::log(RTT::Error) << "Inertia matrix set." << RTT::endlog();
															model_links_[links_idx[nb_links-1]]->UpdateMass();
															RTT::log(RTT::Error) << "Inertia set to model. " << RTT::endlog();


								}
								else
								{
									target_value[0] = target_value[0] + 1.5;
								}

							}
							if (target_value[1] < 1.57)
							{
								target_value[2] = 3.14 - (+ target_value[1] + acos(sin(-target_value[1])*l1/l2) + 1.57) - 0.3 -0.4;
							}
							else
							{
								target_value[2] = -3.14 + (- target_value[1] + 1.7 - acos(cos(-target_value[1]+1.7)*l1/l2)) + 0.3 +0.4;
							}
						}
						target_value[3] = -3.14;
					}
					target_value[4] = -1.4;
				}
				target_value[5] = -1.57;
			}




		}

		/*// For videos
		if (sim_id < 5000)
		{
			target_value[0] = -1;
			target_value[1] = -0.7;
			target_value[2] = 1.3;
			target_value[3] = -2;
			target_value[4] = -1.4;
			target_value[5] = -1.57;
		}
		else if ((sim_id  > 5000) && (sim_id < 6500))
		{

			target_value[2] = target_value[2] - 0.0006;

		}
		else if ((sim_id  > 6500) && (sim_id < 10200))
		{
			target_value[0] = target_value[0] - 0.0006;

		}
		else if ((sim_id  > 10200) && (sim_id < 11700))
		{
			target_value[2] = target_value[2] + 0.0006;
		}


		target_value[0] = -1;
		target_value[1] = -1.57;
		target_value[2] = 1.57;
		target_value[3] = -1.57;
		target_value[4] = -1.57;
		target_value[5] = -1.57;
*/
		pid_it++;

		// PID control of position with torque
		if (pid_it >= dynStepSize)
		{
			for (unsigned j = 0; j < joints_idx.size(); j++)
			{
				error_value[j] = target_value[j] -  model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian();
				control_value[j] = error_value[j]*Kp[j];
				cumulative_error[j] = cumulative_error[j] + error_value[j];
				control_value[j] = control_value[j] + cumulative_error[j]*Ki[j]*dynStepSize;
				control_value[j] = control_value[j] + (error_value[j]-last_error[j])*(Kd[j]/dynStepSize);
				last_error[j] = error_value[j];
				pid_it = 0;
			}
		// For tuning PID.
		//RTT::log(RTT::Error) << "Ki " << Kd[1]  << " agl0 "	<< model->GetJoints()[joints_idx[0]]->GetAngle(0).Radian() <<" trg_agl1 "	<<target_value[1] <<  " agl1 "	<< model->GetJoints()[joints_idx[1]]->GetAngle(0).Radian() <<  " trg_agl2 "	<<target_value[2] << " agl2 "	<< model->GetJoints()[joints_idx[2]]->GetAngle(0).Radian() << RTT::endlog();

		}

		for (unsigned j = 0; j < joints_idx.size(); j++)
		{
			gazebo_joints_[joints_idx[j]]->SetForce(0 , control_value[j]/dynStepSize);
		}



		sim_id ++;




		/*//To be added in the URDF to modify end effector mass
		 *
		 * <link name="ee_link">
    <collision>
      <origin xyz="0.025 0 0.0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0.025 0 0.0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>

    <inertial>
      <origin xyz="0 0 1" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia
        ixx="0.0004" ixy="0.0" ixz="0.0"
        iyy="0.0004" iyz="0.0"
        izz="0.0004"/>
    </inertial>
  </link>
		 */

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

	int nb_iteration; // number of hook iterations for one tested position.
	int sim_id; // number of angle positions tested.

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
	std::vector<double> control_value;
	std::vector<double> target_value;
	int pid_it;

};

ORO_LIST_COMPONENT_TYPE(UR5RttGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();


