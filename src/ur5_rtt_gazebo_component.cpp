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

#define l(lvl) RTT::log(lvl) << "[" << this->getName() << "] "

class UR5RttGazeboComponent: public RTT::TaskContext {
public:

	UR5RttGazeboComponent(std::string const& name) :
			RTT::TaskContext(name), steps_rtt_(0), steps_gz_(0), rtt_done(
			true), gazebo_done(false), new_data(true), cnt_lock_(100), last_steps_rtt_(
					0), set_new_pos(false), nb_no_data_(0), set_brakes(false), nb_static_joints(
					0), // HACK: The urdf has static tag for base_link, which makes it appear in gazebo as a joint
			last_gz_update_time_(0), nb_cmd_received_(0), sync_with_cmds_(true) {
		// Add required gazebo interfaces
		this->provides("gazebo")->addOperation("configure",
				&UR5RttGazeboComponent::gazeboConfigureHook, this,
				RTT::ClientThread);
		this->provides("gazebo")->addOperation("update",
				&UR5RttGazeboComponent::gazeboUpdateHook, this, RTT::ClientThread);

//		this->addOperation("setLinkGravityMode",&UR5RttGazeboComponent::setLinkGravityMode,this,RTT::ClientThread); // TODO

		this->ports()->addPort("JointPositionCommand",
				port_JointPositionCommand).doc(
				"Input for JointPosition-cmds from Orocos to Gazebo world.");
		this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc(
				"Input for JointTorque-cmds from Orocos to Gazebo world.");
		this->ports()->addPort("JointVelocityCommand",
				port_JointVelocityCommand).doc(
				"Input for JointVelocity-cmds from Orocos to Gazebo world.");

		this->ports()->addPort("JointVelocity", port_JointVelocity).doc(
				"Output for JointVelocity-fbs from Gazebo to Orocos world.");
		this->ports()->addPort("JointTorque", port_JointTorque).doc(
				"Output for JointTorques-fbs from Gazebo to Orocos world.");
		this->ports()->addPort("JointPosition", port_JointPosition).doc(
				"Output for JointPosition-fbs from Gazebo to Orocos world.");

		this->provides("debug")->addAttribute("jnt_pos", jnt_pos_);
		this->provides("debug")->addAttribute("jnt_vel", jnt_vel_);
		this->provides("debug")->addAttribute("jnt_trq", jnt_trq_);
		this->provides("debug")->addAttribute("gz_time", gz_time_);
		this->provides("debug")->addAttribute("write_duration",
				write_duration_);
		this->provides("debug")->addAttribute("read_duration", read_duration_);
		this->provides("debug")->addAttribute("rtt_time", rtt_time_);
		this->provides("debug")->addAttribute("steps_rtt", steps_rtt_);
		this->provides("debug")->addAttribute("steps_gz", steps_gz_);
		this->provides("debug")->addAttribute("period_sim", period_sim_);
		this->provides("debug")->addAttribute("period_wall", period_wall_);

		this->provides("misc")->addAttribute("urdf_string", urdf_string);
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

//			RTT::log(RTT::Warning) << "Found joint [" << name << "] idx:"
//								<< idx << RTT::endlog();

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

		jnt_pos_cmd_ = rci::JointAngles::create(7, 0.0);
		jnt_pos_ = rci::JointAngles::create(7, 0.0);

		jnt_trq_cmd_ = rci::JointTorques::create(7, 0.0);
		jnt_trq_ = rci::JointTorques::create(7, 0.0);

		jnt_vel_cmd_ = rci::JointVelocities::create(7, 0.0);
		jnt_vel_ = rci::JointVelocities::create(7, 0.0);

		jnt_pos_brakes_ = rci::JointAngles::create(7, 0.0);

		port_JointPosition.setDataSample(jnt_pos_);
		port_JointVelocity.setDataSample(jnt_vel_);
		port_JointTorque.setDataSample(jnt_trq_);

		last_update_time_ = RTT::os::TimeService::Instance()->getNSecs(); //rtt_rosclock::rtt_now(); // still needed??
		RTT::log(RTT::Warning) << "Done configuring gazebo" << RTT::endlog();

		return true;
	}

	//! Called from Gazebo
	virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model) {
		if (model.get() == NULL) {
			return;
		}

		double rtt_time_now_ = 0;
		double rtt_last_clock = 0;

		do {
			updateData();

			if ((nb_cmd_received_ == 0 && data_fs == RTT::NoData)
					|| jnt_pos_fs == RTT::NewData)
				break;

			rtt_time_now_ = 1E-9
					* RTT::os::TimeService::ticks2nsecs(
							RTT::os::TimeService::Instance()->getTicks());

			if (rtt_time_now_ != rtt_last_clock && data_fs != RTT::NewData)
//				RTT::log(RTT::Debug) << getName() << " "
//						<< "Waiting for UpdateHook at " << rtt_time_now_
//						<< " v:" << nb_cmd_received_ << data_fs
//						<< RTT::endlog();
				rtt_last_clock = rtt_time_now_;

//			RTT::log(RTT::Debug) << getName() << " nb_cmd_received_ = "
//					<< nb_cmd_received_ << RTT::endlog();

		} while (!(RTT::NewData == data_fs && nb_cmd_received_)
				&& sync_with_cmds_);

		// Increment simulation step counter (debugging)
		steps_gz_++;

		// Get the RTT and gazebo time for debugging purposes
		rtt_time_ = 1E-9
				* RTT::os::TimeService::ticks2nsecs(
						RTT::os::TimeService::Instance()->getTicks());
		gazebo::common::Time gz_time = model->GetWorld()->GetSimTime();
		gz_time_ = (double) gz_time.sec + ((double) gz_time.nsec) * 1E-9;

		// Get the wall time
		gazebo::common::Time gz_wall_time = gazebo::common::Time::GetWallTime();
		wall_time_ = (double) gz_wall_time.sec
				+ ((double) gz_wall_time.nsec) * 1E-9;

//		RTT::log(RTT::Error) << "joints_idx.size() = " << joints_idx.size()
//				<< RTT::endlog();
//
//		RTT::log(RTT::Error) << "model = " << model->GetJointCount()
//				<< RTT::endlog();

		// Get state
		for (unsigned j = 0; j < joints_idx.size(); j++) {
//			RTT::log(RTT::Error) << "joints_idx j : " << j << " = "
//					<< joints_idx[j] << RTT::endlog();
//
//			RTT::log(RTT::Error) << "rad j1 = "
//					<< gazebo_joints_[joints_idx[j]]->GetName()
//					<< RTT::endlog();
//			RTT::log(RTT::Error) << "rad j2 = "
//					<< model->GetJoints()[joints_idx[j]]->GetName()
//					<< RTT::endlog();
//
//			RTT::log(RTT::Error) << "rad j2 = "
//					<< model->GetJoints()[joints_idx[j]]->GetAngle(0).Radian()
//					<< RTT::endlog();

			jnt_pos_->setFromRad(j,
					gazebo_joints_[joints_idx[j]]->GetAngle(0).Radian());
			jnt_vel_->setFromRad_s(j,
					gazebo_joints_[joints_idx[j]]->GetVelocity(0));

			gazebo::physics::JointWrench w =
					gazebo_joints_[joints_idx[j]]->GetForceTorque(0u);
			gazebo::math::Vector3 a =
					gazebo_joints_[joints_idx[j]]->GetLocalAxis(0u);
			//this->robotState.effort[i] = a.Dot(w.body1Torque);

			// do i need this?
			/*
			if (effortValQueue.size() > 0) {
				for (int i = 0; i < this->joints.size(); i++) {
					this->robotState.effort[i] *= 1.0 / effortValQueue.size();
				}
			}
			this->effort_average_cnt = (this->effort_average_cnt+1) % this->effort_average_window_size;
			*/
			jnt_trq_->setFromNm(j, a.Dot(w.body1Torque)); // perhaps change to GetForceTorque
//
//			RTT::log(RTT::Error) << "jnt_trq_ on (local axis) = "
//								<< a.Dot(w.body1Torque)
//								<< RTT::endlog();


//			RTT::log(RTT::Error) << "GetLinkTorque(0).x = "
//					<< gazebo_joints_[joints_idx[j]]->GetLinkTorque(0).x
//					<< RTT::endlog();
//			RTT::log(RTT::Error) << "GetLinkTorque(0).y = "
//					<< gazebo_joints_[joints_idx[j]]->GetLinkTorque(0).y
//					<< RTT::endlog();
//			RTT::log(RTT::Error) << "GetLinkTorque(0).z = "
//					<< gazebo_joints_[joints_idx[j]]->GetLinkTorque(0).z
//					<< RTT::endlog();
//
//			RTT::log(RTT::Error) << "body1Torque.x = "
//					<< gazebo_joints_[joints_idx[j]]->GetForceTorque(0).body1Torque.x
//					<< RTT::endlog();
//			RTT::log(RTT::Error) << "body1Torque.y = "
//					<< gazebo_joints_[joints_idx[j]]->GetForceTorque(0).body1Torque.y
//					<< RTT::endlog();
//			RTT::log(RTT::Error) << "body1Torque.z = "
//					<< gazebo_joints_[joints_idx[j]]->GetForceTorque(0).body1Torque.z
//					<< RTT::endlog();

//			RTT::log(RTT::Info) << "GetState: jnt[" << j << "]: jnt_pos_: "
//					<< jnt_pos_->rad(j) << ", jnt_vel_: " << jnt_vel_->rad_s(j)
//					<< ", jnt_trq_" << jnt_trq_->Nm(j) << RTT::endlog();

		}
//		RTT::log(RTT::Error) << "\n\n" << RTT::endlog();

		// Simulates breaks
		// NOTE: Gazebo is calling the callback very fast, so we might have false positive
		// This is only usefull when using ROS interface (not CORBA) + launching gazebo standalone
		// This allows the controller (launched with launch_gazebo:=false) to be restarted online

		switch (data_fs) {
		// Not Connected
		case RTT::NoData:
			set_brakes = true;
			break;

			// Connection lost
		case RTT::OldData:
			if (data_timestamp == last_data_timestamp && nb_no_data_++ >= 2)
				set_brakes = true;
			break;

			// OK
		case RTT::NewData:
			set_brakes = false;
			if (nb_no_data_-- <= 0)
				nb_no_data_ = 0;
			break;
		}

//		RTT::log(RTT::Error) << "Brakes?: " << set_brakes << RTT::endlog();

		// Set Gravity Mode or specified links
		for (std::map<gazebo::physics::LinkPtr, bool>::iterator it =
				this->gravity_mode_.begin(); it != this->gravity_mode_.end();
				++it) {
			it->first->SetGravityMode(it->second);
		}
//
//		RTT::log(RTT::Error) << "Gazebo data_fs : " << data_fs << ", ts: "
//				<< data_timestamp << ", last ts: " << last_data_timestamp
//				<< ", steps rtt: " << steps_rtt_ << ", last steps: "
//				<< last_steps_rtt_ << ", brakes: " << set_brakes
//				<< RTT::endlog();

		// Copy Current joint pos in case of brakes
		if (!set_brakes)
			for (unsigned j = 0; j < joints_idx.size(); j++)
				jnt_pos_brakes_->setFromRad(j, jnt_pos_->rad(j));

		// Force Joint Positions in case of a cmd
		if (set_new_pos) {
			RTT::log(RTT::Error) << "set_new_pos = true" << RTT::endlog();
			// Update specific joints regarding cmd
			for (unsigned j = 0; j < joints_idx.size(); j++) {
				gazebo_joints_[joints_idx[j]]->SetAngle(0,
						jnt_pos_cmd_->rad(j));
				jnt_pos_brakes_->setFromRad(j, jnt_pos_cmd_->rad(j));
			}

			// Aknowledge the settings
			set_new_pos = false;

		} else if (set_brakes) {
			for (unsigned j = 0; j < joints_idx.size(); j++)
				gazebo_joints_[joints_idx[j]]->SetAngle(0,
						jnt_pos_brakes_->rad(j));

		} else {
			// Write command
			// Update specific joints regarding cmd
			for (unsigned j = 0; j < joints_idx.size(); j++) {
				gazebo_joints_[joints_idx[j]]->SetForce(0, jnt_trq_cmd_->Nm(j));
//				RTT::log(RTT::Error) << "set Force: " << j << ", "
//						<< jnt_trq_cmd_->Nm(j) << RTT::endlog();
			}
		}
		last_data_timestamp = data_timestamp;

	}

	virtual bool configureHook() {
		return true;
	}

	void updateData() {
//		if (port_JointPositionCommand.connected()
//				|| port_JointTorqueCommand.connected()
//				|| port_JointVelocityCommand.connected()) {

//		}

		static double last_update_time_sim;
		period_sim_ = rtt_time_ - last_update_time_sim;
		last_update_time_sim = rtt_time_;

		// Compute period in wall clock
		static double last_update_time_wall;
		period_wall_ = wall_time_ - last_update_time_wall;
		last_update_time_wall = wall_time_;

		// Increment simulation step counter (debugging)
		steps_rtt_++;

		// Get command from ports

		RTT::os::TimeService::ticks read_start =
				RTT::os::TimeService::Instance()->getTicks();

		data_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);

		jnt_pos_fs = port_JointPositionCommand.read(jnt_pos_cmd_);

//		if (data_fs == RTT::NewData)
//			for (int i = 0; i < jnt_trq_cmd_->getDimension(); i++) {
//				l(RTT::Warning)<< "i: " << i << " = " << jnt_trq_cmd_->Nm(i) << RTT::endlog();
//			}

		if (jnt_pos_fs == RTT::NewData) {
			//set_new_pos = true; // TODO remove!
		}

		data_timestamp = new_pos_timestamp =
				RTT::os::TimeService::Instance()->getNSecs();

		read_duration_ = RTT::os::TimeService::Instance()->secondsSince(
				read_start);

		// Write state to ports
		RTT::os::TimeService::ticks write_start =
				RTT::os::TimeService::Instance()->getTicks();

		port_JointVelocity.write(jnt_vel_);
		port_JointPosition.write(jnt_pos_);
		port_JointTorque.write(jnt_trq_);

		write_duration_ = RTT::os::TimeService::Instance()->secondsSince(
				write_start);

		switch (data_fs) {
		case RTT::OldData:
			break;
		case RTT::NewData:
//			RTT::log(RTT::Error) << getName() << " " << data_fs << " at "
//					<< data_timestamp << RTT::endlog();
			nb_cmd_received_++;
			last_timestamp = data_timestamp;

//			for (int i = 0; i < jnt_trq_cmd_->getDimension(); i++) {
//				RTT::log(RTT::Info) << jnt_trq_cmd_->Nm(i) << RTT::endlog();
//			}
//			RTT::log(RTT::Info) << "-------" << RTT::endlog();
			break;
		case RTT::NoData:
			nb_cmd_received_ = 0;
			break;
		}
	}

	virtual void updateHook() {
		return;
	}
protected:

	//! Synchronization ??

	std::vector<int> joints_idx;

	std::map<gazebo::physics::LinkPtr, bool> gravity_mode_;

	std::vector<gazebo::physics::JointPtr> gazebo_joints_;
	gazebo::physics::Link_V model_links_;
	std::vector<std::string> joint_names_;

	RTT::InputPort<rci::JointAnglesPtr> port_JointPositionCommand;
	RTT::InputPort<rci::JointTorquesPtr> port_JointTorqueCommand;
	RTT::InputPort<rci::JointVelocitiesPtr> port_JointVelocityCommand;

	RTT::OutputPort<rci::JointAnglesPtr> port_JointPosition;
	RTT::OutputPort<rci::JointTorquesPtr> port_JointTorque;
	RTT::OutputPort<rci::JointVelocitiesPtr> port_JointVelocity;

	RTT::FlowStatus jnt_pos_fs, data_fs;

	rci::JointAnglesPtr jnt_pos_cmd_, jnt_pos_;
	rci::JointTorquesPtr jnt_trq_, jnt_trq_cmd_;
	rci::JointVelocitiesPtr jnt_vel_, jnt_vel_cmd_;
	rci::JointAnglesPtr jnt_pos_brakes_;

	//! RTT time for debugging
	double rtt_time_;
	//! Gazebo time for debugging
	double gz_time_;
	double wall_time_;

	RTT::nsecs last_gz_update_time_, new_pos_timestamp;
	RTT::Seconds gz_period_;
	RTT::Seconds gz_duration_;

	RTT::nsecs last_update_time_;
	RTT::Seconds rtt_period_;
	RTT::Seconds read_duration_;
	RTT::Seconds write_duration_;

	int steps_gz_;
	int steps_rtt_, last_steps_rtt_, nb_no_data_;
//	unsigned int n_joints_;
	int cnt_lock_;
	double period_sim_;
	double period_wall_;
	boost::atomic<bool> new_data, set_new_pos;
	boost::atomic<bool> rtt_done, gazebo_done;

	RTT::nsecs last_data_timestamp, data_timestamp, last_timestamp;bool set_brakes;
	int nb_static_joints;

	int nb_cmd_received_;bool sync_with_cmds_;

	// contains the urdf string for the associated model.
	std::string urdf_string;
};

ORO_LIST_COMPONENT_TYPE(UR5RttGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();

