#ifndef RTT_JPC_HPP
#define RTT_JPC_HPP
// RTT includes
#include <rtt/RTT.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
// RCI includes
#include <rci/dto/JointAngles.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointTorques.h>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolver.hpp>

// BOOST includes
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
// Parser include convert URDF/SDF into KDL::Chain
//#include "parsertools/KDLParser.hpp"

class UR5RttGazeboComponent: public RTT::TaskContext {
public:

	virtual bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
	virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model);
	UR5RttGazeboComponent(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void eeMass(double mass , gazebo::physics::ModelPtr model);


    RTT::InputPort<std::vector<double>> cmdJntTrq_Port;
    RTT::FlowStatus cmdJntTrq_Flow;

    RTT::InputPort<std::vector<double>> trgtPos_Port;
    RTT::FlowStatus trgtPos_Flow;

    RTT::InputPort<std::vector<double>> compPos_Port;
    RTT::FlowStatus compPos_Flow;

    RTT::OutputPort<std::vector<double>> currJntPos_Port;
    RTT::OutputPort<std::vector<double>> refJntPos_Port;
    RTT::OutputPort<std::vector<double>> currJntTrq_Port;
    RTT::OutputPort<std::vector<double>> meanJntTrq_Port;
    RTT::OutputPort<std::vector<double>> currVelocity_Port;
    RTT::OutputPort<double> currMass_Port;
    RTT::InputPort<double> newMass_Port;
    RTT::FlowStatus newMass_Flow;
    double new_mass;
    RTT::OutputPort<std::vector<double>> meanCollTrq_Port;
    std::vector<double> meanCollTrq;

    std::vector<double> trqCmdOutput;
    std::vector<double> currPosition;
    std::vector<double> targetPosition;
    std::vector<double> meanTrq;
    std::vector<double> currVelo;


	std::ofstream data_file;

	int nb_iteration; // number of hook iterations for one tested position.
	int sim_id; // number of angle positions tested.

	std::vector<int> joints_idx;
	std::vector<int> links_idx;
	std::vector<gazebo::physics::JointPtr> gazebo_joints_;
	gazebo::physics::Link_V model_links_;
	std::vector<std::string> joint_names_;
	std::vector<std::string> link_names_;

	// Variable to save intermediate robot position - to decide if the data will be written in the file.
	std::vector< std::vector<double> > inter_torque;


	double PIDStepSize;

	int nb_static_joints;
	int nb_links;
	double l1; // length of link1
	double l2;  // length of link2

	double curr_mass;
	// Variable to save intermediate robot position - to decide if the data will be written in the file.
	std::vector< std::vector<double> > inter_torque_elm;
	int elm_id;
};
#endif
