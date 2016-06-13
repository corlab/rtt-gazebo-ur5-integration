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

class PIDController: public RTT::TaskContext {
public:
    PIDController(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    RTT::OutputPort<std::vector<double>> cmdJntTrq_Port;


    RTT::InputPort<std::vector<double>> currJntPos_Port;
    RTT::FlowStatus currJntPos_Flow;

    RTT::InputPort<std::vector<double>> refJntPos_Port;
    RTT::FlowStatus refJntPos_Flow;

    std::vector<double> trqCmdOutput;
    std::vector<double> currPosition;
    std::vector<double> targetPosition;

    // Variables for PID controller : transform to vector for several joints.
    std::vector<double> error_value;
    std::vector<double> cumulative_error;
    std::vector<double> last_error;
    double dynStepSize;
    std::vector<double> Kp;
    std::vector<double> Kd;
    std::vector<double> Ki;
    int nb_joints;


};
#endif
