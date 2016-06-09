#include <rtt/Component.hpp>
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

#include "pid_component.hpp"


PIDController::PIDController(std::string const& name) : RTT::TaskContext(name), nb_joints(0) , dynStepSize(0.0005) , refJntPos_Flow(RTT::NoData) , currJntPos_Flow(RTT::NoData) , targetPosition(0), error_value(0), cumulative_error(0), last_error(0),  Kp({10000 , 15000  , 15000 , 2700 , 2700 , 2700 }) , Ki({2 , 2 , 2 , 1 , 1 , 1 }) , Kd({209250 ,209250 , 209250 , 209250 , 209250 , 209250})  {

	this->addPort("cmdJntTrq", cmdJntTrq_Port);
    trqCmdOutput = {0 , 0 ,0 ,0 ,0 , 0};
    cmdJntTrq_Port.setDataSample(trqCmdOutput);

    this->addPort("currJntPos", currJntPos_Port);

    this->addPort("refJntPos", refJntPos_Port);

}
// dynStepsize at frequency of the component.

/*
bool PIDController::gazeboConfigureHook(gazebo::physics::ModelPtr model) {


    if (!(cmdJntTrq_Port.connected() && currJntPos_Port.connected()  && refJntPos_Port.connected() )) {
        return false;
    }
    nb_joints = 6;

    for (unsigned j = 0; j < nb_joints; j++)
   {
		error_value.push_back(0);
		cumulative_error.push_back(0);
		last_error.push_back(0);
		trqCmdOutput.push_back(0);
		targetPosition.push_back(0);
   }

    return true;
}
*/

bool PIDController::startHook() {
    return true;
}

/*
void PIDController::gazeboUpdateHook(gazebo::physics::ModelPtr model) {
	refJntPos_Flow = refJntPos_Port.read(targetPosition);

    currJntPos_Flow = currJntPos_Port.read(currPosition);
    if (currJntPos_Flow == RTT::NewData)
    {
    	// Command computation.
    	for (unsigned j = 0; j < nb_joints; j++)
    	{
			error_value[j] = targetPosition[j] -  currPosition[j];
			trqCmdOutput[j] = error_value[j]*Kp[j];
			cumulative_error[j] = cumulative_error[j] + error_value[j];
			trqCmdOutput[j] = trqCmdOutput[j] + cumulative_error[j]*Ki[j]*dynStepSize;
			trqCmdOutput[j] = trqCmdOutput[j] + (error_value[j]-last_error[j])*(Kd[j]/dynStepSize);
			last_error[j] = error_value[j];
    	}
    }

    if (cmdJntTrq_Port.connected()) {
        cmdJntTrq_Port.write(trqCmdOutput);
    }
}
*/

void PIDController::stopHook() {
	return ;
}

void PIDController::cleanupHook() {
	return ;
}

bool PIDController::configureHook() {
	RTT::log(RTT::Error) << "Beginning PIDController configuration." << RTT::endlog();

    if (!(cmdJntTrq_Port.connected() && currJntPos_Port.connected()  && refJntPos_Port.connected() )) {
        return false;
    }
    nb_joints = 6;

    for (unsigned j = 0; j < nb_joints; j++)
   {
		error_value.push_back(0);
		cumulative_error.push_back(0);
		last_error.push_back(0);
		trqCmdOutput.push_back(0);
		targetPosition.push_back(0);
   }

	RTT::log(RTT::Error) << "PIDController configured." << RTT::endlog();

    return true;
}

void PIDController::updateHook() {

	//RTT::log(RTT::Error) << "Beginning PIDController update." << RTT::endlog();

	/*
	refJntPos_Flow = refJntPos_Port.read(targetPosition);

	    currJntPos_Flow = currJntPos_Port.read(currPosition);
	    if (currJntPos_Flow == RTT::NewData)
	    {
	    	// Command computation.
	    	for (unsigned j = 0; j < nb_joints; j++)
	    	{
				error_value[j] = targetPosition[j] -  currPosition[j];
				trqCmdOutput[j] = error_value[j]*Kp[j];
				cumulative_error[j] = cumulative_error[j] + error_value[j];
				trqCmdOutput[j] = trqCmdOutput[j] + cumulative_error[j]*Ki[j]*dynStepSize;
				trqCmdOutput[j] = trqCmdOutput[j] + (error_value[j]-last_error[j])*(Kd[j]/dynStepSize);
				last_error[j] = error_value[j];
	    	}
	    }

	    if (cmdJntTrq_Port.connected()) {
	        cmdJntTrq_Port.write(trqCmdOutput);
	    }
	    */
	//RTT::log(RTT::Error) << "PIDController updated." << RTT::endlog();

}


ORO_LIST_COMPONENT_TYPE(PIDController);
ORO_CREATE_COMPONENT_LIBRARY()
