#include <rtt/Component.hpp>


#include "pid_component.hpp"


PIDController::PIDController(std::string const& name) : RTT::TaskContext(name), nb_joints(0) , dynStepSize(0.0005) , refJntPos_Flow(RTT::NoData) , currJntPos_Flow(RTT::NoData) , targetPosition(0), error_value(0), cumulative_error(0), last_error(0), dynStepSize(5) , Kp({10000 , 15000  , 15000 , 2700 , 2700 , 2700 }) , Ki({2 , 2 , 2 , 1 , 1 , 1 }) , Kd({209250 ,209250 , 209250 , 209250 , 209250 , 209250})  {

	this->addPort("cmdJntTrq", cmdJntTrq_Port);
    trqCmdOutput = {0 , 0 ,0 ,0 ,0 , 0};
    cmdJntTrq_Port.setDataSample(trqCmdOutput);

    this->addPort("currJntPos", currJntPos_Port);

    this->addPort("refJntPos", refJntPos_Port);

}
// dynStepsize at frequency of the component.

bool PIDController::configureHook() {


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

bool PIDController::startHook() {
    return true;
}

void PIDController::updateHook() {
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

void PIDController::stopHook() {
	return ;
}

void PIDController::cleanupHook() {
	return ;
}

ORO_CREATE_COMPONENT(PIDController);
ORO_LIST_COMPONENT_TYPE(PIDController);
ORO_CREATE_COMPONENT_LIBRARY()
