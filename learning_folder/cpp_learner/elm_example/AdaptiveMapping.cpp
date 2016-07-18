#include "AdaptiveMapping.h"

#include <stdexcept>

AdaptiveMapping::AdaptiveMapping(unsigned int inDim, unsigned int outDim) : Mapping(inDim,outDim), param(), rate(1.0) {
	setAdaptionRate(1.0);
}

AdaptiveMapping::~AdaptiveMapping(){}

// ======= Supervised Learning ========
// push mapping-output (for <input>) toward target vector
void AdaptiveMapping::adaptToTarget(RealVectorPtr input, RealVectorPtr target, double weight){
	adaptByError(input, target->minus(evaluate(input)), weight);
}
// error = target-actual
//void AdaptiveMapping::adaptByError(RealVectorPtr input, RealVectorPtr error, double weight){}

// push mapping-output (for <input>) toward target vector
void AdaptiveMapping::adaptToTargets(TimeSeriesPtr input, TimeSeriesPtr targetOutput, TimeSeriesPtr weights){
	if(weights){
		for(unsigned int i=0; i<input->getLength(); i++){
			double weight = weights->getValue(i)->getValues()[0];
			adaptToTarget(input->getValue(i), targetOutput->getValue(i), weight);
		}
	}
	else{
		for(unsigned int i=0; i<input->getLength(); i++){
			adaptToTarget(input->getValue(i), targetOutput->getValue(i));
		}
	}
}

// error = target-actual
void AdaptiveMapping::adaptByError(TimeSeriesPtr input, TimeSeriesPtr error, TimeSeriesPtr weights){
	if(weights){
		for(unsigned int i=0; i<input->getLength(); i++){
			double weight = weights->getValue(i)->getValues()[0];
			adaptByError(input->getValue(i), error->getValue(i), weight);
		}
	}
	else{
		for(unsigned int i=0; i<input->getLength(); i++){
			adaptByError(input->getValue(i), error->getValue(i));
		}
	}
}

// ======= Parameters of adaption/learning process ========
void AdaptiveMapping::setAdaptionParameter(std::string name, double value){
	param[name] = value;
	if(name=="rate"){
		this->rate = value;
	}
}
double AdaptiveMapping::adaptionParameter(std::string name){
	if(0 < param.count(name))
		return param[name];
	else
		throw std::invalid_argument(std::string("No such parameter: ")+name);
}
void AdaptiveMapping::setAdaptionRate(double value){
	if(value < 0.0){
		throw std::invalid_argument("Only non-negative learning rates allowed");
	}
	setAdaptionParameter("rate", value);
}
double AdaptiveMapping::getAdaptionRate(){
	return this->rate;
}




