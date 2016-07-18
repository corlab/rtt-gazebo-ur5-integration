#ifndef _FFWD_PRESSURE_FROM_LENGTH_H_
#define _FFWD_PRESSURE_FROM_LENGTH_H_


#include "AdaptiveLinearMapping.h"

#include <vector>






class ExtremeLearningMachine;
typedef boost::shared_ptr<ExtremeLearningMachine> ExtremeLearningMachinePtr;

class ExtremeLearningMachine : public Mapping {
public:
    static ExtremeLearningMachinePtr create(std::string folderName);
	static ExtremeLearningMachinePtr create(RealMatrixPtr wIn, RealVectorPtr a, RealVectorPtr b, RealMatrixPtr wOut);
	static ExtremeLearningMachinePtr create(RealMatrixPtr wIn, RealVectorPtr a, RealVectorPtr b, RealMatrixPtr wOut, RealMatrixPtr Ax, RealVectorPtr bx, RealMatrixPtr Ay, RealVectorPtr by);
	static ExtremeLearningMachinePtr create(RealMatrixPtr wIn, RealVectorPtr a, RealVectorPtr b, RealMatrixPtr wOut, int selOutStart, int selOutEnd);

	RealVectorPtr evaluate(RealVectorPtr length);

private:
	ExtremeLearningMachine(RealMatrixPtr wIn, RealVectorPtr a, RealVectorPtr b, RealMatrixPtr wOut);
	ExtremeLearningMachine(RealMatrixPtr wIn, RealVectorPtr a, RealVectorPtr b, RealMatrixPtr wOut, RealMatrixPtr Ax, RealVectorPtr bx, RealMatrixPtr Ay, RealVectorPtr by);

	RealMatrixPtr wIn;
	RealVectorPtr a;
	RealVectorPtr b,bx,by;
    RealMatrixPtr wOut, Ax, Ay;
	MappingPtr nonLinearity;

    RealVectorPtr in1,in2, out1,out2;
	int selOutStart;
	int selOutEnd;
};

class LinearLearner;
typedef boost::shared_ptr<LinearLearner> LinearLearnerPtr;
class LinearLearner : public Mapping {
public:

	static LinearLearnerPtr create(RealMatrixPtr wOut);
	static LinearLearnerPtr create(RealMatrixPtr wOut, RealVectorPtr in_mean, RealVectorPtr in_std_dev, RealVectorPtr out_mean, RealVectorPtr out_std_dev);

	RealVectorPtr evaluate(RealVectorPtr length);

private:
	LinearLearner(RealMatrixPtr wOut);
	LinearLearner(RealMatrixPtr wOutP, RealVectorPtr in_mean, RealVectorPtr in_std_dev, RealVectorPtr out_mean, RealVectorPtr out_std_dev);

	RealMatrixPtr wOut;
	RealVectorPtr in_mean;
	RealVectorPtr in_std_dev;
	RealVectorPtr out_mean;
	RealVectorPtr out_std_dev;

};



#endif

