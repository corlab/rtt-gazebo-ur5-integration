#include <iostream>

#include "RealVector.h"
#include "ExtremeLearningMachine.h"

int main()
{
    std::cout << "ELM test 8-) " << std::endl;

    std::string infile("../../elmmodel/data");

    ExtremeLearningMachinePtr elm=ExtremeLearningMachine::create(infile);

    std::cout << "Generating testdata with dimensionality: " << elm->getInputDimension() << std::endl;
    std::cout << "Expecting results with dimensionality: " << elm->getOutputDimension() << std::endl;


    RealVectorPtr inputdata = RealVector::create(elm->getInputDimension(), 0.0);

    double vmin=-3;
    double vmax=3;

    for (int i=0; i<=100; i++)
    {
        double cval = (((vmax-vmin)*i)/100.0) + vmin;
        for (int j=0; j<inputdata->getDimension(); j++) inputdata->setValueEquals(j,cval);


        RealVectorPtr result = elm->evaluate(inputdata);

        std::cout << "Evaluating [" << inputdata << "] -> [" <<  result << "]" << std::endl;
    };


    return 0;
}

