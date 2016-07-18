
#include "ExtremeLearningMachine.h"
#include <iostream>
#include <fstream>




double fermi(double x){
	return 1.0 / (1.0 + exp(-x));
}

ExtremeLearningMachinePtr ExtremeLearningMachine::create(std::string folderName){

    // try loading 'info'
    std::string infoFileName = folderName+std::string("/../info");
    const unsigned int maxChars = 128;
    char infoChars[maxChars];
    std::ifstream infoFile( (const char*) infoFileName.c_str() );
    infoFile.getline(infoChars, maxChars-1);
    infoFile.close();
    std::string info = infoChars;

    if(info == "elm")
    {
        RealMatrixPtr wIn = RealMatrix::load(folderName+std::string("/Win.mat"));
        RealVectorPtr a = RealVector::load(folderName+std::string("/a.vec"));
        RealVectorPtr b = RealVector::load(folderName+std::string("/b.vec"));
        RealMatrixPtr wOut = RealMatrix::load(folderName+std::string("/Wout.mat"));

        unsigned int out_dimensionality = wOut->getNumRows();
        unsigned int hid_dimensionality = wIn->getNumRows();
        unsigned int in_dimensionality = wIn->getNumCols();

        std::cout << "Found ELM model (inp-dim:"<< in_dimensionality<<"; hid-dim:"<< hid_dimensionality <<"out-dim:"<< out_dimensionality <<")." << std::endl;
        std::cerr << "ELM model without datascale is not implemented :(" << std::endl;

        return ExtremeLearningMachine::create(wIn, a, b, wOut);

    }
    else if(info == "elm+datascale")
    {
        RealMatrixPtr wIn = RealMatrix::load(folderName+std::string("/Win.mat"));
        RealVectorPtr a = RealVector::load(folderName+std::string("/a.vec"));
        RealVectorPtr b = RealVector::load(folderName+std::string("/b.vec"));
        RealMatrixPtr wOut = RealMatrix::load(folderName+std::string("/Wout.mat"));
        //wIn=wIn->transpose();
        //wOut=wOut->transpose();

        RealVectorPtr bx = RealVector::load(folderName+std::string("/bx.vec"));
        RealMatrixPtr Ax = RealMatrix::load(folderName+std::string("/Ax.mat"));
        RealVectorPtr by = RealVector::load(folderName+std::string("/by.vec"));
        RealMatrixPtr Ay = RealMatrix::load(folderName+std::string("/Ay.mat"));


        unsigned int out_dimensionality = wOut->getNumRows();
        unsigned int hid_dimensionality = wIn->getNumRows();
        unsigned int in_dimensionality = wIn->getNumCols();

        std::cout << "Found ELM model + datascale (inp-dim:"<< in_dimensionality<<"; hid-dim:"<< hid_dimensionality <<"out-dim:"<< out_dimensionality <<")." << std::endl;


        return ExtremeLearningMachine::create(wIn, a, b, wOut, Ax, bx, Ay, by);


    }
    else
    {
        std::cerr << "Model does not exist of is of unknown type." << std::endl;
    }
}

ExtremeLearningMachinePtr ExtremeLearningMachine::create(RealMatrixPtr wIn, RealVectorPtr a, RealVectorPtr b, RealMatrixPtr wOut){
	if( wIn->getNumRows() != a->getDimension() ){
		throw std::invalid_argument("wIn->getNumRows() != a->getDimension()");
	}
	if( a->getDimension() != b->getDimension() ){
		throw std::invalid_argument("a->getDimension() != b->getDimension()");
	}
	if( b->getDimension() != wOut->getNumCols() ){
		throw std::invalid_argument("b->getDimension() != wOut->getNumCols()");
	}
	return ExtremeLearningMachinePtr( new ExtremeLearningMachine(wIn, a, b, wOut) );

}

ExtremeLearningMachinePtr ExtremeLearningMachine::create(RealMatrixPtr wIn, RealVectorPtr a, RealVectorPtr b, RealMatrixPtr wOut, RealMatrixPtr Ax, RealVectorPtr bx, RealMatrixPtr Ay, RealVectorPtr by)
{
	if( wIn->getNumRows() != a->getDimension() ){
	throw std::invalid_argument("wIn->getNumRows() != a->getDimension()");
	}
	if( a->getDimension() != b->getDimension() ){
		throw std::invalid_argument("a->getDimension() != b->getDimension()");
	}
	if( b->getDimension() != wOut->getNumCols() ){
		throw std::invalid_argument("b->getDimension() != wOut->getNumCols()");
	}
	

	
	return ExtremeLearningMachinePtr( new ExtremeLearningMachine(wIn, a, b, wOut, Ax, bx, Ay, by) );
}

ExtremeLearningMachinePtr ExtremeLearningMachine::create(RealMatrixPtr wIn, RealVectorPtr a, RealVectorPtr b, RealMatrixPtr wOut, int selOutStart, int selOutEnd)
{
	ExtremeLearningMachinePtr res=ExtremeLearningMachine::create(wIn, a, b, wOut);
	res->selOutStart=selOutStart;
	res->selOutEnd=selOutEnd;
	
	return res;

}

RealVectorPtr ExtremeLearningMachine::evaluate(RealVectorPtr length){
	RealVectorPtr res;



        res = Ay->mult(wOut->mult( nonLinearity->evaluate( a->componentMult(wIn->mult(  (Ax->mult(length->minus(bx))->componentMult(in2)->minus(in1))   )->minus(b)) ) )->plus(out1)->componentDiv(out2) )->plus(by);


	
	
	
	if (selOutStart>=0 && selOutEnd>=selOutStart )
	{
		int outdims=selOutEnd-selOutStart;
		//Select output dimensions
		const double* vals = res->getValues();
		RealVectorPtr nres= RealVector::create(outdims, vals+selOutStart);
		return nres;
	}

	return res;
}

ExtremeLearningMachine::ExtremeLearningMachine(RealMatrixPtr wInP, RealVectorPtr aP, RealVectorPtr bP, RealMatrixPtr wOutP)
 : Mapping(wInP->getNumCols(), wOutP->getNumRows()), wIn(wInP), a(aP), b(bP), wOut(wOutP), selOutStart(-1),selOutEnd(-1), nonLinearity(PointWiseMapping::create(a->getDimension(), fermi))
{}


ExtremeLearningMachine::ExtremeLearningMachine(RealMatrixPtr wInP, RealVectorPtr aP, RealVectorPtr bP, RealMatrixPtr wOutP, RealMatrixPtr AxP, RealVectorPtr bxP, RealMatrixPtr AyP, RealVectorPtr byP)
 : Mapping(wInP->getNumCols(), wOutP->getNumRows()), wIn(wInP), a(aP), b(bP), wOut(wOutP), Ax(AxP), bx(bxP), Ay(AyP), by(byP), selOutStart(-1),selOutEnd(-1), nonLinearity(PointWiseMapping::create(a->getDimension(), fermi))
{


    in1=RealVector::create(bxP->getDimension(), 1.0);
    in2=RealVector::create(bxP->getDimension(), 2.0);
    out1=RealVector::create(byP->getDimension(), 1.0);
    out2=RealVector::create(byP->getDimension(), 2.0);
	
	
}

