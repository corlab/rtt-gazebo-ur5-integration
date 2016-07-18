
#include "AdaptiveLinearMapping.h"

#include "VectorMath.h"


#include <iostream>


AdaptiveLinearMappingPtr AdaptiveLinearMapping::create(unsigned int inDim, unsigned int outDim, bool withOffset){
	return boost::shared_ptr<AdaptiveLinearMapping>(new AdaptiveLinearMapping(inDim, outDim, withOffset));
}

AdaptiveLinearMappingPtr AdaptiveLinearMapping::create(RealMatrixPtr weights, RealVectorPtr offset){
	if(offset != NULL && weights->getNumRows() != offset->getDimension()){
		throw std::invalid_argument("AdaptiveLinearMapping::create(RealMatrixPtr, RealVectorPtr): dimension mismatch");
	}
	return boost::shared_ptr<AdaptiveLinearMapping>(new AdaptiveLinearMapping(weights, offset));
}

AdaptiveLinearMappingPtr AdaptiveLinearMapping::solve(TimeSeriesPtr input, TimeSeriesPtr targetOutput, bool withOffset){
	return solve(RealMatrix::create(input), RealMatrix::create(targetOutput), withOffset);
}

AdaptiveLinearMappingPtr AdaptiveLinearMapping::solve(TimeSeriesPtr input, TimeSeriesPtr targetOutput, TimeSeriesPtr sampleWeights, bool withOffset){
	return solve(
		RealMatrix::create(multiplyWeights(input, sampleWeights)), 
		RealMatrix::create(multiplyWeights(targetOutput, sampleWeights)),
		withOffset
	);
}

AdaptiveLinearMappingPtr AdaptiveLinearMapping::solve(RealMatrixPtr input, RealMatrixPtr targetOutput, bool withOffset){
	if(withOffset){
		const unsigned int n = input->getNumCols();
		const unsigned int m = targetOutput->getNumCols();
		// attach 1 to input values
		// receive (W;b) (n+1)x(m)
		RealMatrixPtr weightsAndOffset = (input->attachCols(RealMatrix::create(input->getNumRows(), 1, 1.0))->pseudoInverse()
			->mult(targetOutput))->transpose();

		RealMatrixPtr weights = weightsAndOffset->submatrix(0, n, 0, m);
		RealVectorPtr offset = weightsAndOffset->getColVector(m);
			
		return boost::shared_ptr<AdaptiveLinearMapping>(new AdaptiveLinearMapping(weights, offset));
		
	}
	else{
		RealMatrixPtr weights = (input->pseudoInverse()->mult(targetOutput))->transpose();
		return boost::shared_ptr<AdaptiveLinearMapping>(new AdaptiveLinearMapping(weights, RealVectorPtr(NULL)));
	}
}

RealVectorPtr AdaptiveLinearMapping::evaluate(RealVectorPtr value){
	if(offset){
		//return matrix->mult(value)->plus(offset);
		return VectorMath::matrixMultAdd(matrix, value, offset);
	}
	else{
		return matrix->mult(value);
	}
}

MappingPtr AdaptiveLinearMapping::invert() {
	RealMatrixPtr newMat = this->matrix->pseudoInverse();
	RealVectorPtr newOffset;
	if(offset != NULL){
		newOffset = newMat->mult(this->offset->scale(-1.0));
	}
	return AdaptiveLinearMapping::create(newMat, newOffset);
}

RealMatrixPtr AdaptiveLinearMapping::getJacobian(RealVectorPtr){
	return this->matrix;
}

void AdaptiveLinearMapping::adaptToTarget(RealVectorPtr input, RealVectorPtr target, double weight){
	adaptByError(input, target->minus(evaluate(input)), weight);
}

// error = target-actual
void AdaptiveLinearMapping::adaptByError(RealVectorPtr input, RealVectorPtr error, double weight){
	//RealMatrixPtr deltaW = VectorMath::vecMultVecTScale(error, input, adaptionParameter("rate")*weight);
	//this->matrix = matrix->plus(deltaW);

	double learningRate = getAdaptionRate()*weight;

	VectorMath::matPlusvecMultVecTScale(this->matrix, error, input, learningRate, this->matrix);

	if(offset){
		RealVectorPtr deltaOffset = error;
		//this->offset = offset->plus( deltaOffset->scale(rate );
		this->offset = VectorMath::addScaled(offset, learningRate, deltaOffset);
	}
}

void AdaptiveLinearMapping::setWeightMatrix(RealMatrixPtr mat){
	if(!mat){
		throw std::invalid_argument("AdaptiveLinearMapping::setWeightMatrix(RealMatrixPtr): null pointer");
	}
	this->matrix = mat;
}
RealMatrixPtr AdaptiveLinearMapping::getWeightMatrix(){
	return RealMatrix::create(this->matrix);	
}
void AdaptiveLinearMapping::setOffsetVector(RealVectorPtr vec){
	if(!vec){
		throw std::invalid_argument("AdaptiveLinearMapping::setOffsetVector(RealVectorPtr): null pointer");
	}
	this->offset = vec;
}
RealVectorPtr AdaptiveLinearMapping::getOffsetVector(){
	return this->offset;	
}

// push mapping-output (for <input>) toward target vector
void AdaptiveLinearMapping::adaptToTargets(TimeSeriesPtr input, TimeSeriesPtr targetOutput, TimeSeriesPtr weights){
	// FIXME check dimensions
	RealMatrixPtr inputMat = RealMatrix::create(input);
	RealMatrixPtr targetMat = RealMatrix::create(targetOutput);

	TimeSeriesPtr outputSeries = input->map(this->getTempPointer());

	const unsigned int N = input->getLength();
	const unsigned int D = outputSeries->getValue(0)->getDimension();

	const double learningRate = getAdaptionRate();
	// fixed: this was a stack array: double errorData[N*D]
	// stupid idea, used to be too big and caused segfaults
	double *errorData = new double[N*D];
	for(unsigned int t=0; t<N; t++){
		RealVectorPtr error = targetOutput->getValue(t)->minus(outputSeries->getValue(t));
		if(weights != NULL){
			error = error->scale( weights->getValue(t)->getValues()[0] );
		}
		const double *errorVal = error->getValues();
		for(unsigned int d=0; d<D; d++){
			errorData[d*N+t] = errorVal[d] / N * learningRate;
		}
	}
	RealMatrixPtr errorMatT = RealMatrix::create(D, N, errorData);
	delete[](errorData);

	RealMatrixPtr deltaW = errorMatT->mult(inputMat);
	this->matrix = matrix->plus( deltaW );
	if(offset){
		RealVectorPtr deltaOffset = errorMatT->mult(RealVector::create(N, 1.0));
		this->offset = offset->plus( deltaOffset );
	}
}

void AdaptiveLinearMapping::adaptByError(TimeSeriesPtr input, TimeSeriesPtr error, TimeSeriesPtr weights){
	// FIXME check dimensions
	RealMatrixPtr inputMat = RealMatrix::create(input);

	const unsigned int N = input->getLength();
	const unsigned int D = error->getValue(0)->getDimension();

	const double learningRate = getAdaptionRate();
	double *errorData = new double[N*D];
	for(unsigned int t=0; t<N; t++){
		const double *errorVal = error->getValue(t)->getValues();
		if(weights != NULL){
			double w = weights->getValue(t)->getValues()[0];
			for(unsigned int d=0; d<D; d++){
				errorData[d*N+t] = errorVal[d] / N * learningRate * w;
			}
		}
		else{
			for(unsigned int d=0; d<D; d++){
				errorData[d*N+t] = errorVal[d] / N * learningRate;
			}
		}
	}
	RealMatrixPtr errorMatT = RealMatrix::create(D, N, errorData);
	delete[](errorData);

	RealMatrixPtr deltaW = errorMatT->mult(inputMat);
	this->matrix = matrix->plus( deltaW );
	if(offset){
		RealVectorPtr deltaOffset = errorMatT->mult(RealVector::create(N, 1.0));
		this->offset = offset->plus( deltaOffset );
	}
}

AdaptiveLinearMapping::~AdaptiveLinearMapping(){
}

AdaptiveLinearMapping::AdaptiveLinearMapping(unsigned int inDim, unsigned int outDim, bool withOffset)
 : AdaptiveMapping(inDim, outDim), matrix(RealMatrix::create(outDim,inDim)), offset(NULL) {
	if(withOffset){
		offset = RealVector::create(outDim, 0.0);
	}
	setAdaptionParameter("rate", 0.05);
}

AdaptiveLinearMapping::AdaptiveLinearMapping(RealMatrixPtr weights, RealVectorPtr off)
 : AdaptiveMapping(weights->getNumCols(), weights->getNumRows()), matrix(weights), offset(off) {
	setAdaptionParameter("rate", 0.05);
}

struct null_deleter{
	void operator()(void const *) const{}
};
MappingPtr AdaptiveLinearMapping::getTempPointer(){
	return MappingPtr(this, null_deleter());
}

TimeSeriesPtr AdaptiveLinearMapping::multiplyWeights(TimeSeriesPtr data, TimeSeriesPtr weights){
	TimeSeriesPtr weighted = TimeSeries::create();
	for(unsigned int t=0; t<data->getLength(); t++){
		double w = weights->getValue(t)->getValues()[0];
		weighted->attach(data->getTimestamp(t), data->getValue(t)->scale(w));
	}
	return weighted;
}



