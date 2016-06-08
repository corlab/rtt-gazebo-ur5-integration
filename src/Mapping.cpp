#include "Mapping.h"


#include <iostream>
using namespace std;


unsigned int binomial(unsigned int N, unsigned int k);



MappingPtr Mapping::concat(MappingPtr mapping){
	return SequentialMapping::create(this->clone(), mapping);
}
MappingPtr Mapping::add(RealVectorPtr vec){
	return this->concat(ComponentLinearMapping::add(vec));
}
MappingPtr Mapping::add(double num){
	return this->concat(ComponentLinearMapping::add(RealVector::create(getOutputDimension(), num)));
}
MappingPtr Mapping::scale(double num){
	return this->concat(ComponentLinearMapping::scale(getOutputDimension(), num));
}
MappingPtr Mapping::mult(RealMatrixPtr mat){
	return this->concat( LinearMapping::create(mat, RealVector::create(mat->getNumRows(), 0.0)) );
}
MappingPtr Mapping::componentWise(double (*function)(double), double (*inverse)(double)){
	return this->concat( PointWiseMapping::create(getOutputDimension(), function, inverse) );
}





MappingPtr Identity::create(unsigned int dimension){
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<Identity>(
			new Identity(dimension)
		)
	);
}
RealVectorPtr Identity::evaluate(RealVectorPtr value){
	return value;
}
MappingPtr Identity::invert(){
	return create(getInputDimension());
}
MappingPtr Identity::clone(){
	return Identity::create(getInputDimension());
}
RealMatrixPtr Identity::getJacobian(RealVectorPtr){
	return RealMatrix::identity(getInputDimension());
}
Identity::Identity(int dim) : Mapping(dim, dim) {}




MappingPtr Zero::create(unsigned int dimension){
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<Zero>(
			new Zero(dimension)
		)
	);
}
MappingPtr Zero::create(unsigned int inDim, unsigned int outDim){
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<Zero>(
			new Zero(inDim, outDim)
		)
	);
}
RealVectorPtr Zero::evaluate(RealVectorPtr){
	return RealVector::create(getOutputDimension(), 0.0);
}
MappingPtr Zero::clone(){
	return Zero::create(getInputDimension(), getOutputDimension());
}
RealMatrixPtr Zero::getJacobian(RealVectorPtr value){
	return RealMatrix::create(getOutputDimension(), value->getDimension(), 0.0);
}

Zero::Zero(unsigned int dim) : Mapping(dim,dim) {}
Zero::Zero(unsigned int inDim, unsigned int outDim) : Mapping(inDim,outDim) {}

Modulo::Modulo(unsigned int dim, double mod)
 : Mapping(dim, dim), _mod(mod) 
{}

MappingPtr Modulo::create(unsigned int dim, double mod) {
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<Modulo>(new Modulo(dim, mod)));
}

RealVectorPtr Modulo::evaluate(RealVectorPtr value) {
	const double *oldData = value->getValues();
	double *newData = new double[this->getOutputDimension()];

	for(unsigned int index = 0; index < this->getOutputDimension(); index++){
		newData[index] = (oldData[index] - (floor(oldData[index] / this->_mod)
				* this->_mod));
	}

	RealVectorPtr result = RealVector::create(this->getOutputDimension(), newData);

	delete[](newData);
	return result;
}

MappingPtr Modulo::clone() {
	return Modulo::create(getInputDimension(), this->_mod);
}
RealMatrixPtr Modulo::getJacobian(RealVectorPtr){
	// Jacobian is Identity at all differentiable points
	return RealMatrix::identity(getInputDimension());
}

MappingPtr ComponentLinearMapping::scale(unsigned int dim, double alpha){
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<ComponentLinearMapping>(
			new ComponentLinearMapping(
				RealVector::create(dim, alpha),
				RealVector::create(dim, 0.0)
			)
		)
	);
}
MappingPtr ComponentLinearMapping::add(RealVectorPtr vec){
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<ComponentLinearMapping>(
			new ComponentLinearMapping(
				RealVector::create(vec->getDimension(), 1.0),
				vec
			)
		)
	);
}
MappingPtr ComponentLinearMapping::create(RealVectorPtr gain, RealVectorPtr bias){
	if(gain==NULL || bias==NULL){
		throw std::invalid_argument(
			"gain and slope must not be NULL");
	}
	if(gain->getDimension() != bias->getDimension()){
		throw std::invalid_argument(
			"gain and slope must have same dimension");
	}
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<ComponentLinearMapping>(
			new ComponentLinearMapping(gain, bias)
		)
	);
}
RealVectorPtr ComponentLinearMapping::evaluate(RealVectorPtr value){
	if(value->getDimension() != getInputDimension()){
		std::stringstream mess;
		mess << "ComponentLinearMapping::evaluate(): ";
		mess << "Wrong input dimension: ";
		mess << "expected " << getInputDimension();
		mess << ", but is " << value->getDimension() << ".";
		throw std::invalid_argument(mess.str());
	}

	return value->componentMult(gain)->plus(bias);
}
MappingPtr ComponentLinearMapping::invert(){
	// only, if no gain is zero!
	for(unsigned int i=0; i<gain->getDimension(); i++){
		if(gain->getValues()[i] == 0.0){
			throw std::logic_error("gain 0.0 not invertable");
		}
	}

	RealVectorPtr newGain = 
		RealVector::create(gain->getDimension(), 1.0)->
		componentDiv(gain);
	RealVectorPtr newBias = bias->componentDiv(gain)->scale(-1.0);
	return ComponentLinearMapping::create(newGain, newBias);
}
MappingPtr ComponentLinearMapping::clone(){
	return ComponentLinearMapping::create(gain,bias);
}
RealMatrixPtr ComponentLinearMapping::getJacobian(RealVectorPtr){
	return RealMatrix::diagonal(this->gain);
}
ComponentLinearMapping::ComponentLinearMapping(RealVectorPtr g, RealVectorPtr b)
 : Mapping(g->getDimension(), g->getDimension()), gain(g), bias(b) {}




MappingPtr PointWiseMapping::create(unsigned int dimension, double (*function)(double), double (*inverseFunction)(double)){
	if(function==NULL){
		throw std::invalid_argument(
			"Function pointer must not be NULL for point-wise mapping");
	}
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<PointWiseMapping>(
			new PointWiseMapping(dimension, function, inverseFunction)
		)
	);
}
RealVectorPtr PointWiseMapping::evaluate(RealVectorPtr value){
	if(value->getDimension() != getInputDimension()){
		std::stringstream mess;
		mess << "PointWiseMapping::evaluate(): ";
		mess << "Wrong input dimension: ";
		mess << "expected " << getInputDimension();
		mess << ", but is " << value->getDimension() << ".";
		throw std::invalid_argument(mess.str());
	}

	double *values = new double[getInputDimension()];
	value->getValues(values);
	for(unsigned int i=0; i<getInputDimension(); i++){
		values[i] = function(values[i]);
	}
	RealVectorPtr result = RealVector::create(getInputDimension(), values);
	delete[](values);

	return result;
}
MappingPtr PointWiseMapping::invert(){
	if(inverseFunction == NULL){
		throw std::logic_error("No inverse specified for point-wise mapping.");
	}

	return create(getInputDimension(), inverseFunction, function);
}
MappingPtr PointWiseMapping::clone(){return PointWiseMapping::create(getInputDimension(),function,inverseFunction);}
PointWiseMapping::PointWiseMapping(unsigned int dimension, double (*func)(double), double (*inverseFunc)(double))
	: Mapping(dimension, dimension), function(func), inverseFunction(inverseFunc) {}




MappingPtr SequentialMapping::create(MappingPtr first, MappingPtr second){
	if(first->getOutputDimension() != second->getInputDimension()){
		throw std::invalid_argument(
			"outputDim of first mapping must be equal to inputDim of second");
	}
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<SequentialMapping>(
			new SequentialMapping(first, second)
		)
	);
}
RealVectorPtr SequentialMapping::evaluate(RealVectorPtr value){
	/*if(value->getDimension() != getInputDimension()){
		std::stringstream mess;
		mess << "SequentialMapping::evaluate(): ";
		mess << "Wrong input dimension: ";
		mess << "expected " << getInputDimension();
		mess << ", but is" << value->getDimension() << ".";
		throw std::invalid_argument(mess.str());
	}*/

	return secondMapping->evaluate(firstMapping->evaluate(value));

	/*
	// best programming error ever!!!!
	// if firstMapping or secondMapping throw an exception, the program ends up with a
	// segfault inside the boost/smart_ptr/details/sp_counted_base
	// reason: "value" masks input argument of function 
	try{
		RealVectorPtr value = secondMapping->evaluate(firstMapping->evaluate(value));
		return value;
	}
	catch(std::exception &e){
		std::stringstream mess;
		mess << "Exception in SequentialMapping::evaluate(): ";
		mess << e.what();
		throw std::invalid_argument(mess.str());
	}*/
	
}
MappingPtr SequentialMapping::invert(){
	MappingPtr firstInv = firstMapping->invert();
	MappingPtr secondInv = secondMapping->invert();
	return create(secondInv, firstInv);
}
MappingPtr SequentialMapping::clone(){
	return SequentialMapping::create(firstMapping,secondMapping);
}
RealMatrixPtr SequentialMapping::getJacobian(RealVectorPtr value){
	RealMatrixPtr firstJacobian = firstMapping->getJacobian(value);
	RealMatrixPtr secondJacobian = secondMapping->getJacobian(firstMapping->evaluate(value));
	return secondJacobian->mult(firstJacobian);
}
SequentialMapping::SequentialMapping(MappingPtr first, MappingPtr second)
	: Mapping(first->getInputDimension(), second->getOutputDimension()),
	firstMapping(first), secondMapping(second) {}




MappingPtr SubspaceMapping::create(unsigned int inputDim, unsigned int startIndex, unsigned int outputDim){
	if(inputDim < startIndex+outputDim){
		throw std::invalid_argument(
			"too few input dimensions");
	}
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<SubspaceMapping>(
			new SubspaceMapping(inputDim, startIndex, outputDim)
		)
	);
}
MappingPtr SubspaceMapping::create(unsigned int inputDim, bool *dimSelection){
	if(dimSelection == NULL){
		throw std::invalid_argument(
			"NULL pointer to dimension selection array");
	}
	int outDimCount = 0;
	for(unsigned int i=0; i<inputDim; i++){
		if(dimSelection[i]){
			outDimCount++;
		}
	}
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<SubspaceMapping>(
			new SubspaceMapping(inputDim, outDimCount, dimSelection)
		)
	);
}
RealVectorPtr SubspaceMapping::evaluate(RealVectorPtr value){
	if(value->getDimension() != getInputDimension()){
		std::stringstream mess;
		mess << "SubspaceMapping::evaluate(): ";
		mess << "Wrong input dimension: ";
		mess << "expected " << getInputDimension();
		mess << ", but is " << value->getDimension() << ".";
		throw std::invalid_argument(mess.str());
	}
	if(dimSelection == NULL){
		return value->subspace(start, outDim);
	}
	else{
		const double *inputData = value->getValues();
		double *subspaceData = new double[getOutputDimension()];
		int outDimCount = 0;
		for(unsigned int i=0; i<getInputDimension(); i++){
			if(dimSelection[i]){
				subspaceData[outDimCount++] = inputData[i];
			}
		}
		RealVectorPtr outVec = RealVector::create(getOutputDimension(), subspaceData);
		delete[](subspaceData);
		return outVec;
	}
}
MappingPtr SubspaceMapping::clone(){
	return SubspaceMapping::create(getInputDimension(), start, outDim);
}
RealMatrixPtr SubspaceMapping::getJacobian(RealVectorPtr){
	const unsigned int numCols = getInputDimension();
	const unsigned int numRows = getOutputDimension();
	double *values = new double[numRows*numCols];
	// each row contains exactly one 1.0
	for(unsigned int i=0; i<numRows*numCols; i++){
		values[i] = 0.0;
	}
	if(dimSelection == NULL){
		for(unsigned int row=0; row < numRows; row++){
			const unsigned int col = row+start;
			values[ row*numCols + col ] = 1.0;
		}
	}
	else{
		unsigned int row = 0;
		for(unsigned int col=0; col<numCols; col++){
			if(dimSelection[col]){
				values[ row*numCols + col ] = 1.0;
				row++;
			}
		}
	}
	RealMatrixPtr jacobian = RealMatrix::create(numRows, numCols, values);
	delete[](values);
	return jacobian;
}
SubspaceMapping::~SubspaceMapping(){
	delete[](dimSelection);
}

SubspaceMapping::SubspaceMapping(unsigned int inDim, unsigned int startIndex, unsigned int oDim)
 : Mapping(inDim, oDim), start(startIndex), outDim(oDim), dimSelection(NULL) {}

SubspaceMapping::SubspaceMapping(unsigned int inDim, unsigned int oDim, bool *dimSelect)
 : Mapping(inDim, oDim), start(0), outDim(oDim), dimSelection(new bool[inDim]) {
	unsigned int outDimCount = 0;
	for(unsigned int i=0; i<inDim; i++){
		this->dimSelection[i] = dimSelect[i];
		if(this->dimSelection[i]){
			outDimCount++;
		}
	}
	if(outDimCount != oDim){
		delete[](dimSelection);
		throw std::logic_error("SubspaceMapping: output dimension does not fit number of selected dimensions");
	}
}




MappingPtr LinearMapping::create(RealMatrixPtr a, RealVectorPtr b){
	if(a==NULL || b==NULL){
		throw std::invalid_argument(
			"Matrix A and vector b must not be NULL");
	}
	if(a->getNumRows() != b->getDimension()){
		throw std::invalid_argument(
			"Vector b must have same dimension as Matrix A has rows");
	}
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<LinearMapping>(
			new LinearMapping(a, b)
		)
	);
}
RealVectorPtr LinearMapping::evaluate(RealVectorPtr value){
	if(value->getDimension() != getInputDimension()){
		std::stringstream mess;
		mess << "LinearMapping::evaluate(): ";
		mess << "Wrong input dimension: ";
		mess << "expected " << getInputDimension();
		mess << ", but is " << value->getDimension() << ".";
		throw std::invalid_argument(mess.str());
	}

	return a->mult(value)->plus(b);
}
MappingPtr LinearMapping::invert(){
	RealMatrixPtr newMat = a->pseudoInverse();
	RealVectorPtr newBias = newMat->mult(b->scale(-1.0));
	return LinearMapping::create(newMat, newBias);
}
MappingPtr LinearMapping::clone(){
	return LinearMapping::create(a,b);
}
RealMatrixPtr LinearMapping::getJacobian(RealVectorPtr){
	return this->a;
}
LinearMapping::LinearMapping(RealMatrixPtr aMat, RealVectorPtr bVec)
	: Mapping(aMat->getNumCols(), bVec->getDimension()),
	a(aMat), b(bVec) {}



// binomialkoeffizient(n, k)
// 1  wenn k = 0 dann rückgabe 1
// 2  wenn 2k > n
// 3      dann führe aus ergebnis \leftarrow binomialkoeffizient(n, n-k)
// 4  sonst führe aus ergebnis \leftarrow n
// 5          von i \leftarrow 2 bis k
// 6              führe aus ergebnis \leftarrow ergebnis \cdot (n + 1 - i)
// 7                        ergebnis \leftarrow ergebnis : i
// 8  rückgabe ergebnis
unsigned int binomial(unsigned int N, unsigned int k){
	if(k==0){
		return 1;
	}
	if(N < 2*k){
		return binomial(N, N-k);
	}
	unsigned int b = N;
	for(unsigned int i = 2; i <= k; i++){
		b *= (N + 1 - i);
		b /= i;
	}
	return b;
}

MappingPtr PolynomExpansionMapping::create(unsigned int inputDim, unsigned int polynomOrder)
{
	// compute output dimension
	unsigned int outDim = 1;
	const unsigned int outDimMax = 10000;
	for(unsigned int p=1; p<=polynomOrder; p++){
		outDim += binomial(inputDim+p-1, p);
		if(outDimMax < outDim){
			throw std::invalid_argument("PolynomExpansionMapping does not allow more than 10000 output dimensions");
		}
	}

	// pre-compute binomial coefficients
	unsigned int **binomials = new unsigned int*[inputDim+polynomOrder];
	for(unsigned int i=0; i<inputDim+polynomOrder; i++){
		binomials[i] = new unsigned int[polynomOrder+1];
		const unsigned int maxArg = std::min(polynomOrder, i)+1;
		for(unsigned int k=0; k<maxArg; k++){
			binomials[i][k] = binomial(i, k);
		}
	}

	// create mapping
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<PolynomExpansionMapping>(
			new PolynomExpansionMapping(inputDim, polynomOrder, outDim, binomials)
		)
	);
}

RealVectorPtr PolynomExpansionMapping::evaluate(RealVectorPtr value){
	if(value->getDimension() != getInputDimension()){
		std::stringstream mess;
		mess << "PolynomExpansionMapping::evaluate(): ";
		mess << "Wrong input dimension: ";
		mess << "expected " << getInputDimension();
		mess << ", but is " << value->getDimension() << ".";
		throw std::invalid_argument(mess.str());
	}

	const double *input = value->getValues();
	const unsigned int inDim = getInputDimension();
	double *expanded = new double[getOutputDimension()];

	expanded[0] = 1;
	unsigned int expandedIndex = 1; // index to write next entry in expanded
	unsigned int lastOrderStart = 0;
	unsigned int lastOrderLength = 1;
	for(unsigned int p = 1; p <= order; p++){
		for(unsigned int inputIndex = 0; inputIndex < inDim; inputIndex++){
			//const unsigned int num = binomials[inDim-inputIndex+p-2][p-1];
			//const unsigned int from = lastOrderStart + lastOrderLength - num;
			//
			//for(unsigned int lastIndex = from; lastIndex < from+num; lastIndex++){
			//	expanded[expandedIndex++] = input[inputIndex] * expanded[lastIndex];
			//}

			const unsigned int num = binomials[inDim-inputIndex+p-2][p-1];
			const unsigned int offset = num + expandedIndex - lastOrderStart - lastOrderLength;
			for(unsigned int i = expandedIndex; i < expandedIndex+num; i++){
				expanded[i] = input[inputIndex] * expanded[i-offset];
			}
			expandedIndex += num;
		}
		lastOrderStart += lastOrderLength;
		lastOrderLength = binomials[inDim+p-1][p];
	}
	
	RealVectorPtr vec = RealVector::create(getOutputDimension(), expanded);
	delete[](expanded);

	if(expandedIndex != getOutputDimension()){
		std::stringstream m;
		m << "Something went wrong during polynom expansion: inputDim="<<getInputDimension()
			<<", polynomOrder="<<order<<"; expected output dimension "<<getOutputDimension()
			<<", but is "<<expandedIndex<<". Input is "<<value<<"; constructed output "<<vec;
		throw std::logic_error(m.str());
	}

	return vec;
}

PolynomExpansionMapping::~PolynomExpansionMapping(){
	for(unsigned int i=0; i<getInputDimension()+order; i++){
		delete[](this->binomials[i]);
	}
	delete[](this->binomials);
}

MappingPtr PolynomExpansionMapping::invert(){
	if(order == 0){
		throw std::logic_error("Cannot invert polynomial of order 0");
	}
	else{
		// index 0 holds the constant term, indices 1-inputDim hold the original value, so return these
		return SubspaceMapping::create(getOutputDimension(), 1, getInputDimension());
	}
}
MappingPtr PolynomExpansionMapping::clone(){
	return PolynomExpansionMapping::create(getInputDimension(), order);
}
RealMatrixPtr PolynomExpansionMapping::getJacobian(RealVectorPtr value){

	const unsigned int inDim = getInputDimension();
	const unsigned int outDim = getOutputDimension();

	// at first we need to find out which input dimension
	// contribute to which output dimension with which
	// polynomial order:
	// contribO[ m*outDim + n ] = p  ^= order in which the n'th input dim contributes to the m'th output dim
	unsigned int *contributionOrders = new unsigned int[inDim*outDim];
	for(unsigned int i=0; i<inDim*outDim; i++){
		contributionOrders[i] = 0;
	}

	//expanded[0] = 1;
	unsigned int outIndex = 1; // index to write next entry in expanded
	unsigned int lastOrderStart = 0;
	unsigned int lastOrderLength = 1;
	for(unsigned int p = 1; p <= order; p++){
		for(unsigned int inputIndex = 0; inputIndex < inDim; inputIndex++){
			//const unsigned int num = binomials[inDim-inputIndex+p-2][p-1];
			//const unsigned int offset = num + expandedIndex - lastOrderStart - lastOrderLength;
			//for(unsigned int i = expandedIndex; i < expandedIndex+num; i++){
			//	expanded[i] = input[inputIndex] * expanded[i-offset];
			//}
			//expandedIndex += num;
	
			const unsigned int num = binomials[inDim-inputIndex+p-2][p-1];
			const unsigned int offset = num + outIndex - lastOrderStart - lastOrderLength;
			for(unsigned int o = outIndex; o < outIndex+num; o++){
				for(unsigned int i=0; i<inDim; i++){
					contributionOrders[ o*inDim + i ] = contributionOrders[ (o-offset)*inDim + i ];
				}
				contributionOrders[ o*inDim + inputIndex ] = 1 + contributionOrders[ (o-offset)*inDim + inputIndex ];
			}
			outIndex += num;
			
		}
		lastOrderStart += lastOrderLength;
		lastOrderLength = binomials[inDim+p-1][p];
	}

//double *tmp = new double[inDim*outDim];
//for(unsigned int i=0; i<inDim*outDim; i++){
//	tmp[i] = (double)contributionOrders[i];
//}
//cout << "contributionOrders:" << endl << RealMatrix::create(outDim, inDim, tmp) << endl;

	// now we can derive the components of the Jacobian by standard polynomial derivation
	// e.g.  d(x1 * x2 * x2)/d(x2) = 2 * x1 * x2
	const double *input = value->getValues();
	double *values = new double[inDim*outDim];
	for(unsigned int i=0; i<inDim*outDim; i++){
		values[i] = 0;
	}
	double *inputPowers = new double[inDim*(order+1)];
	for(unsigned int inputIndex=0; inputIndex<inDim; inputIndex++){
		inputPowers[inputIndex*(order+1) + 0] = 1.0;
		for(unsigned int o=1; o<=order; o++){
			inputPowers[inputIndex*(order+1) + o] = input[inputIndex] * inputPowers[inputIndex*(order+1) + o-1];
		}
	}
//cout << "inputPowers:" << endl << RealMatrix::create(inDim, order+1, inputPowers) << endl;

	for(unsigned int outputIndex = 0; outputIndex < outDim; outputIndex++){
		for(unsigned int inputIndex = 0; inputIndex < inDim; inputIndex++){
			if(0 == contributionOrders[ outputIndex*inDim + inputIndex ]){
				continue;
			}
			double derivative = 1;

			for(unsigned int i=0; i<inputIndex; i++){
				derivative *= inputPowers[i*(order+1) + contributionOrders[ outputIndex*inDim + i ]];
			}

			unsigned int thisPow = contributionOrders[ outputIndex*inDim + inputIndex ];
			derivative *= thisPow * inputPowers[inputIndex*(order+1) + (thisPow-1)];

			for(unsigned int i=inputIndex+1; i<inDim; i++){
				derivative *= inputPowers[i*(order+1) + contributionOrders[ outputIndex*inDim + i ]];
			}
			values[outputIndex*inDim + inputIndex] = derivative;
		}
	}
	
	RealMatrixPtr jacobian = RealMatrix::create(outDim, inDim, values);
//cout << "jacobian:" << endl << RealMatrix::create(outDim, inDim, values) << endl;
	delete[](contributionOrders);
	delete[](values);
	delete[](inputPowers);

	return jacobian;
}

PolynomExpansionMapping::PolynomExpansionMapping(unsigned int inDim, unsigned int polynomOrder, unsigned int outDim, unsigned int **binoms)
 : Mapping(inDim, outDim), order(polynomOrder), binomials(binoms) {}
 
 
 
 
 MappingPtr AddMapping::create(MappingPtr first, MappingPtr second){
	if(first == NULL){
		throw std::invalid_argument(
			"AddMapping::create(MappingPtr,MappingPtr): first mapping must not be NULL");
	}
	if(second != NULL){
		if(first->getInputDimension() != second->getInputDimension()){
			throw std::invalid_argument(
				"AddMapping::create(MappingPtr,MappingPtr): input dimensions of mappings do not match ");
		}
		if(first->getOutputDimension() != second->getOutputDimension()){
			throw std::invalid_argument(
				"AddMapping::create(MappingPtr,MappingPtr): output dimensions of mappings do not match ");
		}
	}
	return boost::dynamic_pointer_cast<Mapping>(
		boost::shared_ptr<AddMapping>(
			new AddMapping(first, second)
		)
	);
}
RealVectorPtr AddMapping::evaluate(RealVectorPtr value){
	if(value->getDimension() != getInputDimension()){
		std::stringstream mess;
		mess << "AddMapping::evaluate(): ";
		mess << "Wrong input dimension: ";
		mess << "expected " << getInputDimension();
		mess << ", but is" << value->getDimension() << ".";
		throw std::invalid_argument(mess.str());
	}

	if(secondMapping){
		return firstMapping->evaluate(value)->plus(secondMapping->evaluate(value));
	}
	else{
		RealVectorPtr addValue = firstMapping->evaluate(value);
		return value->plus(addValue);
	}
}

MappingPtr AddMapping::clone(){
	return AddMapping::create(firstMapping,secondMapping);
}
RealMatrixPtr AddMapping::getJacobian(RealVectorPtr value){
	if(secondMapping){
		return firstMapping->getJacobian(value)->plus(secondMapping->getJacobian(value));
	}
	else{
		return RealMatrix::identity(getInputDimension())->plus(firstMapping->getJacobian(value));
	}
}
AddMapping::AddMapping(MappingPtr first, MappingPtr second)
	: Mapping(first->getInputDimension(), first->getOutputDimension()),
	firstMapping(first), secondMapping(second) {}



MappingPtr ConstantMapping::create(RealVectorPtr value){
	return MappingPtr(new ConstantMapping(value));
}
RealVectorPtr ConstantMapping::evaluate(RealVectorPtr){
	return c;
}
MappingPtr ConstantMapping::clone(){
	return create(c);
}
RealMatrixPtr ConstantMapping::getJacobian(RealVectorPtr){
	return RealMatrix::create(getOutputDimension(), getInputDimension(), 0.0);
}
ConstantMapping::ConstantMapping(RealVectorPtr constant)
 : Mapping(constant->getDimension(), constant->getDimension()), c(constant) {}


MappingPtr Clamping::create(RealVectorPtr minParam, RealVectorPtr maxParam){
	return MappingPtr(new Clamping(minParam, maxParam));
}
RealVectorPtr Clamping::evaluate(RealVectorPtr input){
	//return min->componentMax(input)->componentMin(max);
	const double *inData = input->getValues();
	const double *minData = min->getValues();
	const double *maxData = max->getValues();
	const unsigned int dim = getInputDimension();
	for(unsigned int i=0; i<dim; i++){
		if(inData[i] < minData[i]){
			buffer[i] = minData[i];
		}
		else if(inData[i] < maxData[i]){
			buffer[i] = inData[i];
		}
		else{
			buffer[i] = maxData[i];
		}
	}
	return RealVector::create(dim, buffer);
}
MappingPtr Clamping::clone(){
	return MappingPtr(new Clamping(min, max));
}
RealMatrixPtr Clamping::getJacobian(RealVectorPtr value){
	const double *inData = value->getValues();
	const double *minData = min->getValues();
	const double *maxData = max->getValues();
	const unsigned int dim = getInputDimension();
	for(unsigned int i=0; i<dim; i++){
		if(inData[i] < minData[i]){
			buffer[i] = 0.0;
		}
		else if(inData[i] < maxData[i]){
			buffer[i] = 1.0;
		}
		else{
			buffer[i] = 0.0;
		}
	}
	return RealMatrix::diagonal(RealVector::create(dim, buffer));
}
Clamping::~Clamping(){
	delete[](buffer);
}
Clamping::Clamping(RealVectorPtr minParam, RealVectorPtr maxParam)
 : Mapping(minParam->getDimension(), minParam->getDimension()), min(minParam), max(maxParam), 
   buffer(new double[minParam->getDimension()]) {}




