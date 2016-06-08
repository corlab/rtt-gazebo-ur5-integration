#include "RealVector.h"

#include <stdexcept>
#include <math.h>
#include <iostream>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>

//#include <boost/detail/sp_counted_base.hpp>


VectorPool** createPoolArray(unsigned int num);


// ========================================================================================================================
//  Object Pool implementation
// ========================================================================================================================

class VectorPool {
protected:
	RealVector** objectData;
	RealVector** objectFree;
	int size;
	int topIndex;

protected:
	void freeAll(){
		int i = size-1;
	
		for(topIndex = 0; topIndex < size; topIndex++){
			objectFree[topIndex] = objectData[i--];
		}
		return;
	}
public:
	void freeInstance(RealVector* instance){
// 		if((instance) && (topIndex<size) && (instance>=objectData[0]) && (instance<=objectData[size-1])){
			objectFree[topIndex++] = instance;
// 		}
		return;
	}
	
	RealVector* newInstance(void){
		if(topIndex>0){
			return(objectFree[--topIndex]);
		}
		return(0);
	}
	
	VectorPool(unsigned int vectorDimension, int s)
	 : objectData(NULL), objectFree(NULL), size(s), topIndex(s-1)
	{
		objectData = new RealVector*[s];
		if(!objectData){
			throw std::bad_alloc();
		}
		objectFree = new RealVector*[s];
		if(!objectFree){
			delete[](objectData);
			throw std::bad_alloc();
		}
	
		// create vectors
		for(int i=0; i<s; i++){
			this->objectData[i] = new RealVector(vectorDimension, 0.0);
		}
	
		freeAll();
	}

	//VectorPool(){}
	
	virtual ~VectorPool(void){
		delete[] objectData;
		delete[] objectFree;
	}
private:
	// forbid copying
	VectorPool(VectorPool&)
	 : objectData(NULL), objectFree(NULL), size(0), topIndex(0) 
	{throw std::logic_error("No copying allowed");}

	VectorPool operator=(const VectorPool&){throw std::logic_error("No copying allowed");}
};


VectorPool** createPoolArray(unsigned int num){
	VectorPool** arr = new VectorPool*[num];
	for(unsigned int i=0; i<num; i++){
		arr[i] = NULL;
	}
	return arr;
}

static unsigned int POOL_MAX_DIMENSION = 128;
VectorPool** RealVector::pools = createPoolArray(POOL_MAX_DIMENSION+1);


RealVector* RealVector::createFromPool(unsigned int dim){
// 	if( !isPooled(dim) ){
// 		throw std::logic_error("RealVector::createFromPool(unsigned int): there is no VectorPool for that dimension!");
// 	}
	return RealVector::pools[dim]->newInstance();
}

void RealVector::returnToPool(RealVector *vec){
	unsigned int dim = vec->getDimension();
// 	if( !isPooled(dim) ){
// 		throw std::logic_error("RealVector::returnToPool(RealVector*): there is no VectorPool for that dimension!");
// 	}
	return RealVector::pools[dim]->freeInstance(vec);
}

bool RealVector::isPooled(unsigned int dim){
	return (dim <= POOL_MAX_DIMENSION) && (RealVector::pools[dim] != NULL);
}

void RealVector::useVectorPool(unsigned int vectorDimension, unsigned int poolSize){
	if(RealVector::isPooled(vectorDimension)){
		throw std::invalid_argument("there is already a vector pool with that size");
	}
	if(POOL_MAX_DIMENSION < vectorDimension){
		std::cerr << "RealVector::useVectorPool(unsigned int,unsigned int): "
			<< "Pooling is currently only available for vector dimensions <=" 
			<< POOL_MAX_DIMENSION << ". Requested dimension is " << vectorDimension
			 << "." << std::endl;
		// no error or exception, since the functionality is not affected, only the performance
		return;
	}
	RealVector::pools[vectorDimension] = new VectorPool(vectorDimension, poolSize);
}


// ========================================================================================================================
//  Static Factories
// ========================================================================================================================

void intrusive_ptr_add_ref(RealVector * p){
	boost::detail::atomic_increment( &(p->referenceCount) );
}
void intrusive_ptr_release(RealVector * p){
	int prevCount = boost::detail::atomic_exchange_and_add( &(p->referenceCount), -1 );
	if(prevCount == 1){
		// this was the last reference
		if(p->pool == NULL){
			delete(p);
		}
		else{
			p->pool->freeInstance(p);
		}
	}
}

RealVectorPtr RealVector::createRaw(const unsigned int dimension){
	if(isPooled(dimension)){
		VectorPool *pool = pools[dimension];
		RealVector* vec = pool->newInstance();
		if(vec != NULL){
			// create and return intrusive_ptr
			vec->pool = pool;
// 			return boost::intrusive_ptr<RealVector>(vec);
// 			return boost::shared_ptr<RealVector>(vec);
			return RealVectorPtr(vec);
		}
	}
	// else
// 	return boost::intrusive_ptr<RealVector>(new RealVector(dimension));
// 	return boost::shared_ptr<RealVector>(new RealVector(dimension));
	return RealVectorPtr(new RealVector(dimension));
}

RealVectorPtr RealVector::create(const unsigned int dimension, const double initValue){
	RealVectorPtr vector = createRaw(dimension);
	for(unsigned int d=0; d<dimension; d++){
		vector->data[d] = initValue;
	}
	return vector;
}

RealVectorPtr RealVector::create(const unsigned int dimension, const double *values){
	RealVectorPtr vector = createRaw(dimension);
	//for(unsigned int d=0; d<dimension; d++){
	//	vector->data[d] = values[d];
	//}
	std::uninitialized_copy(values, values+dimension, vector->data);
	return vector;
}

RealVectorPtr RealVector::create(const unsigned int dimension, const float *values){
	RealVectorPtr vector = createRaw(dimension);
	for(unsigned int d=0; d<dimension; d++){
		vector->data[d] = (float)values[d];
	}
	return vector;
}

RealVectorPtr RealVector::value(const double x, const double y, const double z){
	RealVectorPtr vector = createRaw(3);
	vector->data[0] = x;
	vector->data[1] = y;
	vector->data[2] = z;
	return vector;
}

RealVectorPtr RealVector::value(const double x, const double y){
	RealVectorPtr vector = createRaw(2);
	vector->data[0] = x;
	vector->data[1] = y;
	return vector;
}

RealVectorPtr RealVector::value(const double x){
	RealVectorPtr vector = createRaw(1);
	vector->data[0] = x;
	return vector;
}

RealVectorPtr RealVector::random(const unsigned int dimension, const double range){
	return RealVector::random(dimension, -range, range);
}

RealVectorPtr RealVector::random(const unsigned int dimension, const double minVal, const double maxVal){
	RealVectorPtr vector = createRaw(dimension);
	for(unsigned int d=0; d<dimension; d++){
		vector->data[d] = minVal + (maxVal-minVal)*rand()/RAND_MAX;
	}
	return vector;
}



RealVectorPtr RealVector::load(std::string filename){
	std::ifstream is(filename.c_str());

	unsigned int dim;
	is >> dim;
	is >> std::ws;

	RealVectorPtr vec = RealVector::createRaw(dim);

	for(unsigned int i=0; i<dim; i++){
		double val;
		is >> val >> std::ws;
		vec->data[i] = val;
	}

	return vec;
}

void RealVector::store(std::string filename){
	std::ofstream os(filename.c_str());

	os << getDimension() << std::endl;
	for(unsigned int i=0; i<getDimension()-1; i++){
			os << this->data[i] << "\t";
	}
	os << this->data[getDimension()-1] << std::endl;
	os.close();
}


// ========================================================================================================================
//  Construction / Destruction
// ========================================================================================================================

RealVector::RealVector(const unsigned int dimension, const double initValue)
 : dim(dimension), data(new double[dimension]), /*floatData(NULL),*/ referenceCount(0), pool(NULL)
{
	for(unsigned int i=0; i<dimension; i++){
		data[i] = initValue;
	}
}
RealVector::RealVector(const unsigned int dimension)
 : dim(dimension), data(new double[dimension]), /*floatData(NULL),*/ referenceCount(0), pool(NULL)
{
}

RealVector::~RealVector(){
	delete[](data);
// 	if(floatData != NULL){
// 		delete[](floatData);
// 	}
}


// ========================================================================================================================
//  Accessors
// ========================================================================================================================

// inline const double* RealVector::getValues() const{
// 	return data;
// }

void RealVector::getValues(double *buffer) const{
	for(unsigned int i=0; i<getDimension(); i++){
		buffer[i] = data[i];
	}
}

// const float* RealVector::getValuesFloat() const{
// 	if(floatData == NULL){
// 		floatData = new float[getDimension()];
// 	}
// 	for(unsigned int i=0; i<getDimension(); i++){
// 		floatData[i] = (float)data[i];
// 	}
// 	return floatData;
// }

void RealVector::getValuesFloat(float *buffer) const{
	for(unsigned int i=0; i<getDimension(); i++){
		buffer[i] = (float)data[i];
	}
}

double RealVector::getValue(const unsigned int dimension) const{
	if(this->getDimension() <= dimension){
		throw std::invalid_argument("dimension index out of range");
	}

	return data[dimension];
}


// unsigned int RealVector::getDimension() const{
// 	return dim;
// }



// ========================================================================================================================
//  Manipulation
// ========================================================================================================================

RealVectorPtr RealVector::subspace(const unsigned int offset, const unsigned int dimension) const {
	if(this->getDimension() < offset+dimension){
		throw std::invalid_argument("illegal dimension");
	}

	RealVectorPtr newVec = RealVector::createRaw(dimension);
	for(unsigned int i=0; i<dimension; i++){
		newVec->data[i] = this->data[offset+i];
	}

	return newVec;
}

RealVectorPtr RealVector::attach(RealVectorPtr vec) const {
	RealVectorPtr joinedVector = RealVector::createRaw(this->getDimension()+vec->getDimension());

	//for(unsigned int i=0; i<this->getDimension(); i++){
	//	joinedVector->data[i] = this->data[i];
	//}
	//for(unsigned int i=0; i<vec->getDimension(); i++){
	//	joinedVector->data[this->getDimension() + i] = vec->data[i];
	//}
	std::uninitialized_copy(this->data, this->data+dim, joinedVector->data);
	std::uninitialized_copy(vec->data, vec->data+vec->getDimension(), joinedVector->data+dim);

	return joinedVector;
}

RealVectorPtr RealVector::plus(RealVectorPtr vector) const{
	if(this->getDimension() != vector->getDimension()){
		std::stringstream msg;
		msg << "RealVector::plus(RealVectorPtr): dimension mismatch: "
		    << "dimension of this vector is " << dim << ", but of ingoing vector is " << vector->dim << ".";
		throw std::invalid_argument(msg.str());
	}

	RealVectorPtr sum = RealVector::createRaw(this->getDimension());
	for(unsigned int i=0; i<getDimension(); i++){
		sum->data[i] = this->data[i] + vector->data[i];
	}

	return sum;
}

RealVectorPtr RealVector::minus(RealVectorPtr vector) const{
	if(this->getDimension() != vector->getDimension()){
		std::stringstream msg;
		msg << "RealVector::minus(RealVectorPtr): dimension mismatch: "
		    << "dimension of this vector is " << dim << ", but of ingoing vector is " << vector->dim << ".";
		throw std::invalid_argument(msg.str());
	}

	RealVectorPtr diff = RealVector::createRaw(this->getDimension());
	for(unsigned int i=0; i<getDimension(); i++){
		diff->data[i] = this->data[i] - vector->data[i];
	}

	return diff;
}

RealVectorPtr RealVector::scale(const double factor) const{
	RealVectorPtr scaledVec = RealVector::createRaw(this->getDimension());
	for(unsigned int i=0; i<getDimension(); i++){
		scaledVec->data[i] = this->data[i] * factor;
	}
	return scaledVec;
}

RealVectorPtr RealVector::componentMult(RealVectorPtr vec) const{
	if(this->getDimension() != vec->getDimension()){
		std::stringstream msg;
		msg << "RealVector::componentMult(RealVectorPtr): dimension mismatch: "
		    << "dimension of this vector is " << dim << ", but of ingoing vector is " << vec->dim << ".";
		throw std::invalid_argument(msg.str());
	}
	RealVectorPtr result = RealVector::createRaw(this->getDimension());
	for(unsigned int i=0; i<getDimension(); i++){
		result->data[i] = this->data[i] * vec->data[i];
	}
	return result;
}

RealVectorPtr RealVector::componentDiv(RealVectorPtr vec) const{
	if(this->getDimension() != vec->getDimension()){
		std::stringstream msg;
		msg << "RealVector::componentDiv(RealVectorPtr): dimension mismatch: "
		    << "dimension of this vector is " << dim << ", but of ingoing vector is " << vec->dim << ".";
		throw std::invalid_argument(msg.str());
	}
	RealVectorPtr result = RealVector::createRaw(this->getDimension());
	for(unsigned int i=0; i<getDimension(); i++){
		result->data[i] = this->data[i] / vec->data[i];
	}
	return result;
}

double RealVector::euclidean() const{
	double sumOfSquares = 0.0;
	for(unsigned int i=0; i<getDimension(); i++){
		sumOfSquares += data[i]*data[i];
	}
	return sqrt(sumOfSquares);
}

RealVectorPtr RealVector::setValue(const unsigned int dimension, const double val) const{
	if(this->getDimension() <= dimension){
		throw std::invalid_argument("dimension index out of range");
	}

	RealVectorPtr newVec = RealVector::createRaw(this->getDimension());
	for(unsigned int i=0; i<getDimension(); i++){
		newVec->data[i] = this->data[i];
	}
	newVec->data[dimension] = val;
	return newVec;
}

void RealVector::setValueEquals(const unsigned int dimension, const double val){
    if(this->getDimension() <= dimension){
        throw std::invalid_argument("dimension index out of range");
    }

    data[dimension] = val;
}

double RealVector::maxValue() const{
	double maxVal = data[0];
	for(unsigned int i = 1; i < dim; i++){
		if(maxVal < data[i]){
			maxVal = data[i];
		}
	}
	return maxVal;
}

double RealVector::minValue() const{
	double minVal = data[0];
	for(unsigned int i = 1; i < dim; i++){
		if(data[i] < minVal){
			minVal = data[i];
		}
	}
	return minVal;
}

RealVectorPtr RealVector::componentMax(RealVectorPtr vector) const{
	if(this->getDimension() != vector->getDimension()){
		std::stringstream msg;
		msg << "RealVector::componentMax(RealVectorPtr): dimension mismatch: "
		    << "dimension of this vector is " << dim << ", but of ingoing vector is " << vector->dim << ".";
		throw std::invalid_argument(msg.str());
	}
	RealVectorPtr maxVec = RealVector::createRaw(dim);
	for(unsigned int i = 0; i < dim; i++){
		if(this->data[i] < vector->data[i]){
			maxVec->data[i] = vector->data[i];
		} else {
			maxVec->data[i] = this->data[i];
		}
	}
	return maxVec;
}

RealVectorPtr RealVector::componentMin(RealVectorPtr vector) const{
	if(this->getDimension() != vector->getDimension()){
		std::stringstream msg;
		msg << "RealVector::componentMin(RealVectorPtr): dimension mismatch: "
		    << "dimension of this vector is " << dim << ", but of ingoing vector is " << vector->dim << ".";
		throw std::invalid_argument(msg.str());
	}
	RealVectorPtr minVec = RealVector::createRaw(dim);
	for(unsigned int i = 0; i < dim; i++){
		if(this->data[i] < vector->data[i]){
			minVec->data[i] = this->data[i];
		} else {
			minVec->data[i] = vector->data[i];
		}
	}
	return minVec;
}

RealVectorPtr RealVector::abs() const{
	RealVectorPtr absVec = RealVector::createRaw(dim);
	for(unsigned int i = 0; i < dim; i++){
		absVec->data[i] = 0.0<this->data[i] ? this->data[i] : -this->data[i] ;
	}
	return absVec;
}


double RealVector::scalarProduct(RealVectorPtr vector) const{
	if(this->getDimension() != vector->getDimension()){
		std::stringstream msg;
		msg << "RealVector::scalarProduct(RealVectorPtr): dimension mismatch: "
		    << "dimension of this vector is " << dim << ", but of ingoing vector is " << vector->dim << ".";
		throw std::invalid_argument(msg.str());
	}
	double prod = 0.0;
	for(unsigned int i = 0; i < dim; i++){
		prod += this->data[i] * vector->data[i];
	}
	return prod;
}



