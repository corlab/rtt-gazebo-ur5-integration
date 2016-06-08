#ifndef _REAL_VECTOR_H_
#define _REAL_VECTOR_H_


#include <ostream>
//#include <map>


#include <boost/shared_ptr.hpp>
#include <boost/intrusive_ptr.hpp>


class RealVector;

void intrusive_ptr_add_ref(RealVector * p);
void intrusive_ptr_release(RealVector * p);
typedef boost::intrusive_ptr<RealVector> RealVectorPtr;
// typedef boost::shared_ptr<RealVector> RealVectorPtr;


class VectorMath;
class VectorPool;


class RealVector {
public:
	static RealVectorPtr create(const unsigned int dimension, const double initValue=0.0);
	static RealVectorPtr create(const unsigned int dimension, const double *values);
	static RealVectorPtr create(const unsigned int dimension, const float *values);

	static RealVectorPtr value(const double x, const double y, const double z);
	static RealVectorPtr value(const double x, const double y);
	static RealVectorPtr value(const double x);

	static RealVectorPtr random(const unsigned int dimension, const double range);
	static RealVectorPtr random(const unsigned int dimension, const double minVal, const double maxVal);

	static RealVectorPtr load(std::string filename);
	void store(std::string filename);

	inline const double* getValues() const{ return data; }
	void getValues(double *buffer) const;
	void getValuesFloat(float *buffer) const;
	double getValue(const unsigned int dimension) const;

	inline unsigned int getDimension() const{ return dim; }

	RealVectorPtr subspace(const unsigned int offset, const unsigned int dimension) const;
	RealVectorPtr attach(RealVectorPtr vec) const;

	RealVectorPtr plus(RealVectorPtr vector) const;
	RealVectorPtr minus(RealVectorPtr vector) const;
	RealVectorPtr scale(const double factor) const;
	RealVectorPtr componentMult(RealVectorPtr vec) const;
	RealVectorPtr componentDiv(RealVectorPtr vec) const;

	double scalarProduct(RealVectorPtr vector) const;

	RealVectorPtr setValue(const unsigned int dimension, const double value) const;
    void setValueEquals(const unsigned int dimension, const double val);

	RealVectorPtr componentMax(RealVectorPtr vector) const;
	RealVectorPtr componentMin(RealVectorPtr vector) const;
	RealVectorPtr abs() const;

	double euclidean() const;
	double maxValue() const;
	double minValue() const;


	~RealVector();


	static void useVectorPool(unsigned int vectorDimension, unsigned int poolSize);

	friend class VectorMath;

private:
	static RealVectorPtr createRaw(const unsigned int dimension);

	RealVector(const unsigned int dimension, const double initValue);
	RealVector(const unsigned int dimension);
	RealVector(RealVector& vec);
	RealVector operator=(RealVector& vec);

	unsigned int dim;
	double* data;
// 	mutable float* floatData;

	friend class VectorPool;
	static RealVector* createFromPool(unsigned int dim);
	static void returnToPool(RealVector *vec);
	static bool isPooled(unsigned int dim);

// 	static std::map<unsigned int, VectorPool*> pools;
	// each vector dimension has an own pool
	// array maps vector-dimension to VectorPool* (for that dimension)
	// entry is NULL if the dimension is not pooled
	// array is much (!) faster when accessing and checking
	// therefore we accept the restriction to pool only up to a certain max. vector dimension
	static VectorPool **pools;

	friend void intrusive_ptr_add_ref(RealVector *);
	friend void intrusive_ptr_release(RealVector *);
	mutable int referenceCount;
	VectorPool *pool; // VectorPool which this vector belongs to or NULL is it does not belong to a pool

};

// std output operator
inline std::ostream& operator<<(std::ostream& out, const RealVectorPtr &vec){
	if(vec == NULL){
		return out << "NONE";
	}

	const double *data = vec->getValues();
	for(unsigned int i=0; i<vec->getDimension()-1; i++){
		out << data[i] << "\t";
	}
	return out << data[vec->getDimension()-1];
}



#endif
