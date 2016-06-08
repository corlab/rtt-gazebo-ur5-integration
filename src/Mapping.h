#ifndef _MAPPING_H_
#define _MAPPING_H_


#include <stdexcept>
#include <sstream>


#include <boost/shared_ptr.hpp>


#include "RealVector.h"
#include "RealMatrix.h"

class Mapping;
typedef boost::shared_ptr<Mapping> MappingPtr;


class Mapping {
public:
	virtual unsigned int getInputDimension() const{
		return inputDim;
	}
	virtual unsigned int getOutputDimension() const{
		return outputDim;
	}
	virtual RealVectorPtr evaluate(RealVectorPtr value) = 0;
	
	virtual ~Mapping(){}

	// optional
	virtual MappingPtr invert(){
		throw std::runtime_error("Function invert() not supported for this mapping");
	}

	// optional
	virtual MappingPtr clone(){
		throw std::runtime_error("Function clone() not supported for this mapping");
	}

	// optional
	// returns the (outputDim)x(inputDim) Jacobian derivative matrix
	virtual RealMatrixPtr getJacobian(RealVectorPtr){
		throw std::runtime_error("Function getJacobian(RealVectorPtr) not supported for this mapping");
	}

	//virtual RealVectorPtr getHiddenValues(RealVectorPtr input){
	//	return RealVector::create(0, (double*)NULL); // NULL??????
	//}

	// methods for concatenating Mappings
	MappingPtr concat(MappingPtr mapping);
	MappingPtr add(RealVectorPtr vec);
	MappingPtr add(double num);
	MappingPtr scale(double num);
	MappingPtr mult(RealMatrixPtr mat);
	MappingPtr componentWise(double (*function)(double), double (*inverse)(double) = NULL);
	
protected:
	Mapping(unsigned int inDim, unsigned int outDim) : inputDim(inDim), outputDim(outDim){}
	Mapping(Mapping& m) : inputDim(m.inputDim), outputDim(m.outputDim){}
private:
	unsigned int inputDim, outputDim;
};


class Identity : public Mapping {
public:
	static MappingPtr create(unsigned int dimension = 1);
	RealVectorPtr evaluate(RealVectorPtr value);
	MappingPtr invert();
	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);
private:
	Identity(int dim);
	// forbid copies
	Identity(Identity&m):Mapping(m){}
	Identity operator=(const Identity&){throw std::logic_error("Copying not allowed");}
};

class Zero : public Mapping {
public:
	static MappingPtr create(unsigned int dimension = 1);
	static MappingPtr create(unsigned int inDim, unsigned int outDim);
	RealVectorPtr evaluate(RealVectorPtr value);
	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);
private:
	Zero(unsigned int dim);
	Zero(unsigned int inDim, unsigned int outDim);
	// forbid copies
	Zero(Zero&m):Mapping(m){}
	Zero operator=(const Zero&){throw std::logic_error("Copying not allowed");}
};

/**
 * Modulo mapping
 */
class Modulo: public Mapping {
public:
    /**
     * Factory of modulo mapping. Give dimension of the mapping and the
     * modulo operand.
     * @param dim Dimension of the mapping (input dim is output dim)
     * @param mod Operand
     * @return Mapping shared pointer
     */
    static MappingPtr create(unsigned int dimension, double mod);
    RealVectorPtr evaluate(RealVectorPtr value);
    MappingPtr clone();
    RealMatrixPtr getJacobian(RealVectorPtr value);
private:
    /**
     * Constructor of modulo mapping. Give dimension of the mapping and the
     * modulo operand.
     * @param dim Dimension of the mapping (input dim is output dim)
     * @param mod Operand
     */
    Modulo(unsigned int dim, double mod);

    Modulo(Modulo&m) :
        Mapping(m), _mod() {
    }
    Modulo operator=(const Modulo&) {
        throw std::logic_error("Copying not allowed");
    }

    /**
     * Modulo operand.
     */
    double _mod;
};

class ComponentLinearMapping : public Mapping {
public:
	static MappingPtr scale(unsigned int dim, double alpha);

	static MappingPtr add(RealVectorPtr vec);

	static MappingPtr create(RealVectorPtr gain, RealVectorPtr bias);

	virtual RealVectorPtr evaluate(RealVectorPtr value);
	
	virtual ~ComponentLinearMapping(){}

	MappingPtr invert();
	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);
private:
	ComponentLinearMapping(RealVectorPtr g, RealVectorPtr b);
	// forbid copies
	ComponentLinearMapping(ComponentLinearMapping&m):Mapping(m),gain(m.gain),bias(m.bias){}
	ComponentLinearMapping operator=(const ComponentLinearMapping&){throw std::logic_error("Copying not allowed");}

	RealVectorPtr gain;
	RealVectorPtr bias;
};

class PointWiseMapping : public Mapping {
public:
	static MappingPtr create(unsigned int dimension, double (*function)(double), double (*inverseFunction)(double) = NULL);

	virtual RealVectorPtr evaluate(RealVectorPtr value);
	
	virtual ~PointWiseMapping(){}

	MappingPtr invert();
	MappingPtr clone();
private:
	PointWiseMapping(unsigned int dimension, double (*function)(double), double (*inverseFunction)(double) = NULL);
	// forbid copies
	PointWiseMapping(PointWiseMapping&m):Mapping(m),function(m.function),inverseFunction(m.inverseFunction){}
	PointWiseMapping operator=(const PointWiseMapping&){throw std::logic_error("Copying not allowed");}

	double (*function)(double);
	double (*inverseFunction)(double);
};

class SequentialMapping : public Mapping {
public:
	static MappingPtr create(MappingPtr first, MappingPtr second);

	virtual RealVectorPtr evaluate(RealVectorPtr value);
	
	virtual ~SequentialMapping(){}

	MappingPtr invert();
	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);
private:
	SequentialMapping(MappingPtr first, MappingPtr second);
	// forbid copies
	SequentialMapping(SequentialMapping&m):Mapping(m),firstMapping(m.firstMapping),secondMapping(m.secondMapping){}
	SequentialMapping operator=(const SequentialMapping&){throw std::logic_error("Copying not allowed");}

	MappingPtr firstMapping;
	MappingPtr secondMapping;
};	

class SubspaceMapping : public Mapping {
public:
	static MappingPtr create(unsigned int inputDim, unsigned int startIndex, unsigned int outputDim);
	static MappingPtr create(unsigned int inputDim, bool *dimSelection);

	virtual RealVectorPtr evaluate(RealVectorPtr value);

	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);
	
	virtual ~SubspaceMapping();

private:
	SubspaceMapping(unsigned int inputDim, unsigned int startIndex, unsigned int outputDim);
	SubspaceMapping(unsigned int inputDim, unsigned int outputDim, bool *dimSelection);

	// forbid copies
	SubspaceMapping(SubspaceMapping&m):Mapping(m),start(m.start),outDim(m.outDim),dimSelection(NULL){}
	SubspaceMapping operator=(const SubspaceMapping&){throw std::logic_error("Copying not allowed");}

	unsigned int start;
	unsigned int outDim;
	bool *dimSelection;
};

class LinearMapping : public Mapping {
public:
	static MappingPtr create(RealMatrixPtr a, RealVectorPtr b);

	virtual RealVectorPtr evaluate(RealVectorPtr value);
	
	virtual ~LinearMapping(){}

	MappingPtr invert();
	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);
private:
	LinearMapping(RealMatrixPtr a, RealVectorPtr b);
	// forbid copies
	LinearMapping(LinearMapping&m):Mapping(m),a(m.a),b(m.b){}
	LinearMapping operator=(const LinearMapping&){throw std::logic_error("Copying not allowed");}

	RealMatrixPtr a;
	RealVectorPtr b;
};

class PolynomExpansionMapping : public Mapping {
public:
	static MappingPtr create(unsigned int inputDim, unsigned int polynomOrder);

	virtual RealVectorPtr evaluate(RealVectorPtr value);
	
	virtual ~PolynomExpansionMapping();

	MappingPtr invert();
	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);
private:
	PolynomExpansionMapping(unsigned int inputDim, unsigned int polynomOrder, unsigned int outputDim, unsigned int **binomials);
	// forbid copies
	PolynomExpansionMapping(PolynomExpansionMapping&m):Mapping(m),order(m.order),binomials(NULL){}
	PolynomExpansionMapping operator=(const PolynomExpansionMapping&){throw std::logic_error("Copying not allowed");}

	unsigned int order;
	unsigned int **binomials;
};

// TODO add mapping result to input vector

class AddMapping : public Mapping {
public:
	static MappingPtr create(MappingPtr first, MappingPtr second=MappingPtr());

	virtual RealVectorPtr evaluate(RealVectorPtr value);
	
	virtual ~AddMapping(){}

// 	MappingPtr invert();
	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);
private:
	AddMapping(MappingPtr first, MappingPtr second);
	// forbid copies
	AddMapping(AddMapping&m):Mapping(m),firstMapping(m.firstMapping),secondMapping(m.secondMapping){}
	AddMapping operator=(const AddMapping&){throw std::logic_error("Copying not allowed");}

	MappingPtr firstMapping;
	MappingPtr secondMapping;
};

class ConstantMapping : public Mapping {
public:
	static MappingPtr create(RealVectorPtr constant);

	virtual RealVectorPtr evaluate(RealVectorPtr value);
	
	virtual ~ConstantMapping(){}

// 	MappingPtr invert();
	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);
private:
	ConstantMapping(RealVectorPtr constant);
	// forbid copies
	ConstantMapping(ConstantMapping&m):Mapping(m),c(m.c){throw std::logic_error("Copying not allowed");}
	ConstantMapping operator=(const ConstantMapping&){throw std::logic_error("Copying not allowed");}

	RealVectorPtr c;
};

class Clamping : public Mapping {
public:
	static MappingPtr create(RealVectorPtr min, RealVectorPtr max);
	RealVectorPtr evaluate(RealVectorPtr input);

	MappingPtr clone();
	RealMatrixPtr getJacobian(RealVectorPtr value);

	virtual ~Clamping();
private:
	Clamping(RealVectorPtr min, RealVectorPtr max);
	// forbid copies
	Clamping(Clamping&m):Mapping(m),min(m.min),max(m.max),buffer(NULL){throw std::logic_error("Copying not allowed");}
	Clamping operator=(const Clamping&){throw std::logic_error("Copying not allowed");}

	RealVectorPtr min;
	RealVectorPtr max;
	double *buffer;
};
	

#endif
