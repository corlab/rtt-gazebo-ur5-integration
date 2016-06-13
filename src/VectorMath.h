#ifndef _VECTOR_MATH_H_
#define _VECTOR_MATH_H_


#include "RealVector.h"
#include <vector>

#include <stdexcept>


class VectorMath {

public:

	static inline double getSquareDistance(RealVectorPtr first, RealVectorPtr second){
		if(first->dim != second->dim){
			throw std::invalid_argument("VectorMath::getSquareDistance(RealVectorPtr,RealVectorPtr): dimension mismatch");
		}
		double sum = 0.0;
		for(unsigned int i=0; i<first->dim; i++){
			double dist = first->data[i] - second->data[i];
			sum += dist*dist;
		}
		return sum;
	}

	static inline RealVectorPtr addScaled(RealVectorPtr &first, const double &factor, RealVectorPtr &second){
		if(first->dim != second->dim){
			throw std::invalid_argument("VectorMath::addScaled(RealVectorPtr,double,RealVectorPtr): dimension mismatch");
		}
		RealVectorPtr result = RealVector::create(first->dim, 0.0);
		for(unsigned int i=0; i<first->dim; i++){
			result->data[i] = first->data[i] + factor*second->data[i];
		}
		return result;
	}

	static inline RealVectorPtr addScaled(RealVectorPtr &first, const double &factor, RealVectorPtr &second, RealVectorPtr &result){
		if(first->dim != second->dim){
			throw std::invalid_argument("VectorMath::addScaled(RealVectorPtr,double,RealVectorPtr): dimension mismatch");
		}
		if(first->dim != result->dim){
			throw std::invalid_argument("VectorMath::addScaled(): dimension mismatch");
		}
		if(result->referenceCount != 1){
			throw std::logic_error("VectorMath::addScaled(): can only write into matrix with refcount 1");
		}
		for(unsigned int i=0; i<first->dim; i++){
			result->data[i] = first->data[i] + factor*second->data[i];
		}
		return result;
	}

	static inline RealVectorPtr subtractAndScale(RealVectorPtr &first, RealVectorPtr &second, const double &factor, RealVectorPtr &result ){
		if(first->dim != second->dim){
			throw std::invalid_argument("VectorMath::subtractAndScale(): dimension mismatch");
		}
		if(first->dim != result->dim){
			throw std::invalid_argument("VectorMath::subtractAndScale(): dimension mismatch");
		}
		if(result->referenceCount != 1){
			throw std::logic_error("VectorMath::subtractAndScale(): can only write into matrix with refcount 1");
		}
		//RealVectorPtr result = RealVector::create(first->dim, 0.0);
		for(unsigned int i=0; i<first->dim; i++){
			result->data[i] = factor*(first->data[i] - second->data[i]);
		}
		return result;
	}

	static inline RealVectorPtr addAndScale(RealVectorPtr &first, RealVectorPtr &second, const double &factor, RealVectorPtr &result ){
		if(first->dim != second->dim){
			throw std::invalid_argument("VectorMath::addAndScale(): dimension mismatch");
		}
		if(first->dim != result->dim){
			throw std::invalid_argument("VectorMath::addAndScale(): dimension mismatch");
		}
		if(result->referenceCount != 1){
			throw std::logic_error("VectorMath::addAndScale(): can only write into matrix with refcount 1");
		}
		//RealVectorPtr result = RealVector::create(first->dim, 0.0);
		for(unsigned int i=0; i<first->dim; i++){
			result->data[i] = factor*(first->data[i] + second->data[i]);
		}
		return result;
	}

	static inline RealVectorPtr matrixMultAdd(RealMatrixPtr matrix, RealVectorPtr vec, RealVectorPtr offset){
		RealVectorPtr result = matrix->mult(vec);
		if(result->dim != offset->dim){
			throw std::invalid_argument("VectorMath::matrixMultAdd(RealMatrixPtr,RealVectorPtr,RealVectorPtr): dimension mismatch");
		}
		for(unsigned int i=0; i<result->dim; i++){
			result->data[i] = result->data[i] + offset->data[i];
		}
		return result;
	}

	// result = vec1 * vec2^T
	static inline RealMatrixPtr vecMultVecT(RealVectorPtr vec1, RealVectorPtr vec2){
		const unsigned int dim1 = vec1->getDimension();
		const unsigned int dim2 = vec2->getDimension();
		const double *vec1data = vec1->getValues();
		const double *vec2data = vec2->getValues();
		RealMatrixPtr result = RealMatrix::create(dim1, dim2, 0.0);
		for(unsigned int i=0; i<dim1; i++){
			for(unsigned int k=0; k<dim2; k++){
				result->matrix(i, k) = vec1data[i] * vec2data[k];
			}
		}
		return result;
	}

	// result = vec1 * vec2^T * factor
	static inline RealMatrixPtr vecMultVecTScale(RealVectorPtr vec1, RealVectorPtr vec2, double factor){
		const unsigned int dim1 = vec1->getDimension();
		const unsigned int dim2 = vec2->getDimension();
		const double *vec1data = vec1->getValues();
		const double *vec2data = vec2->getValues();
		RealMatrixPtr result = RealMatrix::create(dim1, dim2, 0.0);
		for(unsigned int i=0; i<dim1; i++){
			for(unsigned int k=0; k<dim2; k++){
				result->matrix(i, k) = vec1data[i] * vec2data[k] * factor;
			}
		}
		return result;
	}

	// result = mat + vec1 * vec2^T * factor
	static inline RealMatrixPtr matPlusvecMultVecTScale(RealMatrixPtr &mat, RealVectorPtr &vec1, RealVectorPtr &vec2, double &factor, RealMatrixPtr &result){
		if(!result.unique()){
			throw std::logic_error("VectorMath::matPlusvecMultVecTScale(): can only write into matrix with refcount 1");
		}
		const unsigned int dim1 = vec1->getDimension();
		const unsigned int dim2 = vec2->getDimension();
		const double *vec1data = vec1->getValues();
		const double *vec2data = vec2->getValues();
		for(unsigned int i=0; i<dim1; i++){
			for(unsigned int k=0; k<dim2; k++){
				result->matrix(i, k) = mat->matrix(i, k) + vec1data[i] * vec2data[k] * factor;
			}
		}
		return result;
	}

	static inline RealMatrixPtr weightedMatrixSum(std::vector<RealMatrixPtr> matrices, std::vector<double> weights){
		if(matrices.size() == 0){
			throw std::invalid_argument("VectorMath::weightedMatrixSum(std::vector<RealMatrixPtr>,std::vector<double>): matrices empty");
		}
		if(matrices.size() != weights.size()){
			throw std::invalid_argument("VectorMath::weightedMatrixSum(std::vector<RealMatrixPtr>,std::vector<double>): unequal #elements in matrices and weights");
		}
		RealMatrixPtr sum = RealMatrix::create(matrices[0]->getNumRows(), matrices[0]->getNumCols(), 0.0);
		for(unsigned int i=0; i<matrices.size(); i++){
			sum->matrix += weights[i] * matrices[i]->matrix;
		}
		return sum;
	}


};




#endif

