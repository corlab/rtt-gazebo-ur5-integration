#ifndef _REAL_MATRIX_H_
#define _REAL_MATRIX_H_

#include <ostream>


#include <boost/shared_ptr.hpp>
#include <Eigen/Core>


#include "RealVector.h"

class TimeSeries;
class VectorMath;
// typedef boost::shared_ptr<TimeSeries> TimeSeriesPtr;


class RealMatrix;
typedef boost::shared_ptr<RealMatrix> RealMatrixPtr;


typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenRowMatrix;


class RealMatrix {
public:
	static RealMatrixPtr create(unsigned int rows, unsigned int cols, double initValue=0.0);
	static RealMatrixPtr create(unsigned int rows, unsigned int cols, const double *values);
	static RealMatrixPtr create(unsigned int rows, unsigned int cols, const float *values);
	static RealMatrixPtr create(RealMatrixPtr mat);
	static RealMatrixPtr create(RealVectorPtr vec);
	// matrix with vectors in rows
	static RealMatrixPtr create(boost::shared_ptr<TimeSeries> series);

	static RealMatrixPtr identity(unsigned int dimension);

	static RealMatrixPtr diagonal(RealVectorPtr diagValues);
	
	static RealMatrixPtr random(unsigned int rows, unsigned int cols, const double range);
	static RealMatrixPtr random(unsigned int rows, unsigned int cols, const double minVal, const double maxVal);

	static RealMatrixPtr load(std::string filename);
	void store(std::string filename);

	const double* getValues() const;
	void getValues(double *buffer) const;

	double getValue(unsigned int row, unsigned int col) const;

	unsigned int getNumRows() const;
	unsigned int getNumCols() const;

	RealMatrixPtr submatrix(unsigned int offsetRows, unsigned int numRows, unsigned int offsetCols, unsigned int numCols) const;

	RealVectorPtr getRowVector(unsigned int rowIndex);
	RealVectorPtr getColVector(unsigned int colIndex);

	RealMatrixPtr attachCols(RealMatrixPtr mat) const;
	RealMatrixPtr attachRows(RealMatrixPtr mat) const;

	RealMatrixPtr plus(RealMatrixPtr mat) const;

	RealMatrixPtr minus(RealMatrixPtr mat) const;

	RealMatrixPtr scale(const double factor) const;

	RealMatrixPtr setValue(unsigned int i, unsigned int j, double value) const;

	double maxValue() const;

	double minValue() const;

	RealMatrixPtr componentMax(RealMatrixPtr mat) const;

	RealMatrixPtr componentMin(RealMatrixPtr mat) const;

	RealMatrixPtr abs() const;

	RealMatrixPtr transpose() const;

	RealMatrixPtr mult(RealMatrixPtr mat) const;

	RealVectorPtr mult(RealVectorPtr vec) const;

// 	RealMatrixPtr inverse() const;
	RealMatrixPtr pseudoInverse() const;

	RealMatrixPtr sqrt() const;

	RealMatrixPtr orthonormalize() const;

	~RealMatrix();

	friend class VectorMath;

private:
	RealMatrix(unsigned int rows, unsigned int cols, double initValue=0.0);
	RealMatrix(EigenRowMatrix matrix);
	RealMatrix(const RealMatrix& mat);
	RealMatrix operator=(RealMatrix& vec);

	unsigned int rows;
	unsigned int cols;
	
	// default storage is ColMajor, but RowMajor allows direct data access for 
	// left multiplication and creation-from-TimeSeries
	EigenRowMatrix matrix;
	//Eigen::MatrixXd matrix;
};

// std output operator
inline std::ostream& operator<<(std::ostream& out, const RealMatrixPtr &mat){
	if(mat == NULL){
		return out << "NONE";
	}

	for(unsigned int i=0; i<mat->getNumRows(); i++){
		for(unsigned int j=0; j<mat->getNumCols()-1; j++){
			out << mat->getValue(i,j) << "\t";
		}
		out << mat->getValue(i, mat->getNumCols()-1);
		if(i != mat->getNumRows()-1){
			out << std::endl;
		}
	}
	return out;
}



#endif
