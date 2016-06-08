#include "RealMatrix.h"


#include <stdexcept>
#include <math.h>
#include <iostream>
#include <fstream>


#include <Eigen/SVD>
#include <Eigen/Core>


#include "TimeSeries.h"


// import most common Eigen types 
// USING_PART_OF_NAMESPACE_EIGEN
//using namespace Eigen;
using namespace std;

bool isANumber(double value);


/*
 * NOTE: old versions of Eigen2 have a stupid bug, which forces us to use
 * 	 ONLY RowMajor matrices in all (temp.) results
 * 	 The bug seems to be fixed with Eigen2.0.5
 * 
 * 	 Example: (multiplication causes invalid read/write)

	#include <Eigen/Core>
	using namespace Eigen;
	
	int main(){
		typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrix;
		RowMatrix eigena = MatrixXd::Constant(17, 1000, 0.0);
		RowMatrix eigenb = MatrixXd::Constant(1000, 3, 0.0);
	
		// invalid memory operations in here:
		MatrixXd eigenProd = eigena * eigenb;
	
		return 0;
	}
 * 
 * 
 * 	 If we DO use only RowMajor, everything is fine
 * 
 */


//RealVectorPtr vecFromMat(EigenRowMatrix mat){
//	return RealVector::create(mat.rows()*mat.cols(), mat.data());
//}

bool isANumber(double value){
	if(value != value){
		return false; // NaN
	}
	if(value == numeric_limits<double>::infinity()){
		return false; 
	}
	if(value == -numeric_limits<double>::infinity()){
		return false; 
	}
	return true;
}



RealMatrixPtr RealMatrix::create(unsigned int rows, unsigned int cols, double initValue){
	return RealMatrixPtr(new RealMatrix(rows,cols,initValue));
}
RealMatrixPtr RealMatrix::create(unsigned int rows, unsigned int cols, const double *values){
	RealMatrixPtr mat = create(rows,cols,0.0);
	for(unsigned int row=0; row < rows; row++){
		for(unsigned int col=0; col < cols; col++){
			mat->matrix(row, col) = values[row*cols + col];
		}	
	}
	return mat;
}
RealMatrixPtr RealMatrix::create(unsigned int rows, unsigned int cols, const float *values){
	RealMatrixPtr mat = create(rows,cols,0.0);
	for(unsigned int row=0; row < rows; row++){
		for(unsigned int col=0; col < cols; col++){
			mat->matrix(row, col) = values[row*cols + col];
		}	
	}
	return mat;
}
RealMatrixPtr RealMatrix::create(RealMatrixPtr mat){
	if(!mat){
		throw std::invalid_argument("RealMatrix::create(RealMatrixPtr): null pointer");
	}
	return RealMatrixPtr(new RealMatrix(mat->matrix));
}
RealMatrixPtr RealMatrix::create(RealVectorPtr vec){
	if(!vec){
		throw std::invalid_argument("RealMatrix::create(RealVectorPtr): null pointer");
	}
	const unsigned int D = vec->getDimension();
	EigenRowMatrix mat(D, 1);
	const double *values = vec->getValues();
	for(unsigned int i=0; i<D; i++){
		mat(i,0) = values[i];
	}
	return RealMatrixPtr(new RealMatrix(mat));
}
// matrix with vectors in rows
RealMatrixPtr RealMatrix::create(TimeSeriesPtr series){
	if(!series){
		throw std::invalid_argument("RealMatrix::create(TimeSeriesPtr): null pointer");
	}
	const unsigned int cols = series->getValue(0)->getDimension();
	const unsigned int N = series->getLength();
// 	EigenRowMatrix mat((int)N, (int)cols);
// 	for(unsigned int i=0; i<N; i++){
// 		mat.block(i,0,1,cols) = create(series->getValue(i))->transpose()->matrix;
// 	}
// 	return RealMatrixPtr(new RealMatrix(mat));
	RealMatrix *mat = new RealMatrix(N, cols, 0.0);
	double* matData = mat->matrix.data();
	for(unsigned int t=0; t<N; t++){
		const double *values = series->getValue(t)->getValues();
		std::uninitialized_copy(values, values+cols, matData+t*cols);
		//for(unsigned int d=0; d<cols; d++){
		//	mat->matrix(t,d) = values[d];
		//}
	}
	return RealMatrixPtr(mat);
}
RealMatrixPtr RealMatrix::identity(unsigned int dimension){
	RealMatrixPtr mat = create(dimension, dimension, 0.0);
	for(unsigned int i=0; i<dimension; i++){
		mat->matrix(i, i) = 1.0;
	}
	return mat;
}

RealMatrixPtr RealMatrix::diagonal(RealVectorPtr diagValues){
	if(!diagValues){
		throw std::invalid_argument("RealMatrix::diagonal(RealVectorPtr): null pointer");
	}
	const unsigned int dim = diagValues->getDimension();
	// FIXME for efficiency reasons it should be possible to treat diagonal matrices interally
	RealMatrixPtr mat = create(dim, dim, 0.0);
	for(unsigned int i=0; i<dim; i++){
		mat->matrix(i, i) = diagValues->getValues()[i];
	}
	return mat;
}


RealMatrixPtr RealMatrix::random(unsigned int rows, unsigned int cols, const double range){
	return random(rows, cols, -range, range);
}
RealMatrixPtr RealMatrix::random(unsigned int rows, unsigned int cols, const double minVal, const double maxVal){
	RealMatrixPtr mat = create(rows,cols,0.0);
	for(unsigned int row=0; row < rows; row++){
		for(unsigned int col=0; col < cols; col++){
			mat->matrix(row, col) = minVal + (maxVal-minVal)*rand()/RAND_MAX;
		}	
	}
	return mat;
}

RealMatrixPtr RealMatrix::load(std::string filename){
	ifstream is(filename.c_str());

	unsigned int rows;
	is >> rows;
	is >> ws;
	unsigned int cols;
	is >> cols;
	is >> ws;

	RealMatrixPtr mat = RealMatrix::create(rows, cols);

	for(uint r=0; r<rows; r++){
		for(uint c=0; c<cols; c++){
			double val;
			is >> val >> ws;
			mat->matrix(r,c) = val;
		}
	}

	return mat;
}

void RealMatrix::store(std::string filename){
	ofstream os(filename.c_str());

	os << getNumRows() << endl;
	os << getNumCols() << endl;
	for(uint r=0; r<getNumRows(); r++){
		for(uint c=0; c<getNumCols()-1; c++){
			os << this->matrix(r,c) << "\t";
		}
		os << this->matrix(r,getNumCols()-1) << endl;
	}
	os.close();
}

RealMatrix::RealMatrix(unsigned int numRows, unsigned int numCols, double initValue)
 : rows(numRows), cols(numCols), matrix(EigenRowMatrix::Constant(numRows, numCols, initValue))
{
}
RealMatrix::RealMatrix(EigenRowMatrix mat)
 : rows(mat.rows()), cols(mat.cols()), matrix(mat)
{
}
RealMatrix::RealMatrix(const RealMatrix& mat)
 : rows(mat.rows), cols(mat.cols), matrix(mat.matrix)
{
}

RealMatrix::~RealMatrix(){}

const double* RealMatrix::getValues() const{
	return matrix.data();
}
void RealMatrix::getValues(double *buffer) const{
	for(unsigned int row=0; row < getNumRows(); row++){
		for(unsigned int col=0; col < getNumCols(); col++){
			buffer[row*getNumCols() + col] = matrix(row, col);
		}
	}
}


double RealMatrix::getValue(unsigned int row, unsigned int col) const{
	return matrix(row, col);
}

unsigned int RealMatrix::getNumRows() const{
	return rows;
}
unsigned int RealMatrix::getNumCols() const{
	return cols;
}

RealMatrixPtr RealMatrix::submatrix(unsigned int offsetRows, unsigned int numRows, unsigned int offsetCols, unsigned int numCols) const{
	if(this->getNumRows() < offsetRows+numRows){
		throw invalid_argument("Not enough matrix rows for desired sub-matrix.");
	}
	if(this->getNumCols() < offsetCols+numCols){
		throw invalid_argument("Not enough matrix cols for desired sub-matrix.");
	}
	
	RealMatrixPtr subMat = create(numRows, numCols, 0.0);
	for(unsigned int subMatRow = 0; subMatRow < numRows; subMatRow++){
		for(unsigned int subMatCol = 0; subMatCol < numCols; subMatCol++){
			subMat->matrix(subMatRow, subMatCol) = this->matrix(subMatRow+offsetRows, subMatCol+offsetCols);
		}	
	}
	return subMat;
}

RealVectorPtr RealMatrix::getRowVector(unsigned int rowIndex){
	if(rows <= rowIndex){
		throw std::invalid_argument("RealMatrix::getRowVector(unsigned int): index out of bounds");
	}
	double *rowData = new double[cols];
	for(unsigned int i=0; i<cols; i++){
		rowData[i] = this->matrix(rowIndex, i);
	}
	RealVectorPtr rowVector = RealVector::create(cols, rowData);
	delete[](rowData);
	return rowVector;
}

RealVectorPtr RealMatrix::getColVector(unsigned int colIndex){
	if(cols <= colIndex){
		throw std::invalid_argument("RealMatrix::getColVector(unsigned int): index out of bounds");
	}
	double *colData = new double[rows];
	for(unsigned int i=0; i<rows; i++){
		colData[i] = this->matrix(i, colIndex);
	}
	RealVectorPtr colVector = RealVector::create(rows, colData);
	delete[](colData);
	return colVector;
}

RealMatrixPtr RealMatrix::attachCols(RealMatrixPtr mat) const{
	if(this->getNumRows() != mat->getNumRows()){
		throw invalid_argument("Number of rows must match for col-attaching.");
	}
	RealMatrixPtr newMat = create(rows, this->cols+mat->cols);
	newMat->matrix.block(0,0,this->rows,this->cols) = this->matrix;
	newMat->matrix.block(0,this->cols,mat->rows,mat->cols) = mat->matrix;

	return newMat;
}
RealMatrixPtr RealMatrix::attachRows(RealMatrixPtr mat) const{
	if(this->getNumCols() != mat->getNumCols()){
		throw invalid_argument("Number of cols must match for row-attaching.");
	}
	RealMatrixPtr newMat = create(this->rows+mat->rows, this->cols);
	newMat->matrix.block(0,0,this->rows,this->cols) = this->matrix;
	newMat->matrix.block(this->rows,0,mat->rows,mat->cols) = mat->matrix;

	return newMat;
}

RealMatrixPtr RealMatrix::plus(RealMatrixPtr mat) const{
	if(this->cols != mat->cols || this->rows != mat->rows){
		throw invalid_argument("Dimension mismatch while adding matrices");
	}
	return RealMatrixPtr(new RealMatrix(this->matrix + mat->matrix));
}

RealMatrixPtr RealMatrix::minus(RealMatrixPtr mat) const{
	if(this->cols != mat->cols || this->rows != mat->rows){
		throw invalid_argument("Dimension mismatch while subtracting matrices");
	}
	return RealMatrixPtr(new RealMatrix(this->matrix - mat->matrix));
}

RealMatrixPtr RealMatrix::scale(const double factor) const{
	return RealMatrixPtr(new RealMatrix(this->matrix * factor));
}

RealMatrixPtr RealMatrix::setValue(unsigned int i, unsigned int j, double value) const{
	if(getNumRows() <= i){
		throw invalid_argument("row index to high when setting value");	
	}
	if(getNumCols() <= j){
		throw invalid_argument("col index to high when setting value");	
	}
	RealMatrixPtr newMat(new RealMatrix(*this));
	newMat->matrix(i,j) = value;
	return newMat;
}

double RealMatrix::maxValue() const{
	double max = matrix(0,0);
	for(unsigned int i=0; i<rows; i++){
		for(unsigned int j=0; j<cols; j++){
			if(max < matrix(i,j)){
				max = matrix(i,j);
			}
		}
	}
	return max;
}

double RealMatrix::minValue() const{
	double min = matrix(0,0);
	for(unsigned int i=0; i<rows; i++){
		for(unsigned int j=0; j<cols; j++){
			if(matrix(i,j) < min){
				min = matrix(i,j);
			}
		}
	}
	return min;
}

RealMatrixPtr RealMatrix::componentMax(RealMatrixPtr mat) const{
	if(this->cols != mat->cols || this->rows != mat->rows){
		throw invalid_argument("Dimension mismatch in RealMatrix::componentMax");
	}
	RealMatrixPtr max(new RealMatrix(*this));
	for(unsigned int i=0; i<rows; i++){
		for(unsigned int j=0; j<cols; j++){
			if(this->matrix(i,j) < mat->matrix(i,j)){
				max->matrix(i,j) = mat->matrix(i,j);
			} else {
				max->matrix(i,j) = this->matrix(i,j);
			}
		}
	}
	return max;
}

RealMatrixPtr RealMatrix::componentMin(RealMatrixPtr mat) const{
	if(this->cols != mat->cols || this->rows != mat->rows){
		throw invalid_argument("Dimension mismatch in RealMatrix::componentMin");
	}
	RealMatrixPtr min(new RealMatrix(*this));
	for(unsigned int i=0; i<rows; i++){
		for(unsigned int j=0; j<cols; j++){
			if(this->matrix(i,j) < mat->matrix(i,j)){
				min->matrix(i,j) = this->matrix(i,j);
			} else {
				min->matrix(i,j) = mat->matrix(i,j);
			}
		}
	}
	return min;
}

RealMatrixPtr RealMatrix::abs() const{
    return RealMatrixPtr(new RealMatrix(this->matrix.cwiseAbs ()));

}

RealMatrixPtr RealMatrix::transpose() const{
	Eigen::MatrixXd tmp = this->matrix.transpose();
	EigenRowMatrix t = tmp;
	return RealMatrixPtr(new RealMatrix(t));
}

RealMatrixPtr RealMatrix::mult(RealMatrixPtr mat) const{
	if(this->getNumCols() != mat->getNumRows()){
		throw std::invalid_argument("RealMatrix::mult(RealMatrixPtr): dimension mismatch");
	}

	EigenRowMatrix m = this->matrix * mat->matrix;
	return RealMatrixPtr(new RealMatrix(m));
}

RealVectorPtr RealMatrix::mult(RealVectorPtr vec) const{
	// FIXED very (!) inefficient due to conversions
	//return vecFromMat(this->matrix * create(vec)->matrix);
	// the multiplication "by hand" is much faster
	// Factor 2.2 faster in Example app "ExpGoalBabbling1.2 -n 2 -m 10 -p 3 -v 20 -t 1 -c 50" 
	
	unsigned int vecDim = vec->getDimension();
	if(this->cols != vecDim){
		stringstream ss;
		ss << "RealMatrix::mult(RealVectorPtr): trying to multiply a";
		ss << rows << "x" << cols << " matrix with a " << vecDim << "dimensional vector.";
		throw invalid_argument(ss.str());
	}

	const double *vecData = vec->getValues();
	RealVectorPtr result = RealVector::create(rows, 0.0);
	// result is our private instance so far, we can write into its memory
	double *resultData = const_cast<double*>(result->getValues());
	for(unsigned int row = 0; row < rows; row++){
		const double *rowData = this->matrix.data() + row*cols;
		for(unsigned int col = 0; col < cols; col++){
			//resultData[row] += this->matrix(row, col) * vecData[col];
			// even 1.9 times faster with direct mem access
			resultData[row] += rowData[col] * vecData[col];
		}
	}
	
	return result;

	/*const double *vecData = vec->getValues();
	double *resultData = new double[rows];
	for(unsigned int row = 0; row < rows; row++){
		resultData[row] = 0.0;
		const double *rowData = this->matrix.data() + row*cols;
		for(unsigned int col = 0; col < cols; col++){
			//resultData[row] += this->matrix(row, col) * vecData[col];
			// even 1.9 times faster with direct mem access
			resultData[row] += rowData[col] * vecData[col];
		}
	}
	
	RealVectorPtr result = RealVector::create(rows, resultData);
	delete[](resultData);
	return result;*/
}

RealMatrixPtr RealMatrix::pseudoInverse() const{
	// SVD implementation only works for rows>=cols matrices
	// solution: transpose, invert, transpose in case of rows<cols

	// FIXED: Eigen2's SVD may loop forever, if it get NaN of infinity values
	for(unsigned int i=0; i<getNumRows(); i++){
		for(unsigned int k=0; k<getNumCols(); k++){
			double value = this->matrix(i,k);
			if(!isANumber(value)){
				throw std::runtime_error("RealMatrix::pseudoInverse(): matrix to be inverted contains NaN or inf values");
			}
		}
	}

	const double epsilon = 1E-9;
	
	if(getNumRows() < getNumCols()){
		EigenRowMatrix mat = this->matrix.transpose();
        Eigen::JacobiSVD<EigenRowMatrix> svd(mat);

		// invert singular values 
		Eigen::VectorXd singVals = svd.singularValues();
		double maxSingVal = 0.0;
		for(int i=0; i<singVals.rows(); i++){
			if(maxSingVal < singVals(i)){
				maxSingVal = singVals(i);
			}
		}
		if(maxSingVal == 0.0){
			// we received a null matrix...
			// the pseudoinverse is also a null matrix!!!
			return this->transpose();
		}
		Eigen::VectorXd invSingVals = singVals;
		for(int i=0; i<singVals.rows(); i++){
			if(singVals(i) <= epsilon*maxSingVal){
				invSingVals(i) = 0.0; // FIXED can not be safely inverted
			}
			else{
				invSingVals(i) = 1.0 / invSingVals(i);
			}
		}
		
		// create pseudoinverse matrix
		Eigen::MatrixXd pInvTmp = svd.matrixV() * 
			invSingVals.asDiagonal() * 
			svd.matrixU().transpose();
		Eigen::MatrixXd pInvTmp2 = pInvTmp.transpose();
		EigenRowMatrix pInv = pInvTmp2; // computing as ColMajor and then assign works


		return RealMatrixPtr(new RealMatrix(pInv));
	}
	else{
        Eigen::JacobiSVD<EigenRowMatrix> svd(this->matrix);

		// invert singular values
		Eigen::VectorXd singVals = svd.singularValues();
		double maxSingVal = 0.0;
		for(int i=0; i<singVals.rows(); i++){
			if(maxSingVal < singVals(i)){
				maxSingVal = singVals(i);
			}
		}
		if(maxSingVal == 0.0){
			// we received a null matrix...
			// the pseudoinverse is also a null matrix!!!
			return this->transpose();
		}
		Eigen::VectorXd invSingVals = singVals;
		for(int i=0; i<singVals.rows(); i++){
			if(singVals(i) <= epsilon*maxSingVal){
				invSingVals(i) = 0.0; // FIXED can not be safely inverted
			}
			else{
				invSingVals(i) = 1.0 / invSingVals(i);
			}
		}

		// create pseudoinverse matrix
		Eigen::MatrixXd pInvTmp = svd.matrixV() * 
			invSingVals.asDiagonal() * 
			svd.matrixU().transpose();
		EigenRowMatrix pInv = pInvTmp;
	
		return RealMatrixPtr(new RealMatrix(pInv));
	}
}
RealMatrixPtr RealMatrix::sqrt() const{
	// SVD implementation only works for rows>=cols matrices
	// solution: transpose, invert, transpose in case of rows<cols

	// FIXED: Eigen2's SVD may loop forever, if it get NaN of infinity values
	for(unsigned int i=0; i<getNumRows(); i++){
		for(unsigned int k=0; k<getNumCols(); k++){
			double value = this->matrix(i,k);
			if(!isANumber(value)){
				throw std::runtime_error("RealMatrix::pseudoInverse(): matrix to be inverted contains NaN or inf values");
			}
		}
	}

	//const double epsilon = 1E-9;
	
	if(getNumRows() < getNumCols()){
		EigenRowMatrix mat = this->matrix.transpose();
        Eigen::JacobiSVD<EigenRowMatrix> svd(mat);

		// invert singular values 
		Eigen::VectorXd singVals = svd.singularValues();
		//double maxSingVal = 0.0;
		//for(int i=0; i<singVals.rows(); i++){
		//	if(maxSingVal < singVals(i)){
		//		maxSingVal = singVals(i);
		//	}
		//}
		//if(maxSingVal == 0.0){
			// we received a null matrix...
			// the pseudoinverse is also a null matrix!!!
			return this->transpose();
		//}
		Eigen::VectorXd sqrtSingVals = singVals;
		for(int i=0; i<singVals.rows(); i++){
			sqrtSingVals(i) = std::sqrt(singVals(i));
		}
		
		// create pseudoinverse matrix
		Eigen::MatrixXd sqrtTmp = svd.matrixV() * 
			sqrtSingVals.asDiagonal() * 
			svd.matrixU().transpose();
		Eigen::MatrixXd sqrtTmp2 = sqrtTmp.transpose();
		EigenRowMatrix sqrtT = sqrtTmp2; // computing as ColMajor and then assign works


		return RealMatrixPtr(new RealMatrix(sqrtT.transpose()));
	}
	else{
        Eigen::JacobiSVD<EigenRowMatrix> svd(this->matrix);

		// invert singular values
		Eigen::VectorXd singVals = svd.singularValues();

		Eigen::VectorXd sqrtSingVals = singVals;
		for(int i=0; i<sqrtSingVals.rows(); i++){
			sqrtSingVals(i) = std::sqrt(singVals(i));
		}

		// create pseudoinverse matrix
		Eigen::MatrixXd sqrtTmp = svd.matrixV() * 
			sqrtSingVals.asDiagonal() * 
			svd.matrixU().transpose();
		EigenRowMatrix sqrtT = sqrtTmp;
	
		return RealMatrixPtr(new RealMatrix(sqrtT.transpose()));
	}
}

RealMatrixPtr RealMatrix::orthonormalize() const{
	// FIXED: Eigen2's SVD may loop forever, if it get NaN of infinity values
	for(unsigned int i=0; i<getNumRows(); i++){
		for(unsigned int k=0; k<getNumCols(); k++){
			double value = this->matrix(i,k);
			if(!isANumber(value)){
				throw std::runtime_error("RealMatrix::pseudoInverse(): matrix to be inverted contains NaN or inf values");
			}
		}
	}
	
	if(getNumRows() != getNumCols()){
		throw std::logic_error( "RealMatrix::orthonormalize(): only quadratic matrices can be orthonormalized" );
	}

    Eigen::JacobiSVD<EigenRowMatrix> svd(this->matrix);
	Eigen::MatrixXd normalized = svd.matrixU() * svd.matrixV().transpose();
	
	return RealMatrixPtr(new RealMatrix(normalized));
}


