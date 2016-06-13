#ifndef _ADAPTIVE_LINEAR_MAPPING_H_
#define _ADAPTIVE_LINEAR_MAPPING_H_

#include <string>
#include <map>
#include <stdexcept>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include "RealVector.h"
#include "RealMatrix.h"
#include "Mapping.h"
#include "TimeSeries.h"

#include "AdaptiveMapping.h"

class AdaptiveLinearMapping;
typedef boost::shared_ptr<AdaptiveLinearMapping> AdaptiveLinearMappingPtr;

class AdaptiveLinearMapping : public AdaptiveMapping {
public:
	static AdaptiveLinearMappingPtr create(unsigned int inDim, unsigned int outDim, bool withOffset=false);

	static AdaptiveLinearMappingPtr create(RealMatrixPtr weights, RealVectorPtr offset = RealVectorPtr(NULL));

	static AdaptiveLinearMappingPtr solve(TimeSeriesPtr input, TimeSeriesPtr targetOutput, bool withOffset=false);

	static AdaptiveLinearMappingPtr solve(TimeSeriesPtr input, TimeSeriesPtr targetOutput, TimeSeriesPtr sampleWeights, bool withOffset=false);

	static AdaptiveLinearMappingPtr solve(RealMatrixPtr input, RealMatrixPtr targetOutput, bool withOffset=false);

	RealVectorPtr evaluate(RealVectorPtr value);

  	MappingPtr invert();

	RealMatrixPtr getJacobian(RealVectorPtr value);

	virtual void adaptToTarget(RealVectorPtr input, RealVectorPtr target, double weight=1.0);
	// error = target-actual
	virtual void adaptByError(RealVectorPtr input, RealVectorPtr error, double weight=1.0);

	void setWeightMatrix(RealMatrixPtr matrix);
	RealMatrixPtr getWeightMatrix();

	void setOffsetVector(RealVectorPtr vec);
	RealVectorPtr getOffsetVector();

	// push mapping-output (for <input>) toward target vector
	virtual void adaptToTargets(TimeSeriesPtr input, TimeSeriesPtr targetOutput, TimeSeriesPtr weights = TimeSeriesPtr());

	virtual void adaptByError(TimeSeriesPtr input, TimeSeriesPtr error, TimeSeriesPtr weights = TimeSeriesPtr());

	virtual ~AdaptiveLinearMapping();

protected:
	AdaptiveLinearMapping(unsigned int inDim, unsigned int outDim, bool withOffset);
	
	AdaptiveLinearMapping(RealMatrixPtr weights, RealVectorPtr offset);

private:
	MappingPtr getTempPointer();
	static TimeSeriesPtr multiplyWeights(TimeSeriesPtr data, TimeSeriesPtr weights);
	
	// weight matrix
	RealMatrixPtr matrix;
	RealVectorPtr offset;
};

#endif
