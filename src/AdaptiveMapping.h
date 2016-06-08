#ifndef _ADAPTIVE_MAPPING_H_
#define _ADAPTIVE_MAPPING_H_

#include <string>
#include <map>

#include <boost/shared_ptr.hpp>

#include "RealVector.h"
#include "Mapping.h"
#include "TimeSeries.h"

class AdaptiveMapping;
typedef boost::shared_ptr<AdaptiveMapping> AdaptiveMappingPtr;


class AdaptiveMapping : public Mapping {
public:
	// ======= Un-Supervised Learning ========
// 	virtual void adapt(RealVectorPtr input);
	
	// ======= Supervised Online Learning ========
	// push mapping-output (for <input>) toward target vector
	virtual void adaptToTarget(RealVectorPtr input, RealVectorPtr target, double weight=1.0);
	// error = target-actual
	virtual void adaptByError(RealVectorPtr input, RealVectorPtr error, double weight=1.0) = 0;

	// ======= Supervised Batch Learning ========
	// push mapping-output (for <input>) toward target vector
	virtual void adaptToTargets(TimeSeriesPtr input, TimeSeriesPtr targetOutput, TimeSeriesPtr weights = TimeSeriesPtr());
	// error = target-actual
	virtual void adaptByError(TimeSeriesPtr input, TimeSeriesPtr error, TimeSeriesPtr weights = TimeSeriesPtr());

	// ======= Parameters of adaption/learning process ========
	virtual void setAdaptionParameter(std::string name, double value);
	virtual double adaptionParameter(std::string name);
	void setAdaptionRate(double value); // ^= setAdaptionParameter("rate", value); but faster
	double getAdaptionRate(); // ^= adaptionParameter("rate"); but faster
	


	virtual ~AdaptiveMapping();

protected:
	AdaptiveMapping(unsigned int inDim, unsigned int outDim);

private:
	std::map<std::string,double> param;
	double rate;

};

#endif
