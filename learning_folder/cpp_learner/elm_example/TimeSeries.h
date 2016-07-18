#ifndef _TIME_SERIES_H_
#define _TIME_SERIES_H_

#include <stdexcept>
#include <vector>
#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
//#include "boost/date_time/posix_time/duration.hpp"

#include "RealVector.h"
#include "Mapping.h"


class TimeSeries;
typedef boost::shared_ptr<TimeSeries> TimeSeriesPtr;


const boost::posix_time::ptime TIME_ZERO = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
const boost::posix_time::ptime TIME_START = boost::posix_time::second_clock::local_time();

class TimeSeries {
public:
	static TimeSeriesPtr create();

	// copy
	static TimeSeriesPtr create(TimeSeriesPtr series);


	static TimeSeriesPtr load(std::string filename);
	void store(std::string filename);

	virtual ~TimeSeries();

	std::vector<boost::posix_time::ptime> getTimestamps();
	
	// in chronological order
	std::vector<RealVectorPtr> getValues();

	RealVectorPtr getValue(boost::posix_time::ptime time);

	RealVectorPtr getValue(unsigned int index);

	boost::posix_time::ptime getTimestamp(unsigned int index);

	RealVectorPtr interpolateValue(boost::posix_time::ptime time);

	// time must be higher than for all elements
	void attach(boost::posix_time::ptime time, RealVectorPtr value);

	// attach with current locel time
	void attach(RealVectorPtr value);

	void attach(TimeSeriesPtr series);

	unsigned int getLength();
	bool isEmpty();

	bool subsumes(boost::posix_time::ptime time);

	void removeFirst();
	void removeFirst(unsigned int n);

	TimeSeriesPtr map(MappingPtr mapping); 

private:
	TimeSeries();

	int indexOf(boost::posix_time::ptime);
	int indexOfFirstLaterTimestamp(boost::posix_time::ptime);

	std::vector<boost::posix_time::ptime> timestamps;
	std::vector<RealVectorPtr> values;
};


// std output operator
inline std::ostream& operator<<(std::ostream& out, const TimeSeriesPtr &series){
	if(series == NULL){
		return out << "NONE";
	}

	for(unsigned int t=0; t<series->getLength(); t++){
		out << to_simple_string(series->getTimestamp(t)) << "\t";
		out << series->getValue(t) << std::endl;
	}
	return out;
}


#endif
