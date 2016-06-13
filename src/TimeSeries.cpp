
#include "TimeSeries.h"

#include <iostream>
#include <fstream>


using namespace std;
using namespace boost::posix_time;


TimeSeriesPtr TimeSeries::create(){
	return TimeSeriesPtr(new TimeSeries());
}

TimeSeriesPtr TimeSeries::create(TimeSeriesPtr series){
	TimeSeries *copy = new TimeSeries();
	copy->timestamps = series->timestamps;
	copy->values = series->values;
	return TimeSeriesPtr(copy);
}

TimeSeriesPtr TimeSeries::load(std::string filename){
	ifstream is(filename.c_str());

	unsigned int length;
	is >> length;
	is >> ws;

	TimeSeriesPtr series = TimeSeries::create();

	for(uint t=0; t < length; t++){
		ptime time;
		is >> time;
		is >> ws;

		unsigned int dim;
		is >> dim;
		is >> ws;

		double *vals = new double[dim];

		for(uint d=0; d < dim; d++){
			is >> vals[d];
			is >> ws;
		}
		series->attach(time, RealVector::create(dim, vals));
		delete[](vals);
	}
	
	return series;
}

void TimeSeries::store(std::string filename){
	ofstream os(filename.c_str());

	os << getLength() << endl;
	for(uint t=0; t<getLength(); t++){
		os << timestamps[t] << "\t";
		const unsigned int dim = values[t]->getDimension();
		const double *vals = values[t]->getValues();
		os << dim << "\t";
		for(uint d=0; d<dim-1; d++){
			os << vals[d] << "\t";
		}
		os << vals[dim-1] << endl;
	}
	os.close();
}

TimeSeries::~TimeSeries(){}

std::vector<boost::posix_time::ptime> TimeSeries::getTimestamps(){
	return timestamps;
}

// in chronological order
std::vector<RealVectorPtr> TimeSeries::getValues(){
	return values;
}

RealVectorPtr TimeSeries::getValue(boost::posix_time::ptime time){
	int elemIndex = indexOf(time);
	if(elemIndex < 0){
		throw std::invalid_argument(std::string("No value defined for time: ") + to_simple_string(time));
	}
	return values[elemIndex];
}

RealVectorPtr TimeSeries::getValue(unsigned int index){
	if(timestamps.size() <= index){
		throw std::out_of_range("TimeSeries: index out of bounds");
	}
	return values[index];
}

boost::posix_time::ptime TimeSeries::getTimestamp(unsigned int index){
	if(timestamps.size() <= index){
		throw std::out_of_range("TimeSeries: index out of bounds");
	}
	return timestamps[index];
}

RealVectorPtr TimeSeries::interpolateValue(boost::posix_time::ptime time){
	if(getLength() < 2){
		throw std::length_error("TimeSeries must have at least two elements for interpolation.");
	}
	if( !subsumes(time) ){
		throw std::out_of_range("TimeSeries does not subsume timestamp for interpolation.");
	}
	
	int elemIndex = indexOf(time);
	if(elemIndex != -1){
		// exact match with stored vector!
		return getValue(elemIndex);	
	}
 
	// linear interpolation
	int firstLaterIndex = indexOfFirstLaterTimestamp(time);
	boost::posix_time::time_duration timeFrameDuration = getTimestamp(firstLaterIndex) - getTimestamp(firstLaterIndex-1);
	boost::posix_time::time_duration timeFramePassed = time - getTimestamp(firstLaterIndex-1);
	
	const double ratio = (double)timeFramePassed.total_microseconds() / (double)timeFrameDuration.total_microseconds();
	RealVectorPtr start = getValue(firstLaterIndex-1);
	RealVectorPtr end = getValue(firstLaterIndex);
	return start->plus( end->minus(start)->scale(ratio) );
}

// time must be higher than for all elements
void TimeSeries::attach(boost::posix_time::ptime time, RealVectorPtr value){
	if( !timestamps.empty() && time <= timestamps.back()){
		stringstream msg;
		msg << "Timestamp prior to last element not allowed. "
			<< "Last timestamp is \"" << timestamps.back() << "\", "
			<< "This timestamp is \"" << time << "\".";
		throw std::invalid_argument(msg.str());
	}

	timestamps.push_back(time);
	values.push_back(value);
}

void TimeSeries::attach(RealVectorPtr value){
	if(!isEmpty()){
		if(value->getDimension() != getValue(0)->getDimension()){
			throw std::invalid_argument("Vectors must have same dimensionality along timeseries.");	
		}	
	}
	boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	attach(time, value);
}

void TimeSeries::attach(TimeSeriesPtr series){
	// FIXME check timestamps OR shift timestamps of input appropriately
// 	std::vector<boost::posix_time::ptime> timestamps = series->timestamps;
// 	for(std::vector<boost::posix_time::ptime>::iterator it = timestamps.begin(); it != timestamps.end(); it++){
// 		this->attach(*it, series->getValue(*it));
// 	}
	const unsigned int N = series->getLength();
	for(unsigned int t=0; t<N; t++){
		attach(series->timestamps[t], series->values[t]);
	}
}

unsigned int TimeSeries::getLength(){
	return timestamps.size();
}

bool TimeSeries::isEmpty(){
	return timestamps.empty();
}

bool TimeSeries::subsumes(boost::posix_time::ptime time){
	return getTimestamp(0) <= time || time <=getTimestamp(getLength()-1);
}

void TimeSeries::removeFirst(){
	if(isEmpty()){
		throw std::length_error("Cannot remove first entry from empty timeseries");
	}
	values.erase(values.begin());
	timestamps.erase(timestamps.begin());
}

void TimeSeries::removeFirst(unsigned int n){
	if(getLength() < n){
		throw std::length_error("Cannot remove more entries than existing in TimeSeries");
	}
	
	values.erase(values.begin(), values.begin()+n);
	timestamps.erase(timestamps.begin(), timestamps.begin()+n);
	//for(unsigned int i = 0; i < n; i++){
		// FIXME very costly for std::vector
	//	removeFirst();
	//}
}

TimeSeriesPtr TimeSeries::map(MappingPtr mapping){
	TimeSeriesPtr mapped = TimeSeries::create();
	mapped->timestamps = this->timestamps;
	for(unsigned int i=0; i<mapped->timestamps.size(); i++){
// 		mapped->values[i]
		mapped->values.push_back(mapping->evaluate(this->values[i]));
	}
	return mapped;
}

TimeSeries::TimeSeries()
 : timestamps(), values()
{}

int TimeSeries::indexOf(boost::posix_time::ptime time){
	// FIXME faster search needed. either binary-search of other container
	int elemIndex = -1;
	for(int i = 0; i < (int)timestamps.size(); i++){
		if(timestamps[i] == time){
			elemIndex = i;
			break;
		}
	}
	return elemIndex;
}

int TimeSeries::indexOfFirstLaterTimestamp(boost::posix_time::ptime time){
	// FIXME faster search needed. either binary-search of other container
	int elemIndex = -1;
	for(int i = 0; i < (int)timestamps.size(); i++){
		if(time < timestamps[i]){
			elemIndex = i;
			break;
		}
	}
	return elemIndex;
}

