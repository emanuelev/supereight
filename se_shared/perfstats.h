/*
 Copyright (c) 2011-2013 Gerhard Reitmayr, TU Graz

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef PERFSTATS_H
#define PERFSTATS_H

#ifdef __APPLE__
    #include <mach/clock.h>
    #include <mach/mach.h>
#endif


#include <algorithm>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <vector>
#include <ctime>
#include <iomanip>
#include <time.h>
#include <math.h>
#include <mutex>

struct PerfStats {
	enum Type {
		TIME,
		COUNT,
		PERCENTAGE,
		ENERGY,
		POWER,
		VOLTAGE,
		CURRENT,
		FREQUENCY,
		INT,
		DOUBLE,
		DISTANCE,
		FRAME,
		UNDEFINED
	};
	struct Stats {
		std::mutex mutex;
		std::vector<double> data;
		Type type;
		double lastAbsolute;
		double lastPeriod;
		double sum() const {
			return std::accumulate(data.begin(), data.end(), 0.0);
		}
		double average() const {
			return sum() / std::max(data.size(), size_t(1));
		}
		double max() const {
			return *std::max_element(data.begin(), data.end());
		}
		double min() const {
			return *std::min_element(data.begin(), data.end());
		}
	};
	struct Results {
		double mean;
		double sd;
		double min;
		double max;
		double sum;
	};
	int insertion_id;
	std::map<int, std::string> order;
	std::map<std::string, Stats> stats;
	double last;

	double get_time() {
#ifdef __APPLE__
		clock_serv_t cclock;
		mach_timespec_t clockData;
		host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
		clock_get_time(cclock, &clockData);
		mach_port_deallocate(mach_task_self(), cclock);
#else
		struct timespec clockData;
		clock_gettime(CLOCK_MONOTONIC, &clockData);
#endif
		return (double) clockData.tv_sec + clockData.tv_nsec / 1000000000.0;
	}

	double sample(const std::string& key, double t, Type type = COUNT) {
		double now = get_time();
		Stats& s = stats[key];

		s.mutex.lock();
		double last = s.lastAbsolute;
		if (last == 0) {
			order[insertion_id] = key;
			insertion_id++;
		}

		s.data.push_back(t);
		s.type = type;
		s.lastPeriod = now - last;
		s.lastAbsolute = now;

		s.mutex.unlock();
		return (now);
	}
	PerfStats() {
		insertion_id = 0;
	}
	double start(void) {
		last = get_time();
		return last;
	}
	double sample(const std::string &key) {
		const double now = get_time();
		sample(key, now - last, TIME);
		last = now;
		return now;
	}
	const Stats& get(const std::string& key) const {
		return stats.find(key)->second;
	}
	void reset(void) {
		stats.clear();
	}
	void reset(const std::string & key);
	double getLastData(const std::string & key);
	Type getType(const std::string & key);
	double getSampleTime(const std::string & key);
	void print(std::ostream& out = std::cout) const;
	void debug();
	void print_all_data(std::ostream& out, bool include_all_data = true) const;
	double lastTime;
};

inline void PerfStats::reset(const std::string & key) {
	std::map<std::string, Stats>::iterator s = stats.find(key);
	if (s != stats.end())
		s->second.data.clear();
}
/*
 inline void PerfStats::debug() {
 return;
 for (std::map<std::string,Stats>::const_iterator it=stats.begin(); it!=stats.end(); it++){
 for(std::vector<double>::const_iterator vit = it->second.data.begin(); vit != it->second.data.end(); vit++) {
 std::cout <<  it->first << "@ " << *vit << "\n";
 }
 }
 }
 */
inline void PerfStats::print(std::ostream& out) const {
	static int first = 1;
	if (first) {

		for (std::map<int, std::string>::const_iterator it = order.begin();
				it != order.end(); it++) {

			out << std::left << std::setw(10) << it->second << "\t";
		}
		out << std::endl;
		first = 0;
	}
	out.setf(std::ios::fixed, std::ios::floatfield);
	out.precision(10);
	//for (std::map<std::string,Stats>::const_iterator it=stats.begin(); it!=stats.end(); it++){
	for (std::map<int, std::string>::const_iterator kt = order.begin();
			kt != order.end(); kt++) {
		std::map<std::string, Stats>::const_iterator it = stats.find(
				kt->second);
		if (it == stats.end())
			continue;
		switch (it->second.type) {
		case TIME: {
			out << it->second.data.back() << "\t";
		}
			break;
		case COUNT: {
			out << it->second.data.back() << "\t";
		}
			break;
		case PERCENTAGE: {
			out << it->second.data.back() * 100.0 << "\t";
		}
			break;
		case DISTANCE:
		case POWER:
		case ENERGY:
		case DOUBLE: {
			out << it->second.data.back() << "\t";
		}
			break;
		case FRAME:
		case INT: {
			out << std::left << std::setw(10) << int(it->second.data.back())
					<< "\t";
		}

		default:
			break;
		}
	}
	out << std::endl;
}
inline double PerfStats::getLastData(const std::string & key) {
	std::map<std::string, Stats>::iterator s = stats.find(key);
	if (s != stats.end())
		return (s->second.data.back());

	return (0);
}

inline double PerfStats::getSampleTime(const std::string & key) {
	std::map<std::string, Stats>::iterator s = stats.find(key);
	if (s != stats.end()) {
		return (s->second.lastPeriod);
	}
	return (0.0);
}

inline PerfStats::Type PerfStats::getType(const std::string & key) {
	std::map<std::string, Stats>::iterator s = stats.find(key);
	if (s != stats.end())
		return (s->second.type);
	return (UNDEFINED);
}

inline void PerfStats::print_all_data(std::ostream& out, bool include_all_data) const {
	struct Results *res = NULL;
	struct Results *resPtr = NULL;
	int frames = 0;
	bool done = false;
	unsigned int idx = 0;

	resPtr = (struct Results *) malloc(sizeof(struct Results) * stats.size());
	out.precision(10);
	res = resPtr;
	//for (std::map<std::string,Stats>::const_iterator it=stats.begin(); it!=stats.end(); it++){
	for (std::map<int, std::string>::const_iterator kt = order.begin();
		 kt != order.end(); kt++) {
		std::map<std::string, Stats>::const_iterator it = stats.find(
				kt->second);
		if (it == stats.end())
			continue;

		(*res).mean = 0.0;
		(*res).min = 9e10;
		(*res).max = -9e10;
		(*res).sd = 0.0;
		res++;

	}
	out.precision(10);
	done = false;
	out.setf(std::ios::fixed, std::ios::floatfield);
	//while (!done) {
	res = resPtr;
	for (std::map<int, std::string>::const_iterator kt = order.begin();
		 kt != order.end(); kt++) {
		std::map<std::string, Stats>::const_iterator it = stats.find(
				kt->second);
		if (it == stats.end())
			continue;

		for (idx = 0; idx < it->second.data.size(); idx++) {
			(*res).mean = (*res).mean + it->second.data[idx];
			if (it->second.data[idx] > (*res).max)
				(*res).max = it->second.data[idx];
			if (it->second.data[idx] < (*res).min)
				(*res).min = it->second.data[idx];
			frames++;
		}
		(*res).sum = res->mean;
		(*res).mean = (*res).mean / idx;
		res++;
	}
//idx++;
	// }

	idx = 0;

	int count =0;

	if (include_all_data)
		std::cerr << " Done max min\n";

	while (count < insertion_id) {
		res = resPtr;
		for (std::map<int, std::string>::const_iterator kt = order.begin();
			 kt != order.end(); kt++) {
			std::map<std::string, Stats>::const_iterator it = stats.find(
					kt->second);
			if (it == stats.end()) {
				res++;
				continue;
			}


			if (idx < it->second.data.size()) {
				if (include_all_data) {
					switch (it->second.type) {
						case TIME: {
							out << it->second.data[idx] << "\t";
						}
							break;
						case COUNT: {
							out << it->second.data[idx] << "\t";
						}
							break;
						case PERCENTAGE: {
							out << it->second.data[idx] * 100.0 << "\t";
						}
							break;
						case DISTANCE:
						case POWER:
						case ENERGY:
						case DOUBLE: {
							out << it->second.data[idx] << "\t";
						}
							break;
						case FRAME:
						case INT: {
							out << std::left << std::setw(10)
							<< int(it->second.data[idx]) << "\t";
						}
						default:
							break;
					}
				}
				//  out << std::setw(10) << it->second.data[idx] << "\t"
				(*res).sd = (*res).sd
							+ ((it->second.data[idx] - (*res).mean)
							   * (it->second.data[idx] - (*res).mean));
			} else {
				if (idx == it->second.data.size())
					count++;
			}
			res++;
		}
		if (include_all_data && done) {
			out << "# End of file";
		}

		idx++;

		if (include_all_data)
			out << std::endl;
	}

	res = resPtr;

	int i = 0;
	int max = order.size();

	for (std::map<int, std::string>::const_iterator kt = order.begin(); kt != order.end(); kt++) {
		std::map<std::string, Stats>::const_iterator it = stats.find(kt->second);
		if (it == stats.end())
			continue;

		std::cout.precision(10);
		std::cout << "\"" << it->first << "\" : { ";

		std::cout << "\"mean\":\"" << (*res).mean << "\", ";
		std::cout << "\"std\":\"" << sqrt((*res).sd / idx)  << "\", ";
		std::cout << "\"min\":\"" << (*res).min << "\", ";
		std::cout << "\"max\":\"" << (*res).max << "\", ";
		std::cout << "\"sum\":\"" << (*res).sum << "\"";
		std::cout << "}";

		if (i + 1 != max)
		{
			std::cout << ", ";
		}

		++i;
		res++;
	}

	free(resPtr);
	return;

}

extern PerfStats Stats;

#endif // PERFSTATS_H
