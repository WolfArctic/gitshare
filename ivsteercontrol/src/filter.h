#ifndef _FILTER_H
#define _FILTER_H
#pragma once
#include <vector>
#include "steercontrol.h"

class Filter{
public:
	Filter();
	~Filter();
	void mean_filter_init(mean_filter_param *mf_param, int window_size);
	double mean_filter_apply(mean_filter_param *mf_param, double in_data);
	std::vector<double> double_sort(std::vector<double> data);
};

#endif
