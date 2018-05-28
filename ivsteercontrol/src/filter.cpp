#include "filter.h"


Filter::Filter(){}
Filter::~Filter(){}
void Filter::mean_filter_init(mean_filter_param *mf_param, int window_size)
{
    mf_param->window_size = window_size;
    for(int i=0;i<window_size;i++)
        mf_param->data.push_back(0);
}
double Filter::mean_filter_apply(mean_filter_param *mf_param, double in_data)
{
    double output = 0;
    std::vector<double> v;
    for(int i=0; i<mf_param->window_size-1; i++)
        mf_param->data[i] = mf_param->data[i+1];

    mf_param->data[mf_param->window_size-1] = in_data;
    v = double_sort(mf_param->data);
    for(int i = 1; i<v.size()-1; i++)
        output = output+v[i];
    output = output/(mf_param->window_size - 2);
    return output;
}
std::vector<double> Filter::double_sort(std::vector<double> data)
{
    for(int i=0; i<data.size(); i++)
        for(int j=data.size()-1; j>i; j--)
            if(data[j]<data[j-1])
            {
                double temp = data[j-1];
                data[j-1] = data[j];
                data[j] = temp;
            }
    return data;
}
