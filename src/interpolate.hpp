#ifndef __INTERPOLATE_HPP__
#define __INTERPOLATE_HPP__

#include <vector>

using std::vector;

vector<double> Interpolate(const vector<double>& ref_v,
                           const vector<double>& ref_fn,
                           const vector<double>& points);

#endif  // __INTERPOLATE_HPP__
