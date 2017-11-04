#include "interpolate.hpp"
#include "spline.h"

vector<double> Interpolate(const vector<double>& ref_v,
                           const vector<double>& ref_fn,
                           const vector<double>& points) {
    tk::spline s;
    s.set_points(ref_v, ref_fn);
    vector<double> r;
    for (const auto& p : points) {
        r.push_back(s(p));
    }
    return r;
}
