#ifndef __JMT_HPP__
#define __JMT_HPP__

#include <vector>

using std::vector;

vector<double> GetJmtTrajectoryCoeffs(
        const vector<double>& start,
        const vector<double>& end,
        double t);

double EvaluatePolynomeAt(const vector<double>& coeffs,
                          double t);

#endif  //  __JMT_HPP__
