#include "jmt.hpp"

#include "Eigen-3.3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> GetJmtTrajectoryCoeffs(
        const vector<double>& start,
        const vector<double>& end,
        double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    MatrixXd a(3, 3);
    a << t3, t4, t5, 3 * t2, 4 * t3, 5 * t4, 6 * t, 12 * t2, 20 * t3;
    MatrixXd aInv = a.inverse();

    VectorXd b(3);
    b << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
         end[1] - (start[1] + start[2] * t),
         end[2] - (start[2]);
    VectorXd alpha = aInv * b;

    vector<double> result = {
            start[0], start[1], 0.5 * start[2],
            alpha[0], alpha[1], alpha[2]
    };
    return result;
}

double EvaluatePolynomeAt(
        const vector<double>& coeffs,
        double t) {
    double result = 0;
    for (int i = 0; i < coeffs.size(); ++i) {
        result += coeffs[i] * pow(t, i);
    }
    return result;
}
