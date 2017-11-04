#include "vehicle.hpp"

#include <iostream>
#include <limits>

#include "coordinates.hpp"
#include "jmt.hpp"
#include "interpolate.hpp"
#include "parameters.hpp"
#include "spline.h"

Trajectory Vehicle::PredictTrajectory(double s_d, int points_to_generate,
        int points_to_start) const {
    double s_offset = s_ + s_d * points_to_start * kInterpointTime;
    Trajectory trajectory;
    for (int i = 0; i < points_to_generate; ++i) {
        double s = s_offset + s_d * i * kInterpointTime;
        TrajectoryPoint trajectory_point = { s, d_ };
        trajectory.emplace_back(std::move(trajectory_point));
    }
    return trajectory;
}

vector<Trajectory> Vehicle::PredictTrajectories(int points_to_generate,
        int points_to_start) const {
    vector<Trajectory> trajectories;
    for (int i = -kOtherVehiclesTrajectoriesToGenerate;
            i < kOtherVehiclesTrajectoriesToGenerate; ++i) {
        double s_d = s_d_ + i * kOtherVehicleSpeedStep;
        if (s_d < 0) {
            s_d = 0;
        }
        Trajectory t = PredictTrajectory(s_d, points_to_generate,
                points_to_start);
        trajectories.emplace_back(std::move(t));
    }
    return trajectories;
}

EgoVehicle::EgoVehicle(const vector<double>& previous_x,
        const vector<double>& previous_y, const WaypointsDescriptor& waypoints,
        const LocalizationData& localization_data) :
        localization_data_(localization_data),
        previous_x_(previous_x),
        previous_y_(previous_y) {

    int subpath_size = previous_x.size();
    if (subpath_size < 3) {
        s_ = localization_data.s_;
        d_ = localization_data.d_;
        s_d_ = localization_data.speed_;
        d_d_ = 0;
        s_dd_ = 0;
        d_dd_ = 0;
        return;
    }

    double x_0 = previous_x[subpath_size - 1];
    double y_0 = previous_y[subpath_size - 1];

    double x_1 = previous_x[subpath_size - 2];
    double y_1 = previous_y[subpath_size - 2];
    double angle = atan2(y_0 - y_1, x_0 - x_1);
    vector<double> frenet = getFrenet(x_0, y_0, angle, waypoints.x, waypoints.y,
            waypoints.s);
    double s_0 = frenet[0];
    double d_0 = frenet[1];

    int next_wp_idx = NextWaypoint(x_0, y_0, angle, waypoints.x, waypoints.y);
    double dx_0 = waypoints.dx[next_wp_idx - 1];
    double dy_0 = waypoints.dy[next_wp_idx - 1];

    double sx_0 = -dy_0;
    double sy_0 = dx_0;

    double v_x_1 = (x_0 - x_1) / kInterpointTime;
    double v_y_1 = (y_0 - y_1) / kInterpointTime;

    double s_d_0 = v_x_1 * sx_0 + v_y_1 * sy_0;
    double d_d_0 = v_x_1 * dx_0 + v_y_1 * dy_0;

    double x_2 = previous_x[subpath_size - 3];
    double y_2 = previous_y[subpath_size - 3];
    double v_x_2 = (x_1 - x_2) / kInterpointTime;
    double v_y_2 = (y_1 - y_2) / kInterpointTime;
    double x_dd = (v_x_1 - v_x_2) / kInterpointTime;
    double y_dd = (v_y_1 - v_y_2) / kInterpointTime;
    double s_dd = x_dd * sx_0 + y_dd * sy_0;
    double d_dd = x_dd * dx_0 + y_dd * dy_0;

    s_ = s_0;
    d_ = d_0;
    s_d_ = s_d_0;
    d_d_ = d_d_0;
    s_dd_ = s_dd;
    d_dd_ = d_dd;
}

vector<Trajectory> EgoVehicle::GetPossibleTrajectories(
        int points_to_generate) const {
    double time_to_plan = points_to_generate * kInterpointTime;
    vector<Trajectory> trajectories;
    for (int i = -kTrajectoriesToGenerate; i <= kTrajectoriesToGenerate; ++i) {
        double s_d_final = s_d_ + i * kSpeedStep;
        if (s_d_final <= 0) {
            continue;
        }
        if (s_d_final > kMaxSpeedLimit) {
            s_d_final = kMaxSpeedLimit;
        }
        double s_d_average = std::min(fabs(s_d_final + s_d_) / 2,
                kMaxSpeedLimit);
        double s_final = s_ + s_d_average * time_to_plan;

        for (int lane = 0; lane < 3; ++lane) {
            double d_final = 2.0 + lane * kLaneWidthMeters;
            Trajectory trajectory = GetTrajectoryForTarget(s_final, s_d_final,
                    0.0, d_final, 0.0, 0.0, time_to_plan, points_to_generate);
            trajectories.emplace_back(std::move(trajectory));
        }
        if (s_d_final == kMaxSpeedLimit) {
            break;
        }
    }
    return trajectories;
}

Trajectory EgoVehicle::GetTrajectoryForTarget(double s_f, double s_d_f,
        double s_dd_f, double d_f, double d_d_f, double d_dd_f, double duration,
        int points) const {
    auto s_coeffs = GetJmtTrajectoryCoeffs( { s_, s_d_, s_dd_ }, { s_f, s_d_f,
            s_dd_f }, duration);
    auto d_coeffs = GetJmtTrajectoryCoeffs( { d_, d_d_, d_dd_ }, { d_f, d_d_f,
            d_dd_f }, duration);

    double s_prev = s_;
    double d_prev = d_;
    Trajectory trajectory;
    for (double i = 0; i < points; ++i) {
        double t = i * kInterpointTime;
        double s = EvaluatePolynomeAt(s_coeffs, t);
        double d = EvaluatePolynomeAt(d_coeffs, t);
        double dist = sqrt(pow((s - s_prev), 2) + pow((d - d_prev), 2));
        if (dist > kMaxSpeedLimit * kInterpointTime) {
            double multiplier = kMaxSpeedLimit * kInterpointTime / dist;
            s = s_prev + (s - s_prev) * multiplier;
            d = d_prev + (d - d_prev) * multiplier;
        }
        s_prev = s;
        d_prev = d;
        TrajectoryPoint point = { s, d };
        trajectory.emplace_back(std::move(point));
    }
    return trajectory;
}

void EgoVehicle::FillXYTrajectory(const Trajectory& trajectory,
        const WaypointsDescriptor& waypoints, vector<double>* next_x,
        vector<double>* next_y) {
    vector<double> s_traj;
    vector<double> x_traj;
    vector<double> y_traj;

    int previous_path_size = previous_x_.size();
    if (previous_path_size >= 4) {
        double prev_s = s_ - s_d_ * kInterpointTime;
        s_traj.push_back(prev_s);
        x_traj.push_back(previous_x_[previous_path_size - 2]);
        y_traj.push_back(previous_y_[previous_path_size - 2]);
        s_traj.push_back(s_);
        x_traj.push_back(previous_x_[previous_path_size - 1]);
        y_traj.push_back(previous_y_[previous_path_size - 1]);
    } else {
        double prev_s = s_ - 1;
        double prev_x = localization_data_.x_ - cos(localization_data_.yaw_);
        double prev_y = localization_data_.y_ - sin(localization_data_.yaw_);
        s_traj.push_back(prev_s);
        x_traj.push_back(prev_x);
        y_traj.push_back(prev_y);
        s_traj.push_back(s_);
        x_traj.push_back(localization_data_.x_);
        y_traj.push_back(localization_data_.y_);
    }

    // Add points at s_ + 30, and s_ + 60.
    double s_30 = s_ + 30;
    double d_30 = (*trajectory.rbegin())[1];
    vector<double> xy_30 = getXY(s_30, d_30, waypoints.s, waypoints.x,
            waypoints.y);
    s_traj.push_back(s_30);
    x_traj.push_back(xy_30[0]);
    y_traj.push_back(xy_30[1]);
    double s_60 = s_ + 60;
    double d_60 = d_30;  // no change of the line
    vector<double> xy_60 = getXY(s_60, d_60, waypoints.s, waypoints.x,
            waypoints.y);
    s_traj.push_back(s_60);
    x_traj.push_back(xy_60[0]);
    y_traj.push_back(xy_60[1]);

    tk::spline s_x;
    s_x.set_points(s_traj, x_traj);
    tk::spline s_y;
    s_y.set_points(s_traj, y_traj);

    for (int i = 0; i < previous_x_.size(); ++i) {
        next_x->push_back(previous_x_[i]);
        next_y->push_back(previous_y_[i]);
    }

    //    double x_prev = localization_data_.x_;
    //    double y_prev = localization_data_.y_;
    //    if (!next_x->empty()) {
    //        x_prev = *(next_x->rbegin());
    //        y_prev = *(next_y->rbegin());
    //    }
    //    double s_prev = trajectory[0][0];
    for (int i = 0; i < kTotalPointsInPath - previous_x_.size(); ++i) {
        const TrajectoryPoint& point = trajectory[i + 1];
        double s = point[0];
        do {
            double x = s_x(s);
            double y = s_y(s);
            next_x->push_back(x);
            next_y->push_back(y);
            break;
        } while (true);
    }
}

std::pair<double, double> FindMinimalDistance(const Trajectory& ego_trajectory,
        const vector<Trajectory>& other_trajectories) {
    double minimal_ahead_distance = std::numeric_limits<double>::max();
    double minimal_behind_distance = std::numeric_limits<double>::max();

    double d_f = (*ego_trajectory.rbegin())[1];
    double d_s = (*ego_trajectory.begin())[1];
    bool lane_change = false;
    if (fabs(d_f - d_s) > kLaneChangeOffset) {
        lane_change = true;
    }
    for (int i = 0; i < ego_trajectory.size(); ++i) {
        const auto& ego_point = ego_trajectory[i];
        for (const auto& other : other_trajectories) {
            const auto& other_point = other[i];
            if (fabs(other_point[1] - ego_point[1]) < kMinCrossLineDDistance) {
                if (other_point[0] >= ego_point[0]) {
                    double distance = other_point[0] - ego_point[0];
                    if (distance < minimal_ahead_distance) {
                        minimal_ahead_distance = distance;
                    }
                } else if (lane_change
                        && (fabs(d_s - other_point[1]) > kMinCrossLineDDistance)) {
                    double distance = ego_point[0] - other_point[0];
                    if (distance < minimal_behind_distance) {
                        minimal_behind_distance = distance;
                    }
                }
            }
        }
    }
    return std::pair<double, double>(minimal_ahead_distance,
            minimal_behind_distance);
}

double FindAverageSpeed(const Trajectory& ego_trajectory) {
    return ((*ego_trajectory.rbegin())[0] - (*ego_trajectory.begin())[0])
            / (ego_trajectory.size() * kInterpointTime);
}

double FindDChange(const Trajectory& ego_trajectory) {
    return fabs((*ego_trajectory.rbegin())[1] - (*ego_trajectory.begin())[1]);
}

void ExtractTrajectoryParams(const Trajectory& ego_trajectory,
        double* average_speed, double* max_speed, double* max_accl,
        double* d_change) {
    double maximum_speed = 0;
    double maximum_accl = 0;
    double prev_speed = 0;
    for (int i = 1; i < ego_trajectory.size(); ++i) {
        double delta_s = (ego_trajectory[i][0] - ego_trajectory[i - 1][0]);
        double speed = delta_s / kInterpointTime;
        double accl = 0;
        if (i > 1) {
            accl = fabs(speed - prev_speed) / kInterpointTime;
            prev_speed = speed;
        }
        if (accl > maximum_accl) {
            maximum_accl = accl;
        }
        if (speed > maximum_speed) {
            maximum_speed = speed;
        }
    }
    *max_speed = maximum_speed;
    *max_accl = maximum_accl;
    *average_speed = FindAverageSpeed(ego_trajectory);
    *d_change = FindDChange(ego_trajectory);
}

double GetTrajectoryCost(const Trajectory& ego_trajectory,
        const vector<Trajectory>& other_trajectories) {
    double max_speed;
    double max_accl;
    double average_speed;
    double d_change;
    ExtractTrajectoryParams(ego_trajectory, &average_speed, &max_speed,
            &max_accl, &d_change);
    std::pair<double, double> minimal_distances = FindMinimalDistance(
            ego_trajectory, other_trajectories);
    double minimal_ahead_distance = minimal_distances.first;
    double minimal_behind_distance = minimal_distances.second;

    double distance_ahead_cost = 0;
    if (minimal_ahead_distance <= kMinimalAllowedAheadDistance) {
        distance_ahead_cost = pow(kDistanceAheadCostMultiplier, 2)
                / minimal_ahead_distance;
    } else {
        distance_ahead_cost = kDistanceAheadCostMultiplier
                / minimal_ahead_distance;
    }

    double distance_behind_cost = 0;
    if (minimal_behind_distance <= kMinimalAllowedBehindDistance) {
        distance_behind_cost = pow(kDistanceBehindCostMultiplier, 5)
                / minimal_behind_distance;
    } else if (minimal_behind_distance < kMaxBehindDistanceToConsider) {
        distance_behind_cost = kDistanceBehindCostMultiplier
                / minimal_behind_distance;
    }

    double speed_cost = 0;
    double speedCostMultiplier =
            minimal_ahead_distance < kMinimalAllowedAheadDistance ?
                    0 : kSpeedCostMultiplier;
    speed_cost = kSpeedCostMultiplier * fabs(kMaxAllowedSpeed - average_speed);

    double lane_change_cost = kLaneCostMultiplier * d_change;
    return distance_ahead_cost + distance_behind_cost + speed_cost
            + lane_change_cost;
}

Trajectory FindBestTrajectory(const vector<Trajectory>& ego_trajectories,
        const vector<Trajectory>& other_trajectories) {
    double best_cost = std::numeric_limits<double>::max();
    int best_idx = 0;
    for (int i = 0; i < ego_trajectories.size(); ++i) {
        double cost = GetTrajectoryCost(ego_trajectories[i],
                other_trajectories);
        if (cost < best_cost) {
            best_cost = cost;
            best_idx = i;
        }
    }
    return ego_trajectories[best_idx];
}

