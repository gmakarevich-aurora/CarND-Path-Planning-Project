#ifndef __VEHICLE_HPP__
#define __VEHICLE_HPP__

#include <vector>

#include "waypoints.hpp"

using std::vector;

struct LocalizationData {
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double speed_;
    double end_path_s_;
    double end_path_d_;
};

using TrajectoryPoint = vector<double>;
using Trajectory = vector<TrajectoryPoint>;

class Vehicle {
 public:
    virtual ~Vehicle() = default;
    Vehicle(double s, double d, double s_d, double d_d)
        : s_(s), d_(d), s_d_(s_d), d_d_(d_d) {}

    // Make predictions about vehicle trajectories.
    //
    // points_to_generate: how many points to_generate.
    // points_to_start: how many points before time 0.
    vector<Trajectory> PredictTrajectories(
            int points_to_generate, int points_to_start) const;

 private:
    Trajectory PredictTrajectory(
            double s_d, int points_to_generate,
            int points_to_start) const;

    // S-coordinate at time 0.
    double s_;
    // D-coordinate at time 0.
    double d_;
    // S-velocity
    double s_d_;
    // D-velocity
    double d_d_;
};

class EgoVehicle {
 public:
    virtual ~EgoVehicle() = default;

    EgoVehicle(
        const vector<double>& previous_x,
        const vector<double>& previous_y,
        const WaypointsDescriptor& waypoints,
        const LocalizationData& localization_data);

    vector<Trajectory> GetPossibleTrajectories(
            int points_to_generate) const;

    void FillXYTrajectory(const Trajectory& trajectory,
                          const WaypointsDescriptor& waypoints,
                          vector<double>* next_x,
                          vector<double>* next_y);

 private:
    Trajectory GetTrajectoryForTarget(
            double s_f, double s_d_f, double s_dd_f,
            double d_f, double d_d_f, double d_dd_f,
            double duration, int points) const;

    LocalizationData localization_data_;
    vector<double> previous_x_;
    vector<double> previous_y_;

    double s_;    // s at the end of previously calculated path.
    double s_d_;  // s'
    double s_dd_; // s''
    double d_;    // d
    double d_d_;  // d'
    double d_dd_; // d''
};

Trajectory FindBestTrajectory(const vector<Trajectory>& ego_trajectories,
                              const vector<Trajectory>& other_trajectories);


#endif  // __VEHICLE_HPP__
