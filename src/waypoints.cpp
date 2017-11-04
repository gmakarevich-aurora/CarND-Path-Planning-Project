#include "waypoints.hpp"

#include <iostream>
#include <vector>

#include "interpolate.hpp"
#include "parameters.hpp"

WaypointsDescriptor InterpolateWaypoints(
        const WaypointsDescriptor& map_waypoints,
        int center_waypoints_idx) {
    vector<double> waypoints_s;
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;
    double center_s = map_waypoints.s[center_waypoints_idx];
    for (int i = - kWaypointsBehind; i < kWaypointsAhead; ++i) {
        int idx = (center_waypoints_idx + i) % map_waypoints.size();
        if (idx < 0) {
            idx += map_waypoints.size();
        }
        double s = map_waypoints.s[idx];
        if (i < 0 && s > center_s) {
            s -= kTrackLength;
        }
        if (i > 0 && s < center_s) {
            s += kTrackLength;
        }
        waypoints_s.push_back(s);
        waypoints_x.push_back(map_waypoints.x[idx]);
        waypoints_y.push_back(map_waypoints.y[idx]);
        waypoints_dx.push_back(map_waypoints.dx[idx]);
        waypoints_dy.push_back(map_waypoints.dy[idx]);
    }

    vector<double> calculated_s;
    double s_start = *waypoints_s.begin();
    double s_end = *waypoints_s.rbegin();
    for (double s = s_start; s < s_end; s += kDetailedWaypointDistance) {
        calculated_s.push_back(s);
    }

    vector<double> calculated_x = Interpolate(waypoints_s, waypoints_x,
                                              calculated_s);
    vector<double> calculated_y = Interpolate(waypoints_s, waypoints_y,
                                              calculated_s);
    vector<double> calculated_dx = Interpolate(waypoints_s, waypoints_dx,
                                               calculated_s);
    vector<double> calculated_dy = Interpolate(waypoints_s, waypoints_dy,
                                               calculated_s);


    return WaypointsDescriptor {
        std::move(calculated_s),
        std::move(calculated_x),
        std::move(calculated_y),
        std::move(calculated_dx),
        std::move(calculated_dy)
    };
}

