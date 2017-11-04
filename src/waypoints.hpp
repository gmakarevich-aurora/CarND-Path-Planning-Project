#ifndef __WAYPOINTS_HPP__
#define __WAYPOINTS_HPP__

#include <vector>

using std::vector;

struct WaypointsDescriptor {
    vector<double> s;
    vector<double> x;
    vector<double> y;
    vector<double> dx;
    vector<double> dy;

    int size() const { return x.size(); }
};

WaypointsDescriptor InterpolateWaypoints(
        const WaypointsDescriptor& map_waypoints,
        int center_waypoints_idx);


#endif  // __WAYPOINTS_HPP__
