#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__

// Maximum number of points to keep from previously calculated path.
static constexpr int kMaxPreviousPathToKeep = 25;

// How many waypoint behind and ahead to consider for interpolation of the
// waypoints.
constexpr int kWaypointsBehind = 5;
constexpr int kWaypointsAhead = 5;

// The distance between interpolated waypoints.
constexpr double kDetailedWaypointDistance = 0.001;

// The total length of the track in meters.
constexpr double kTrackLength = 6945.554;

// Time between two path points.
constexpr double kInterpointTime = 0.02;

// Number of points in the path.
constexpr int kTotalPointsInPath = 50;

constexpr int kPlanningTrajectoryPoints = 75;

constexpr double kMaxSpeedLimit = 45.0 * 0.44704;
constexpr double kMaxAllowedOverDist = 0.001;
constexpr double kMinSpeedLimit = 10.0 * 0.44704;

constexpr int kOtherVehiclesTrajectoriesToGenerate = 1;
constexpr double kOtherVehicleSpeedStep = 4.0 * 0.44704;

constexpr int kTrajectoriesToGenerate = 4;

constexpr double kSpeedStep = 2.0 * 0.44704;

constexpr double kLaneWidthMeters = 4.0;
constexpr double kMinCrossLineDDistance = 3.0;

// Minimal distance to keep from the car ahead.
constexpr double kMinimalAllowedAheadDistance = 30.0;
constexpr double kMinimalAllowedBehindDistance = 10.0;
constexpr double kMaxAheadDistanceToConsider = 100.0;
constexpr double kMaxBehindDistanceToConsider = 20.0;

constexpr double kMaxAllowedSpeed = kMaxSpeedLimit + 2.0 * 0.44704;
constexpr double kMaxAllowedAccl = 0.125;

constexpr double kDistanceAheadCostMultiplier = 25;
constexpr double kDistanceBehindCostMultiplier = 3;
constexpr double kSpeedCostMultiplier = 3;
constexpr double kLaneCostMultiplier = 0.5;

constexpr double kLaneChangeOffset = 1.0;

#endif  //  __PARAMETERS_HPP__
