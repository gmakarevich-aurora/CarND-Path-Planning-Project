# Model Documentation

The task of the model is to generate the trajectory points for the car.
The input of the model is the previously generated path points, not yet used by
the simulator, the current coordinates and speed of the car, and current coordinates
and speed of other cars on the same side of the road.

All modelling is performed in freenet coordinates (s, d). So, the first step is
to identify s,d parameters (s,d, s', d', s'', d'') of the ego car at last point
of not-yet used previously generated path. If there is not enough points,
then current parameters reported by the simulator are used. The code for the calculation
of the Ego vehicle state is in function EgoVehicle::EgoVehicle.

At next step, the model generates multiple possible final states. in which the Ego vehicle
may be after next 55 (kPlanningTrajectoryPoints parameter) time points. The final states
differ by the final lane (all three lanes are considered), and final speed. The final speed
is considered in the range with the center at the current speed and step, equal to 2mph
(parameter kSpeedStep). The model does not consider the final states with the final speed
in excess of the maximum speed limit (kMaxSpeedLimit). Hence, at maximum 45 final states are
considered ((2 * kTrajectoriesToGenerate + 1) * 3). The code for this generation is at
function EgoVehicle::GetPossibleTrajectories. The change of s-coordinate of the state is
calculated as average speed (s_start + s_final)/2 multiplied by overall trajectory duration.
The final acceleration is set up to 0. The final d-coordinate is set to match the center of new
lane, the final d-speed and d-accelleration is set to 0.

For each final state the trajectory parameters are estimated using jmt-algorithm,
described in the lecture. The coordinates are further adjusted to guarantee that
s,d-coordinates changes do not exceed the maximum speed limit (
the code is in the function EgoVehicle::GetTrajectoryForTarget)


Similarly, for all other vehicles, we adjust s-coordinate to the time-point,
corresponding to the last re-used path point. For each other vehicle the model generates
three trajectories assuming straight move in s,d coordinates with s-speed in the range
centered at car's current speed with step kOtherVehicleSpeedStep in both directions. The code
to generate other cars trajectories is at Vehicle::PredictTrajectory function.

For each ego vehicle trajectory the model calculates the cost using other vehicles trajectories as
input. The code is at function GetTrajectoryCost. The cost function considers the minimal distance
to the car ahead and behind. The cost multiplier for the distance is dramatically increased once the distance
gets less than minimally allowed. The trajectory cost also includes speed cost, which is lineary dependent
on the difference between the speed and maximum allowed speed. If the minimal distance to the car ahead
gets less than minimally allowed, the speed cost multiplier is set to 0, to stimulate the speed drop.
To prevent random changes of the lanes, there is cost associated with lane change.

The ego trajectory with minimal cost is selected and it's s,d coordinates are transformed into x,y global coordinates.


