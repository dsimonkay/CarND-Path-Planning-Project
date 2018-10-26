
#ifndef CARND_PATH_PLANNER_PARAMETERS_H
#define CARND_PATH_PLANNER_PARAMETERS_H

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

// lane width
const double LANE_WIDTH = 4.0;            // meters

// a theoretical vehicle length
const double VEHICLE_LENGTH = 6.0;        // meters

// number of waypoints predicted by the planner
const int PREDICTED_WAYPOINTS = 50;

// time slot
const double TIME_SLOT = 0.02;        // seconds

// distance limit between the ego vehicle and the car in front of us
// where the ego vehicle starts to think about a lane change to pass that car
const double PROXIMITY_FRONTIER = 30.0;   // meters
const double MIN_GAP = 15;                // meters

// used reference velocity for maximum speed
const double MAX_V = 22.25;          // 22.25 meter/second = 49.771832 miles/hour

// the absolute value of the velocity change must not exceed this threshold
const double MAX_V_CHANGE = 0.25;         // 0.3 m/s within about 3*0.02 or 4*0.02 seconds ~ 4 m/s^2

#endif /* CARND_PATH_PLANNER_PARAMETERS_H */