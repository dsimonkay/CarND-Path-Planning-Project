
#ifndef CARND_PATH_PLANNER_CONSTANS_H
#define CARND_PATH_PLANNER_CONSTANS_H

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

// lane width
const double LANE_WIDTH = 4.0;            // meters

// vehicle length
const double VEHICLE_LENGTH = 5.0;        // meters

// number of waypoints predicted by the planner
const int PREDICTED_WAYPOINTS = 50;

// time interval
const double TIME_INTERVAL = 0.02;        // seconds

// distance limit between the ego vehicle and the car in front of us
// where the ego vehicle starts to think about a lane change to pass that car
const double PROXIMITY_FRONTIER = 30.0;   // meters

// used reference velocity for maximum speed
const double REFERENCE_V = 22.2;          // 22.2 meter/second = 49.65999 miles/hour

// the absolute value of the velocity change must not exceed this threshold
const double MAX_V_CHANGE = 0.07;         // 0.07 m/s within 0.02 seconds = 3.5 m/s^2

#endif /* CARND_PATH_PLANNER_CONSTANS_H */