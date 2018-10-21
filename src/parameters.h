
#ifndef CARND_PATH_PLANNER_CONSTANS_H
#define CARND_PATH_PLANNER_CONSTANS_H

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

// lane width
const double LANE_WIDTH = 4.0;  // meters

// number of waypoints predicted
const int WAYPOINTS = 50;

// number of previous waypoints used
const int PREV_WAYPOINTS_USED = 20;

// time interval
const double TIME_INTERVAL = 0.02;  // seconds

// maximum allowed distance between the ego vehicle and the car in front of us
const double PROXIMITY_FRONTIER = 30.0;

// used reference velocity for maximum speed
const double REFERENCE_V = 22.1;    // 22.1 meter/second = 49.504 miles/hour

// velocity changing unit
const double UNIT_CHANGE_V = 0.4;   // m/s within 0.02 seconds

#endif /* CARND_PATH_PLANNER_CONSTANS_H */