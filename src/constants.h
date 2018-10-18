
#ifndef CARND_PATH_PLANNER_CONSTANS_H
#define CARND_PATH_PLANNER_CONSTANS_H

// lane width
const double LANE_WIDTH = 4.0;  // meters

// number of waypoints predicted
const int WAYPOINTS = 50;

// number of previous waypoints used
const int PREV_WAYPOINTS_USED = 20;

// time interval
const double TIME_INTERVAL = 0.02;  // seconds

// maximum allowed distance between the ego vehicle and the car in front of us
const double PROXIMITY_GAP = 30.0;

// used reference velocity for maximum speed
const double REFERENCE_V = 22.1;    // 22.1 meter/second = 49.504 miles/hour

#endif /* CARND_PATH_PLANNER_CONSTANS_H */