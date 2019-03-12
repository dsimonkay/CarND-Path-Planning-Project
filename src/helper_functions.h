#ifndef CARND_PATH_PLANNER_HELPER_FUNCTIONS_H
#define CARND_PATH_PLANNER_HELPER_FUNCTIONS_H

#include <map>
#include <string>
#include <vector>


// Data structure used for prediction
struct VehicleInfo {
  int id;
  int lane;
  double vx;
  double vy;
  double speed;
  double s;
  double d;
  double distance;          // difference between the S coordinates of this car and the ego vehicle
  double future_s;
  double future_d;
  double future_distance;   // future S coordinate difference
};


/*
 * Correcting the anomaly that can arise due to the closed nature of this highway
 *
 * Parameters:
 *  - s1: first S value
 *  - s2: second S value. This is the reference point, that is, we are seeking the distance of S1 from S2.
 *        (The distance is positive if S1 is ahead of S2 and negative in the other case.)
 *
 * Returns: the true distance (S1 - S2) of the two S values, considering the closed-loop nature of the highway.
 */
double s_difference(double s1, double s2);



/* Function to extract the nearest vehicles in the current and the neighboring lanes relative to the ego vehicle.
 *
 * Parameters:
 *  - sensor_fusion:              a vector consisting of the following data: [id, x, y, vx, vy, s, d]
 *  - ego_lane:                   the current lane (0, 1, 2) of the ego vehicle
 *  - ego_s_now:                  actual coordinate of the ego vehicle along the S axis
 *  - ego_s_future:               future coordinate of the ego vehicle along the S axis
 *  - proximity_gap:              a minimum distance that has to be kept to the vehicle ahead
 *  - prediction_horizon:         number of time intervals we want to calculate the future
 *  - left_lane_change_feasible:  [IN/OUT]  the name says it :)
 *  - right_lane_change_feasible: [IN/OUT]  the name says it :)
 *  - lane_change_needed:         [IN/OUT]  the name says it :)
 *  - debug:                      flag controlling whether the function should produce a debug output to the console
 *
 * On return the following keys will be provided in the resulting map:
 *  - left_ahead
 *  - left_behind
 *  - center_ahead
 *  - center_behind
 *  - right_ahead
 *  - right_behind
 *
 * The terms "left", "center" and "right" are relative to the ego vehicle's current lane.
 * In case no vehicle can be detected in the give location, the "id" member of the VehicleInfo
 * data structure will be set to -1.
 */
map<string, VehicleInfo> processSensorFusionData(const std::vector< std::vector<double> > &sensor_fusion,
                                                 int ego_lane,
                                                 double ego_s_now,
                                                 double ego_s_future,
                                                 double proximity_gap,
                                                 int prediction_horizon,
                                                 bool &left_lane_change_feasible,
                                                 bool &right_lane_change_feasible,
                                                 bool &lane_change_needed,
                                                 bool debug);



/* This function decides in which lane and by which speed the ego vehicle should continue its route.
 *
 * Parameters:
 *  - nearest_vehicles:           data delivered by processSensorFusionData()
 *  - ego_lane:                   [IN/OUT]  the current lane (0, 1, 2) of the ego vehicle
 *  - ego_v:                      [IN/OUT]  ego vehicle reference velocity
 *  - ego_s_now:                  actual coordinate of the ego vehicle along the S axis
 *  - ego_s_future:               future coordinate of the ego vehicle along the S axis
 *  - proximity_gap:              a minimum distance that has to be kept to the vehicle ahead
 *  - lane_change_in_progress:    the name says it :)
 *  - left_lane_change_feasible:  as determined by processSensorFusionData()
 *  - right_lane_change_feasible: as determined by processSensorFusionData()
 *  - lane_change_needed:         as determined by processSensorFusionData()
 *  - debug:                      flag controlling whether the function should produce a debug output to the console
 */
void decideWhatToDo(std::map<std::string, VehicleInfo> nearest_vehicles,
                    int &ego_lane,
                    double &ego_v,
                    double ego_s_now,
                    double ego_s_future,
                    double proximity_gap,
                    bool lane_change_in_progress,
                    bool left_lane_change_feasible,
                    bool right_lane_change_feasible,
                    bool lane_change_needed,
                    bool debug );



#endif /* CARND_PATH_PLANNER_HELPER_FUNCTIONS_H */
