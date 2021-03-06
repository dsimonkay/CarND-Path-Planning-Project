#include <iostream>
#include <limits>
#include <map>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>
#include "spline.h"
#include "parameters.h"
#include "helper_functions.h"
#include "helper_functions_udacity.h"

using namespace tk;


double s_difference(double s1, double s2) {

  double s_diff = s1 - s2;

  // optimistically assuming a sensor range of +/- 250 meters
  if ( abs(s_diff) > MAX_S - 500 ) {

    // we've encountered a case where s1 and s2 are on two different sides of the origo of the loop
    s_diff += (s_diff < 0 ? MAX_S : -MAX_S);
  }

  return s_diff;
}


std::map<std::string, VehicleInfo> processSensorFusionData(const std::vector< std::vector<double> > &sensor_fusion,
                                                 int ego_lane,
                                                 double ego_s_now,
                                                 double ego_s_future,
                                                 double proximity_gap,
                                                 int prediction_horizon,
                                                 bool &left_lane_change_feasible,
                                                 bool &right_lane_change_feasible,
                                                 bool &lane_change_needed,
                                                 bool debug) {

  std::map<std::string, VehicleInfo> nearest_vehicles;

  // initializing the map to be returned
  nearest_vehicles["left_ahead"] = {-1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, numeric_limits<double>::max(), numeric_limits<double>::max(), 0.0, numeric_limits<double>::max()};
  nearest_vehicles["left_behind"] = {-1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, -numeric_limits<double>::max(), -numeric_limits<double>::max(), 0.0, -numeric_limits<double>::max()};
  nearest_vehicles["center_ahead"] = {-1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, numeric_limits<double>::max(), numeric_limits<double>::max(), 0.0, numeric_limits<double>::max()};
  nearest_vehicles["center_behind"] = {-1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, -numeric_limits<double>::max(), -numeric_limits<double>::max(), 0.0, -numeric_limits<double>::max()};
  nearest_vehicles["right_ahead"] = {-1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, numeric_limits<double>::max(), numeric_limits<double>::max(), 0.0, numeric_limits<double>::max()};
  nearest_vehicles["right_behind"] = {-1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, -numeric_limits<double>::max(), -numeric_limits<double>::max(), 0.0, -numeric_limits<double>::max()};

  // processing sensor fusion data
  for ( int i = 0;  i < sensor_fusion.size(); i++ ) {

    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_car_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = (double)(sensor_fusion[i][5]);
    double check_car_d = (double)(sensor_fusion[i][6]);
    double check_car_lane = (int)(check_car_d / LANE_WIDTH);

    // ignoring vehicles on the other side of the road
    if ( check_car_d < 0 ) {
      continue;
    }

    // projecting the vehicle's position into the future
    double check_car_s_future = check_car_s + (double)prediction_horizon * TIME_SLOT * check_car_speed;

    // calculating vehicle distance (=that is, the difference along the S axis!) now and in the hypothetical future
    double distance_to_vehicle = s_difference(check_car_s, ego_s_now);
    double future_distance_to_vehicle = s_difference(check_car_s_future, ego_s_future);

    // is this vehicle in front of us or behind us (along the S axis)?
    bool check_car_is_ahead = distance_to_vehicle > 0;

    // checking whether this vehicles stays behind us OR in front of us in a safe distance
    bool distance_to_vehicle_is_safe =    (distance_to_vehicle < SAFE_DISTANCE_BEHIND  &&  future_distance_to_vehicle < SAFE_DISTANCE_BEHIND)
                                       || (distance_to_vehicle > proximity_gap  &&  future_distance_to_vehicle > proximity_gap);

    // assembling info register key and updating lange change feasibility or the necessity to change the lane
    std::string lane_key;
    if ( check_car_lane == ego_lane - 1 ) {
      lane_key = "left_";
      left_lane_change_feasible &= distance_to_vehicle_is_safe;

    } else if ( check_car_lane == ego_lane ) {
      lane_key = "center_";
      lane_change_needed |= check_car_is_ahead && future_distance_to_vehicle < proximity_gap;

    } else if ( check_car_lane == ego_lane + 1 ) {
      lane_key = "right_";
      right_lane_change_feasible &= distance_to_vehicle_is_safe;

    } else {
      // ignoring vehicle which is two lanes away from our lane
      continue;
    }

    // assembling keys, part 2: appending "ahead" and "behind"
    std::string ahead = lane_key;
    ahead.append("ahead");

    std::string behind = lane_key;
    behind.append("behind");

    // if ( check_car_is_ahead  &&  future_distance_to_vehicle < nearest_vehicles[ahead].future_distance ) {
    if ( check_car_is_ahead  &&  distance_to_vehicle < nearest_vehicles[ahead].distance ) {

      // we've found a new "nearest vehicle ahead" candidate in the given lane
      nearest_vehicles[ahead].id = sensor_fusion[i][0];
      nearest_vehicles[ahead].lane = check_car_lane;
      nearest_vehicles[ahead].vx = vx;
      nearest_vehicles[ahead].vy = vy;
      nearest_vehicles[ahead].speed = check_car_speed;
      nearest_vehicles[ahead].s = check_car_s;
      nearest_vehicles[ahead].d = check_car_d;
      nearest_vehicles[ahead].distance = distance_to_vehicle;
      nearest_vehicles[ahead].future_s = check_car_s_future;
      nearest_vehicles[ahead].future_d = check_car_d;
      nearest_vehicles[ahead].future_distance = future_distance_to_vehicle;

    // } else if ( !check_car_is_ahead  &&  future_distance_to_vehicle > nearest_vehicles[behind].future_distance ) {
    } else if ( !check_car_is_ahead  &&  distance_to_vehicle > nearest_vehicles[behind].distance ) {

      // we've found a new "nearest vehicle behind" candidate in the given lane
      nearest_vehicles[behind].id = sensor_fusion[i][0];
      nearest_vehicles[behind].lane = check_car_lane;
      nearest_vehicles[behind].vx = vx;
      nearest_vehicles[behind].vy = vy;
      nearest_vehicles[behind].speed = check_car_speed;
      nearest_vehicles[behind].s = check_car_s;
      nearest_vehicles[behind].d = check_car_d;
      nearest_vehicles[behind].distance = distance_to_vehicle;
      nearest_vehicles[behind].future_s = check_car_s_future;
      nearest_vehicles[behind].future_d = check_car_d;
      nearest_vehicles[behind].future_distance = future_distance_to_vehicle;
    }
  }

  // displaying some debug info if requested
  if ( debug ) {

    double left_ahead = (nearest_vehicles["left_ahead"].id > -1 ? nearest_vehicles["left_ahead"].distance : 999.0 );
    double left_behind = (nearest_vehicles["left_behind"].id > -1 ? nearest_vehicles["left_behind"].distance : -999.0 );
    double center_ahead = (nearest_vehicles["center_ahead"].id > -1 ? nearest_vehicles["center_ahead"].distance : 999.0 );
    double center_behind = (nearest_vehicles["center_behind"].id > -1 ? nearest_vehicles["center_behind"].distance : -999.0 );
    double right_ahead = (nearest_vehicles["right_ahead"].id > -1 ? nearest_vehicles["right_ahead"].distance : 999.0 );
    double right_behind = (nearest_vehicles["right_behind"].id > -1 ? nearest_vehicles["right_behind"].distance : -999.0 );

    double left_ahead_future = (nearest_vehicles["left_ahead"].id > -1 ? nearest_vehicles["left_ahead"].future_distance : 999.0 );
    double left_behind_future = (nearest_vehicles["left_behind"].id > -1 ? nearest_vehicles["left_behind"].future_distance : -999.0 );
    double center_ahead_future = (nearest_vehicles["center_ahead"].id > -1 ? nearest_vehicles["center_ahead"].future_distance : 999.0 );
    double center_behind_future = (nearest_vehicles["center_behind"].id > -1 ? nearest_vehicles["center_behindd"].future_distance : -999.0 );
    double right_ahead_future = (nearest_vehicles["right_ahead"].id > -1 ? nearest_vehicles["right_ahead"].future_distance : 999.0 );
    double right_behind_future = (nearest_vehicles["right_behind"].id > -1 ? nearest_vehicles["right_behind"].future_distance : -999.0 );

    char buffer[80];
    std::cout << std::endl <<
         std::endl <<
         std::endl << "s: " << ego_s_now <<
         std::endl << "--------------------  Current | future distances [m]  ----------------" <<
         std::endl << "              Left                  Center                Right" <<
         std::endl << "             ------                --------              -------" <<
         std::endl << "Status:  " << (left_lane_change_feasible ? "     safe      " : "### BLOCKED ###") <<
                      "        " << (lane_change_needed ? "### CHANGE ###" : "     keep     ") <<
                      "       " << (right_lane_change_feasible ? "     safe" : "### BLOCKED ###") << std::endl;

    sprintf(buffer, "Ahead:   %+6.1f | %+6.1f       %+6.1f | %+6.1f       %+6.1f | %+6.1f",
            left_ahead, left_ahead_future, center_ahead, center_ahead_future, right_ahead, right_ahead_future);
    std::cout << std::string(buffer) << std::endl;

    sprintf(buffer, "Behind:  %+6.1f | %+6.1f       %+6.1f | %+6.1f       %+6.1f | %+6.1f",
            left_behind, left_behind_future, center_behind, center_behind_future, right_behind, right_behind_future);
    std::cout << std::string(buffer) << std::endl;
    std::cout <<         "----------------------------------------------------------------------" << std::endl;
  }

  return nearest_vehicles;
}







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
                    bool debug ) {

  // buffer for debug messages
  std::stringstream message;


  // determining anticipated velocity values for the lanes (if that's possible)
  double left_lane_v = 0.0;
  if ( left_lane_change_feasible ) {

    // adjust lane speed to the nearest vehicle in case it's close enough
    if (    nearest_vehicles["left_ahead"].id > -1
         && s_difference(nearest_vehicles["left_ahead"].future_s, ego_s_future) < 2*proximity_gap ) {
      left_lane_v = nearest_vehicles["left_ahead"].speed;

    } else {
      // press the pedal to the metal
      left_lane_v = MAX_V;
    }
  }

  double right_lane_v = 0.0;
  if ( right_lane_change_feasible ) {

    // adjust lane speed to the nearest vehicle in case it's close enough
    if (    nearest_vehicles["right_ahead"].id > -1
         && s_difference(nearest_vehicles["right_ahead"].future_s, ego_s_future) < 2*proximity_gap ) {
      right_lane_v = nearest_vehicles["right_ahead"].speed;

    } else {
      // press the pedal to the metal
      right_lane_v = MAX_V;
    }
  }


  // so, are we approaching a vehicle in our lane within the foreseeable future?
  if ( lane_change_needed ) {

    // do the lane change -- if not right in the middle of changing it
    if ( !lane_change_in_progress  &&  left_lane_change_feasible  &&  right_lane_change_feasible ) {

      // we can pick a target lane; both of the are free
      if ( left_lane_v == right_lane_v ) {

        // the agony of choice
        if ( nearest_vehicles["left_ahead"].future_s >= nearest_vehicles["right_ahead"].future_s ) {
          ego_lane--;

        } else {
          // we could still roll the dice in case they're equally far away...
          ego_lane++;
        }

      } else if ( left_lane_v > right_lane_v ) {
        ego_lane--;

      } else {
        ego_lane++;
      }

    } else if ( !lane_change_in_progress  &&  left_lane_change_feasible ) {
      ego_lane--;

    } else if ( !lane_change_in_progress  &&  right_lane_change_feasible ) {
      ego_lane++;

    } else {
      // we can't leave the lane right now; we have to adjust our velocity to the car which is ahead of us
      ego_v -= MAX_V_CHANGE;
      ego_v = max(ego_v, nearest_vehicles["center_ahead"].speed);
      message << "Keeping lane and slowing down (" << ego_v;

      // decreasing the speed even more in case the vehicle is (or is going to be) too close
      if (    s_difference(nearest_vehicles["center_ahead"].future_s, ego_s_future) < proximity_gap
           || s_difference(nearest_vehicles["center_ahead"].s, ego_s_now) < proximity_gap ) {
        ego_v -= 0.5 * MAX_V_CHANGE;
        message << " --> " << ego_v;
      }

      message << ")  ";
    }


  } else {
    // just keep on driving in the current lane while trying to maximize velocity
    ego_v += MAX_V_CHANGE;
    ego_v = min(ego_v, MAX_V);

    message << "Accelerating to " << ego_v << " m/s. ";

    // basic strategy: trying to return to the center lane in case it's feasible
    if (    ego_lane == 0
         && right_lane_change_feasible
         && (nearest_vehicles["right_ahead"].id == -1  ||  s_difference(nearest_vehicles["right_ahead"].future_s, ego_s_future) > 2*proximity_gap) ) {
      ego_lane++;
      message << "Returning to the center lane. ";

    } else if (    ego_lane == 2
                && left_lane_change_feasible
                && (nearest_vehicles["left_ahead"].id == -1  ||  s_difference(nearest_vehicles["left_ahead"].future_s, ego_s_future) > 2*proximity_gap) ) {
      ego_lane--;
      message << "Returning to the center lane. ";
    }
  }

  // wanna hear some news?
  if ( debug ) {
    std::cout << message.str() << std::endl;
  }
}
