#include <chrono>
#include <iostream>
#include <fstream>
#include <map>
#include <math.h>
#include <string>
#include <sstream>
#include <thread>
#include <uWS/uWS.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "parameters.h"
#include "helper_functions_udacity.h"
#include "helper_functions.h"

using namespace std;

// for convenience
using json = nlohmann::json;



int main(int argc, char* argv[]) {

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;  // normal component x to the waypoint
  vector<double> map_waypoints_dy;  // normal component y to the waypoint

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {

    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // initial ego vehicle lane, current reference velocity and control variable for debug output
  int lane = 1;
  double reference_v = 0.0;
  double debug = false;

  // processing command line parameter[s] (okay, we're only prepared to react to "debug")
 for( int i = 1;  i < argc;  i++ ) {

    string param = string(argv[i]);
    string param_lower;
    transform(param.begin(), param.end(), back_inserter(param_lower), ::tolower);

    if ( param_lower == "-d" || param_lower == "--debug" ) {
      debug = true;
    }
  }


  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
               &lane, &reference_v, &debug](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // the waypoints of the planned path
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // number of the previous (inherited) waypoints
          int prev_size = previous_path_x.size();

          // saving the current S position and determining the best known future position of the ego vehicle
          double car_s_now = car_s;
          double car_s_future = prev_size > 0 ? end_path_s : car_s;

          // collecting information about the vehicles that are the closest to our position (along the S axis)
          map<string, VehicleInfo> nearest_vehicles = getNearestVehicles(sensor_fusion, lane,
                                                                         car_s_now, car_s_future,
                                                                         prev_size, debug);
          // adaptive proximity gap
          double proximity_gap = PROXIMITY_FRONTIER * reference_v / MAX_V;
          proximity_gap = max(proximity_gap, MIN_GAP);

          // buffer for debug messages
          stringstream message;

          // are we approaching a vehicle in our lane within the foreseeable future?
          if ( nearest_vehicles["center_ahead"].id > -1  &&  nearest_vehicles["center_ahead"].future_distance < proximity_gap ) {

            message << "Lane change needed. ";

            // trying to change the lane: determine whether there's enough gap for a merge
            bool left_lane_change_feasible = lane > 0;
            if ( nearest_vehicles["left_ahead"].id > -1 ) {

              left_lane_change_feasible &= nearest_vehicles["left_ahead"].s > car_s_now + 3*VEHICLE_LENGTH;
              left_lane_change_feasible &= nearest_vehicles["left_ahead"].future_s > car_s_future + 3*VEHICLE_LENGTH;
            }

            if ( nearest_vehicles["left_behind"].id > -1 ) {

              left_lane_change_feasible &= nearest_vehicles["left_behind"].s < car_s_now - 2*VEHICLE_LENGTH;
              left_lane_change_feasible &= nearest_vehicles["left_behind"].future_s < car_s_future - 2*VEHICLE_LENGTH;
            }

            double left_lane_v = MAX_V;
            if ( nearest_vehicles["left_ahead"].id > -1 && nearest_vehicles["left_ahead"].future_s < car_s_future + 2*proximity_gap ) {
              left_lane_v = nearest_vehicles["left_ahead"].speed;
            }

            bool right_lane_change_feasible = lane < 2;
            if ( nearest_vehicles["right_ahead"].id > -1 ) {

              right_lane_change_feasible &= nearest_vehicles["right_ahead"].s > car_s_now + 3*VEHICLE_LENGTH;
              right_lane_change_feasible &= nearest_vehicles["right_ahead"].future_s > car_s_future + 3*VEHICLE_LENGTH;
            }

            if ( nearest_vehicles["right_behind"].id > -1 ) {

              right_lane_change_feasible &= nearest_vehicles["right_behind"].s < car_s_now - 2*VEHICLE_LENGTH;
              right_lane_change_feasible &= nearest_vehicles["right_behind"].future_s < car_s_future - 2*VEHICLE_LENGTH;
            }

            double right_lane_v = MAX_V;
            if ( nearest_vehicles["right_ahead"].id > -1 && nearest_vehicles["right_ahead"].future_s < car_s_future + 2*proximity_gap ) {
              right_lane_v = nearest_vehicles["right_ahead"].speed;
            }

            if ( left_lane_change_feasible ) {
              message << "Left change feasible (lane speed: " << left_lane_v << ") ";
            }

            if ( right_lane_change_feasible ) {
              message << "Right change feasible (lane speed: " << right_lane_v << ") ";
            }


            // TODO: caculate trajectory costs for left and/or right lane change

            // do the lane change
            if ( left_lane_change_feasible  &&  left_lane_v > right_lane_v ) {
              message << "Changing left. ";
              lane--;

            } else if ( right_lane_change_feasible  &&  right_lane_v > left_lane_v ) {
              message << "Changing right. ";
              lane++;

            } else if ( left_lane_change_feasible ) {
              message << "Changing left. ";
              lane--;

            } else if ( right_lane_change_feasible ) {
              message << "Changing right. ";
              lane++;

            } else {
              // we can't leave the lane right now; we have to adjust our velocity to the car which is ahead of us
              reference_v -= MAX_V_CHANGE;
              reference_v = max(reference_v, nearest_vehicles["center_ahead"].speed);
              message << "Keeping lane and slowing down (" << reference_v << ") ";

              // decreasing the speed even more in case the vehicle is (or is going to be) too close
              if ( nearest_vehicles["center_ahead"].future_s - car_s_future < PROXIMITY_FRONTIER || nearest_vehicles["center_ahead"].s - car_s_now < PROXIMITY_FRONTIER ) {
                reference_v -= 0.5 * MAX_V_CHANGE;
                message << "!! (" << reference_v << ") ";
              }
            }

          } else {
            // just keep on driving in the current lane while trying to maximize velocity
            reference_v += MAX_V_CHANGE;
            reference_v = min(reference_v, MAX_V);

            message << "Accelerating to " << reference_v << " m/s. ";

            // trying to return to the center lane in case it's free
            if (    lane == 0
                 && (nearest_vehicles["right_ahead"].id == -1  ||  nearest_vehicles["right_ahead"].future_s > car_s_future + 3*proximity_gap)
                 && (nearest_vehicles["right_behind"].id == -1  ||  nearest_vehicles["right_behind"].future_s < car_s_future - 2*VEHICLE_LENGTH) ) {
              message << "Returning to the center lane. ";
              lane++;

            } else if (    lane == 2
                        && (nearest_vehicles["left_ahead"].id == -1  ||  nearest_vehicles["left_ahead"].future_s > car_s_future + 3*proximity_gap)
                        && (nearest_vehicles["left_behind"].id == -1  ||  nearest_vehicles["leftt_behind"].future_s < car_s_future - 2*VEHICLE_LENGTH) ) {
              message << "Returning to the center lane. ";
              lane--;
            }
          }

          // Displaying debug info
          if ( debug ) {
            cout << message.str() << endl;
          }

          // variables that will hold the X-Y coordinates of the reference points for the spline
          std::vector<double> pts_x;
          std::vector<double> pts_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          double ref_x_prev;
          double ref_y_prev;

          // if the previous path is almost empty, using the car's position as starting reference
          if ( prev_size < 2 ) {

            // using two points that make the path tangent to the car.
            // NOTE: cos() expects an angle in radians! -- thus the usage of ref_yaw instead of car_yaw
            ref_x_prev = car_x - cos(ref_yaw);
            ref_y_prev = car_y - sin(ref_yaw);

          } else { // using the previous path's end point as starting reference

            // redefine reference state as previous path's end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            ref_x_prev = previous_path_x[prev_size - 2];
            ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }

          // common operation: pushing two points onto the spline defining arrays
          // that make the path tangent to the car
          pts_x.push_back(ref_x_prev);
          pts_x.push_back(ref_x);

          pts_y.push_back(ref_y_prev);
          pts_y.push_back(ref_y);

          // adding evenly spaced points in Frenet ahead of the starting reference
          // and transforming them to global map coordinates right away
          double speed_factor = 1.7 * reference_v / MAX_V;
          speed_factor = max(speed_factor, 1.0);

          vector<double> next_wp0 = getXY(car_s + speed_factor * 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + speed_factor * 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + speed_factor * 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);

          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);

          // transforming the X-Y coordinates describing the spline to the vehicle's local coordinate system
          for( int i = 0;  i < pts_x.size();  i++ ) {

            // the map coordinates of the ith waypoint relative to the car (or reference point)
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;

            // transforming that point to local car coordinates
            pts_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            pts_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          // start with all of the previous path points from last time
          for ( int i = 0;  i < prev_size; i++ ) {

            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }



          // creating the spline
          tk::spline s;
          s.set_points(pts_x, pts_y);

          // // So we have a target velocity to reach (and we're gonna generate at least one point)
          // int path_segments_to_generate = max(1, PREDICTED_WAYPOINTS - prev_size);
          // double v_diff = target_v - end_path_v;
          // double segment_v_change = v_diff / path_segments_to_generate;

          // // velocity change must not exceed the threshold
          // if ( abs(segment_v_change) - MAX_V_CHANGE > 0.01 ) {
          //   segment_v_change = segment_v_change < 0 ? -MAX_V_CHANGE : MAX_V_CHANGE;
          // }

          // double x_point = 0.0;
          // double segment_v = end_path_v;
          // cout << "end path v: " << end_path_v;

          // // generating the rest of the points for the path planner
          // for ( int i = 0;  i < path_segments_to_generate;  i++ ) {

          //   // calculating the new velocity...
          //   segment_v += segment_v_change;

          //   // ...and applying constraints to it
          //   segment_v = min(target_v, segment_v);
          //   segment_v = min(MAX_V, segment_v);
          //   segment_v = max(0.0, segment_v);

          //   // storing the resulting value as the new "end-of-path" velocity
          //   end_path_v = segment_v;

          //   x_point += TIME_SLOT * segment_v;
          //   double y_point = s(x_point);

          //   // transforming back from local to global coordinate system
          //   double x_global = x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
          //   double y_global = x_point * sin(ref_yaw) + y_point * cos(ref_yaw);

          //   // we need global coordinates: shifting the point relative to the global map's origo
          //   x_global += ref_x;
          //   y_global += ref_y;

          //   next_x_vals.push_back(x_global);
          //   next_y_vals.push_back(y_global);
          // }

          // cout << " --> " << end_path_v << " in " << path_segments_to_generate << " steps" << endl;



          // calculate how to break up spline points so that we travel at our desired reference velocity...
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          // ...as presented in Aaron's 'visualisation'
          const double N = target_dist / (TIME_SLOT * reference_v);
          const double x_add_on_unit = target_x / N;
          double x_point = 0.0;

          // fill up the rest of our path planner after filling it with previuos points
          for ( int i = prev_size;  i < PREDICTED_WAYPOINTS;  i++ ) {

            x_point += x_add_on_unit;
            double y_point = s(x_point);

            // transforming back from local to global coordinate system
            double x_global = x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
            double y_global = x_point * sin(ref_yaw) + y_point * cos(ref_yaw);

            // we need global coordinates: shifting the point relative to the global map's origo
            x_global += ref_x;
            y_global += ref_y;

            next_x_vals.push_back(x_global);
            next_y_vals.push_back(y_global);
          }


          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  );


  // We don't need this since we're not using HTTP but if it's removed,
  // the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {

    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());

    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });


  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });


  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;

  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
