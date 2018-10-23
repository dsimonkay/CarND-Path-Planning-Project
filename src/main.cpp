#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "parameters.h"
#include <map>
#include <string>
#include <limits>

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {

  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");

  if (found_null != string::npos) {
    return "";

  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }

  return "";
}


double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++) {

    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen) {

      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4) {

    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {

  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

struct VehicleInfo {
  int id;
  int idx;
  double vx;
  double vy;
  double speed;
  double s;
  double d;
  double distance;
  int lane;
  double future_s;
  double future_d;
  double future_distance;
};


int main() {

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

  // initial ego vehicle lane
  int lane = 1;
  double reference_v = 0.0;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
               &lane, &reference_v](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

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
          // Data element format: [id, x, y, vx, vy, s, d]
          auto sensor_fusion = j[1]["sensor_fusion"];

          // the resulting path plan
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int prev_size = previous_path_x.size();

          // determining the best known future position of the ego vehicle
          double car_s_now = car_s;
          double car_s_future = prev_size > 0 ? end_path_s : car_s;

          bool proximity_warning = false;
          bool lane_change_needed = false;
          bool slow_down = false; 

          // we're pretty optimistic
          double target_v = MAX_V;

          // adaptive proximity gap
          double proximity_gap = 1.2 * PROXIMITY_FRONTIER * reference_v / MAX_V;
          proximity_gap = max(proximity_gap, MIN_GAP);

          // Collecting information about the vehicles that are the closest to our position (along the S axis) --
          // also in the current as well as in all neighboring lanes.
          map<string, VehicleInfo> nearest_vehicles;
          nearest_vehicles["left_ahead"] = {-1, -1, 0.0, 0.0, 0.0, 0.0, 0.0, numeric_limits<double>::max(), 0, 0.0, 0.0, numeric_limits<double>::max()};
          nearest_vehicles["left_behind"] = {-1, -1, 0.0, 0.0, 0.0, 0.0, 0.0, numeric_limits<double>::min(), 0, 0.0, 0.0, numeric_limits<double>::min()};
          nearest_vehicles["center_ahead"] = {-1, -1, 0.0, 0.0, 0.0, 0.0, 0.0, numeric_limits<double>::max(), 0, 0.0, 0.0, numeric_limits<double>::max()};
          nearest_vehicles["center_behind"] = {-1, -1, 0.0, 0.0, 0.0, 0.0, 0.0, numeric_limits<double>::min(), 0, 0.0, 0.0, numeric_limits<double>::min()};
          nearest_vehicles["right_ahead"] = {-1, -1, 0.0, 0.0, 0.0, 0.0, 0.0, numeric_limits<double>::max(), 0, 0.0, 0.0, numeric_limits<double>::max()};
          nearest_vehicles["right_behind"] = {-1, -1, 0.0, 0.0, 0.0, 0.0, 0.0, numeric_limits<double>::min(), 0, 0.0, 0.0, numeric_limits<double>::min()};

          // Processing sensor fusion data
          for ( int i = 0;  i < sensor_fusion.size(); i++ ) {

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_car_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = (double)(sensor_fusion[i][5]);
            double check_car_d = (double)(sensor_fusion[i][6]);
            double check_car_lane = (int)(check_car_d / LANE_WIDTH);

            // ignoring the vehicles on the other side of the road
            if ( check_car_d < 0 ) {
              continue;
            }

            // correcting the anomaly that might occur due the closed nature of this very highway
            // check_car_s += (check_car_s < car_s_now ? MAX_S : 0);

            // is this vehicle in frond of us or behind us (along the S axis)?
            bool check_car_is_ahead = check_car_s > car_s_now;

            // projecting the vehicle's position into the future
            double check_car_s_future = check_car_s + (double)prev_size * TIME_INTERVAL * check_car_speed;

            // assembling info register key: lane part
            string lane_key;
            if ( check_car_lane == lane - 1 ) {
              lane_key = "left_";

            } else if ( check_car_lane == lane ) {
              lane_key = "center_";

            } else if ( check_car_lane == lane + 1 ) {
              lane_key = "right_";

            } else {
              // we're not interested in a vehicle which is two lanes away from our lane
              continue;
            }

            // assembling keys, part 2: appending "ahead" and "behind"
            string ahead = lane_key;
            ahead.append("ahead");

            string behind = lane_key;
            behind.append("behind");

            // calculating vehicle distance now and in the predicted future
            double distance_to_vehicle = (double)(sensor_fusion[i][5]) - car_s_now;
            double future_distance_to_vehicle = check_car_s_future - car_s_future;

            // if ( check_car_is_ahead  &&  future_distance_to_vehicle < nearest_vehicles[ahead].future_distance ) {
            if ( check_car_is_ahead  &&  distance_to_vehicle < nearest_vehicles[ahead].distance ) {

              // we've found a new "nearest vehicle ahead" candidate in the given lane
              nearest_vehicles[ahead].id = sensor_fusion[i][0];
              nearest_vehicles[ahead].idx = i;
              nearest_vehicles[ahead].vx = vx;
              nearest_vehicles[ahead].vy = vy;
              nearest_vehicles[ahead].speed = check_car_speed;
              nearest_vehicles[ahead].s = check_car_s;
              nearest_vehicles[ahead].d = check_car_d;
              nearest_vehicles[ahead].distance = distance_to_vehicle;
              nearest_vehicles[ahead].lane = check_car_lane;
              nearest_vehicles[ahead].future_s = check_car_s_future;
              nearest_vehicles[ahead].future_d = check_car_d;
              nearest_vehicles[ahead].future_distance = future_distance_to_vehicle;

            // } else if ( !check_car_is_ahead  &&  future_distance_to_vehicle > nearest_vehicles[behind].future_distance ) {
            } else if ( !check_car_is_ahead  &&  distance_to_vehicle > nearest_vehicles[behind].distance ) {

              // we've found a new "nearest vehicle behind" candidate in the given lane
              nearest_vehicles[behind].id = sensor_fusion[i][0];
              nearest_vehicles[behind].idx = i;
              nearest_vehicles[behind].vx = vx;
              nearest_vehicles[behind].vy = vy;
              nearest_vehicles[behind].speed = check_car_speed;
              nearest_vehicles[behind].s = check_car_s;
              nearest_vehicles[behind].d = check_car_d;
              nearest_vehicles[behind].distance = distance_to_vehicle;
              nearest_vehicles[behind].lane = check_car_lane;
              nearest_vehicles[behind].future_s = check_car_s_future;
              nearest_vehicles[behind].future_d = check_car_d;
              nearest_vehicles[behind].future_distance = future_distance_to_vehicle;
            }

            // // check whether this car's distance will be within a certain value in the future
            // if ( check_car_s_future > car_s_future  &&  (check_car_s_future - car_s_future) < PROXIMITY_FRONTIER) {

            //   // TODO: do some logic here, lower reference velocity so we don't crash into the
            //   // car in front of us.
            //   // NOTE: could also flag to try to change lanes
            //   proximity_warning = true;
            //   lane_change_needed = true;
            //   // if ( lane > 0 ) {
            //   //   lane = 0;
            //   // }
            // }
          }

          if ( nearest_vehicles["center_ahead"].id > -1 ) {
            cout << "Distance to the closest car (" << nearest_vehicles["center_ahead"].id << ") in the future: " << nearest_vehicles["center_ahead"].future_distance << " meters (actual: " << nearest_vehicles["center_ahead"].distance << " meters)" << endl;

          } else {
            cout << "No car ahead." << endl;
          }

          // now that we have all required information at our hands...
          if ( nearest_vehicles["center_ahead"].id > -1  &&  nearest_vehicles["center_ahead"].future_distance < proximity_gap ) {

            cout << "LANE CHANGE NEEDED. Distance to the closest car (" << nearest_vehicles["center_ahead"].id << ") in the future: " << nearest_vehicles["center_ahead"].future_distance << " meters (actual: " << nearest_vehicles["center_ahead"].distance << " meters)" << endl;
            lane_change_needed = true;
          }


          if ( lane_change_needed ) {

            cout << "car_s_now: " << car_s_now << endl;

            // trying to change the lane: find whether there's enough gap for a merge
            bool left_lane_change_feasible = lane > 0;
            left_lane_change_feasible &= nearest_vehicles["left_ahead"].id == -1 || nearest_vehicles["left_ahead"].future_s > nearest_vehicles["center_ahead"].future_s + VEHICLE_LENGTH;
            left_lane_change_feasible &= nearest_vehicles["left_behind"].id == -1 || nearest_vehicles["left_behind"].future_s < nearest_vehicles["center_ahead"].future_s - proximity_gap - VEHICLE_LENGTH;
            left_lane_change_feasible &= nearest_vehicles["left_behind"].id == -1 || nearest_vehicles["left_behind"].s < car_s_now - VEHICLE_LENGTH;

            double left_lane_v = MAX_V;
            if ( nearest_vehicles["left_ahead"].future_s < nearest_vehicles["center_ahead"].future_s + 2*proximity_gap ) {
              left_lane_v = nearest_vehicles["left_ahead"].speed;
            }

            bool right_lane_change_feasible = lane < 2;
            right_lane_change_feasible &= nearest_vehicles["right_ahead"].id == -1 || nearest_vehicles["right_ahead"].future_s > nearest_vehicles["center_ahead"].future_s + VEHICLE_LENGTH;
            right_lane_change_feasible &= nearest_vehicles["right_behind"].id == -1 || nearest_vehicles["right_behind"].future_s < nearest_vehicles["center_ahead"].future_s - proximity_gap - VEHICLE_LENGTH;
            right_lane_change_feasible &= nearest_vehicles["right_behind"].id == -1 || nearest_vehicles["right_behind"].s < car_s_now - VEHICLE_LENGTH;

            double right_lane_v = MAX_V;
            if ( nearest_vehicles["right_ahead"].future_s < nearest_vehicles["center_ahead"].future_s + 2*proximity_gap ) {
              right_lane_v = nearest_vehicles["right_ahead"].speed;
            }

            if ( left_lane_change_feasible ) {
              cout << "\tLeft change feasible." << endl;
              cout << "\t\tLeft ahead now/future: " << nearest_vehicles["left_ahead"].s << " / " << nearest_vehicles["left_ahead"].future_s;
              cout << "  s distance now/future: " << nearest_vehicles["left_ahead"].distance << " / " << nearest_vehicles["left_ahead"].future_distance << endl;
              cout << "\t\tLeft behind now/future: " << nearest_vehicles["left_behind"].s << " / " << nearest_vehicles["left_behind"].future_s;
              cout << "  s distance now/future: " << nearest_vehicles["left_behind"].distance << " / " << nearest_vehicles["left_behind"].future_distance << endl;
            }

            if ( right_lane_change_feasible ) {
              cout << "\tRight change feasible." << endl;
              cout << "\t\tright ahead now/future: " << nearest_vehicles["right_ahead"].s << " / " << nearest_vehicles["right_ahead"].future_s;
              cout << "  s distance now/future: " << nearest_vehicles["right_ahead"].distance << " / " << nearest_vehicles["right_ahead"].future_distance << endl;
              cout << "\t\tright behind now/future: " << nearest_vehicles["right_behind"].s << " / " << nearest_vehicles["right_behind"].future_s;
              cout << "  s distance now/future: " << nearest_vehicles["right_behind"].distance << " / " << nearest_vehicles["right_behind"].future_distance << endl;
            }


            // TODO: caculate trajectory costs for left and/or right lane change

            // do the lane change
            if ( left_lane_change_feasible  &&  left_lane_v > right_lane_v ) {
              lane--;

            } else if ( right_lane_change_feasible  &&  right_lane_v > left_lane_v ) {
              lane++;

            } else if ( left_lane_change_feasible ) {
              lane--;

            } else if ( right_lane_change_feasible ) {
              lane++;

            } else {
              // we can't leave the lane right now; we have to adjust our velocity to the car
              // which is ahead of us
              cout << "Slowing down to " << nearest_vehicles["center_ahead"].speed << " m/s" << endl;
              slow_down = true;
              reference_v -= MAX_V_CHANGE;
              reference_v = max(reference_v, nearest_vehicles["center_ahead"].speed);

              // decreasing the speed even more in case the vehicle is (or will be) too close
              if ( nearest_vehicles["center_ahead"].future_s - car_s_future < PROXIMITY_FRONTIER || nearest_vehicles["center_ahead"].s - car_s_now < PROXIMITY_FRONTIER ) {
                reference_v -= 0.5 * MAX_V_CHANGE;
              }
            }

          } else {
            // just keep on driving in the current lane while trying to maximize velocity
            cout << "Accelerating / keeping max. velocity (no lane change needed)." << endl;
            reference_v += MAX_V_CHANGE;
            reference_v = min(reference_v, MAX_V);

            // trying to occupy the center lane in case it's free
            if ( lane == 0 && (nearest_vehicles["right_ahead"].id == -1  ||  nearest_vehicles["right_ahead"].future_s - car_s_future > 3*proximity_gap) ) {
              lane++;

            } else if ( lane == 2 && (nearest_vehicles["leftt_ahead"].id == -1  ||  nearest_vehicles["left_ahead"].future_s - car_s_future > 3*proximity_gap) ) {
              lane--;
            }
          }


          // if ( proximity_warning ) {
          //   reference_v -= UNIT_CHANGE_V;

          // } else if ( reference_v < MAX_V ) {
          //   reference_v += UNIT_CHANGE_V;
          //   reference_v = min(reference_v, MAX_V);
          // }


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
          double speed_factor = 1.75 * reference_v / MAX_V;
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

          //   x_point += TIME_INTERVAL * segment_v;
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
          const double N = target_dist / (TIME_INTERVAL * reference_v);
          const double x_add_on_unit = target_x / N;
          double x_point = 0.0;

          // fill up the rest of our path planner after filling it previuos points.
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
