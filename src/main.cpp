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

#include "TrajectoryGenerator.h"
#include "Car.h"
#include "spline.h"
#include <cassert>

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

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

// Transform from Frenet s,d coordinates to Cartesian x,y
// in particular, uses splines instead of an estimated angle to project out into d, making the results much smoother
vector<double> getXY_splines(double s, double d, tk::spline const &spline_fit_s_to_x, tk::spline const &spline_fit_s_to_y, tk::spline const &spline_fit_s_to_dx, tk::spline const &spline_fit_s_to_dy) {
  double x_mid_road = spline_fit_s_to_x(s);
  double y_mid_road = spline_fit_s_to_y(s);
  double dx = spline_fit_s_to_dx(s);
  double dy = spline_fit_s_to_dy(s);

  double x = x_mid_road + dx * d;
  double y = y_mid_road + dy * d;

  return {x, y};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> const &maps_s, vector<double> const &maps_x, vector<double> const &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
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


void fit_spline_segment(double car_s, vector<double> const &map_waypoints_s, vector<double> const &map_waypoints_x, vector<double> const &map_waypoints_y, vector<double> const &map_waypoints_dx, vector<double> const &map_waypoints_dy, vector<double> &waypoints_segment_s, vector<double> &waypoints_segment_s_worldSpace, tk::spline &spline_fit_s_to_x, tk::spline &spline_fit_s_to_y, tk::spline &spline_fit_s_to_dx, tk::spline &spline_fit_s_to_dy) {
  // get 10 previous and 20 next waypoints
  vector<double> waypoints_segment_x, waypoints_segment_y, waypoints_segment_dx, waypoints_segment_dy;
  vector<int> wp_indeces;
  const int lower_wp_i = 9;
  const int upper_wp_i = 20;
  int prev_wp = -1;
  while(car_s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
    prev_wp++;
  for (int i = lower_wp_i; i > 0; i--) {
    if (prev_wp - i < 0)
      wp_indeces.push_back(map_waypoints_s.size() + (prev_wp - i));
    else
      wp_indeces.push_back((prev_wp - i) % map_waypoints_s.size());
  }
  wp_indeces.push_back(prev_wp);
  for (int i = 1; i < upper_wp_i; i++)
    wp_indeces.push_back((prev_wp + i) % map_waypoints_s.size());

  // FILL NEW SEGMENT WAYPOINTS
  const double max_s = 6945.554;
  bool crossed_through_zero = false;
  double seg_start_s = map_waypoints_s[wp_indeces[0]];
  for (int i = 0; i < wp_indeces.size(); i++) {
    int cur_wp_i = wp_indeces[i];
    waypoints_segment_x.push_back(map_waypoints_x[cur_wp_i]);
    waypoints_segment_y.push_back(map_waypoints_y[cur_wp_i]);
    waypoints_segment_dx.push_back(map_waypoints_dx[cur_wp_i]);
    waypoints_segment_dy.push_back(map_waypoints_dy[cur_wp_i]);
    // need special treatment of segments that cross over the end/beginning of lap
    if (i > 0) {
      if (cur_wp_i < wp_indeces[i-1])
        crossed_through_zero = true;
    }
    waypoints_segment_s_worldSpace.push_back(map_waypoints_s[cur_wp_i]);
    if (crossed_through_zero)
      waypoints_segment_s.push_back(abs(seg_start_s - max_s) + map_waypoints_s[cur_wp_i]);
    else
      waypoints_segment_s.push_back(map_waypoints_s[cur_wp_i] - seg_start_s);
  }
  // fit splines
  spline_fit_s_to_x.set_points(waypoints_segment_s, waypoints_segment_x);
  spline_fit_s_to_y.set_points(waypoints_segment_s, waypoints_segment_y);
  spline_fit_s_to_dx.set_points(waypoints_segment_s, waypoints_segment_dx);
  spline_fit_s_to_dy.set_points(waypoints_segment_s, waypoints_segment_dy);
}


// converts world space s coordinate to local space based on provided mapping
double get_local_s(double world_s, vector<double> const &waypoints_segment_s_worldSpace, vector<double> const &waypoints_segment_s) {
  int prev_wp = 0;
  // special case: first wp in list is larger than s. Meaning we are crossing over 0 somewhere.
  // go to index with value zero first and search from there.
  if (waypoints_segment_s_worldSpace[0] > world_s) {
    while (waypoints_segment_s_worldSpace[prev_wp] != 0.0)
      prev_wp += 1;
  }
  while ((waypoints_segment_s_worldSpace[prev_wp+1] < world_s) && (waypoints_segment_s_worldSpace[prev_wp+1] != 0))
    prev_wp += 1;
  double diff_world = world_s - waypoints_segment_s_worldSpace[prev_wp];
  double s_local = waypoints_segment_s[prev_wp] + diff_world;
  return s_local;
}

int main() {
  uWS::Hub h;

  TrajectoryGenerator PTG;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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

  // Create object for ego vehicle;
  Car my_car;

  int horizon_global = 175; //200
  int horizon = horizon_global;
  int update_interval_global = 40; // update every second
  int update_interval = update_interval_global;
  double speed_limit_global = 47.5;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,
      &map_waypoints_dy,&PTG,&my_car,&horizon,&horizon_global,&update_interval_global,&update_interval,&speed_limit_global]
      (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      //auto sdata = string(data).substr(0, length);
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

      // update actual position
      my_car.set_frenet_pos(car_s, car_d);

      // Previous path data given to the Planner
      vector<double> previous_path_x = j[1]["previous_path_x"];
      vector<double> previous_path_y = j[1]["previous_path_y"];
      int prev_path_size = previous_path_x.size();

      // Previous path's end s and d values
      double end_path_s = j[1]["end_path_s"];
      double end_path_d = j[1]["end_path_d"];

      // Sensor Fusion Data, a list of all other cars on the same side of the road.
      auto sensor_fusion = j[1]["sensor_fusion"];

      json msgJson;

      vector<double> next_x_vals;
      vector<double> next_y_vals;

      double speed_limit = speed_limit_global;

      bool smooth_path = previous_path_x.size() > 0;

      if (previous_path_x.size() < horizon - update_interval) {

        // Extract surrounding waypoints and fit a spline
        vector<double> waypoints_segment_s;
        vector<double> waypoints_segment_s_worldSpace;

        tk::spline spline_fit_s_to_x;
        tk::spline spline_fit_s_to_y;
        tk::spline spline_fit_s_to_dx;
        tk::spline spline_fit_s_to_dy;

        fit_spline_segment(
            car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y,
            map_waypoints_dx, map_waypoints_dy, waypoints_segment_s,
            waypoints_segment_s_worldSpace, spline_fit_s_to_x,
            spline_fit_s_to_y, spline_fit_s_to_dx, spline_fit_s_to_dy);

        // Create local frenet space

        // Convert current car_s into our local Frenet space
        double car_local_s = get_local_s(car_s, waypoints_segment_s_worldSpace, waypoints_segment_s);

        // Convert sensor fusion data into local Frenet space
        for (int i = 0; i < sensor_fusion.size(); i++) {
          sensor_fusion[i][5] = get_local_s(sensor_fusion[i][5], waypoints_segment_s_worldSpace, waypoints_segment_s);
        }

        // Turn sensor fusion data into Car objects
        vector<Car> envir_vehicles(sensor_fusion.size());
        for (int i = 0; i < sensor_fusion.size(); i++) {
          envir_vehicles[i].set_frenet_pos(sensor_fusion[i][5], sensor_fusion[i][6]);
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];
          double velocity_per_timestep = sqrt(pow(vx, 2) + pow(vy, 2)) / 50.0;
          envir_vehicles[i].set_frenet_motion(velocity_per_timestep, 0.0, 0.0, 0.0);
        }

        // #################################################################
        // HACK: dealing with undesirably high speeds in tight left turns
        // Since path is planned in Frenet, going through curves increases the actual distance covered - and along with that velocity
        // #################################################################
        // Frenet space is sampled from road center, so effect is almost zero in left lane, and most amplified in right lane
        // figure out current lane
        // 0: left, 1: middle, 2: right
        int cur_lane_i = 0;
        if (car_d > 8) cur_lane_i = 2;
        else if (car_d > 4) cur_lane_i = 1;
        // sample three points along potential path
        double dx0 = spline_fit_s_to_dx(car_local_s);
        double dx1 = spline_fit_s_to_dx(car_local_s + 100);
        double dy0 = spline_fit_s_to_dy(car_local_s);
        double dy1 = spline_fit_s_to_dy(car_local_s + 100);

        double dx_dif = abs(dx0 - dx1);
        double dy_dif = abs(dy0 - dy1);

        if (dx_dif >= 0.1) {
          if (cur_lane_i == 2) { // right lane
            // worst: 0.1 => 25% speed reduction
            double scale_factor = 0.72 + (0.28 * (1 - (dx_dif - 0.04) * 0.5));
            speed_limit *= scale_factor;
          } else if (cur_lane_i == 1) { // center lane
            double scale_factor = 0.80 + (0.2 * (1 - (dx_dif - 0.04) * 0.5));
            speed_limit *= scale_factor;
          } else { // left lane
            double scale_factor = 0.92 + (0.08 * (1 - (dx_dif - 0.04) * 0.5));
            speed_limit *= scale_factor;
          }
        }
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // END HACK
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        // ###################################################
        // PLAN PATH
        // ###################################################
        // get current position from past path, taking into account lag
        int lag = horizon - update_interval - previous_path_x.size();
        if (lag > 10) lag = 0; // sim start
        // get last known car state
        //            vector<double> prev_car_s = my_car.get_s();
        //            vector<double> prev_car_d = my_car.get_d();
        // collect best guess at current car state. S position in local segment space
        cout << "lag: " << lag << endl;
        double est_car_s_vel = my_car._future_states[lag][0];
        double est_car_s_acc = my_car._future_states[lag][1];
        double est_car_d_vel = my_car._future_states[lag][2];
        double est_car_d_acc = my_car._future_states[lag][3];
        vector<double> car_state = {car_local_s, est_car_s_vel, est_car_s_acc, car_d, est_car_d_vel, est_car_d_acc};

        vector<vector<double>> new_path = PTG.generate_trajectory(car_state, speed_limit, horizon, envir_vehicles);
        update_interval = update_interval_global;
        horizon = horizon_global;
        if (PTG.get_current_action() == "lane_change") {
          cout << "LANE CHANGE" << endl;
          update_interval = horizon - 50;
        } else if (PTG.get_current_action() == "lane_change") {
          cout << "EMERGENCY" << endl;
          horizon = 120;
          update_interval = horizon - 80;
        }

        // ###################################################
        // store ego vehicle velocity and acceleration in s and d for next cycle
        // ###################################################
        // make a bold prediction into the future
        for (int i = 0; i < 10; i++) {
          double s0 = new_path[0][i + update_interval];
          double s1 = new_path[0][i + update_interval + 1];
          double s2 = new_path[0][i + update_interval + 2];
          double d0 = new_path[1][i + update_interval];
          double d1 = new_path[1][i + update_interval + 1];
          double d2 = new_path[1][i + update_interval + 2];
          double s_v1 = s1 - s0;
          double s_v2 = s2 - s1;
          double s_a = s_v2 - s_v1;
          double d_v1 = d1 - d0;
          double d_v2 = d2 - d1;
          double d_a = d_v2 - d_v1;
          my_car._future_states[i] = {s_v1, s_a, d_v1, d_a};
        }
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // END - store ego vehicle velocity and acceleration in s and d for next cycle
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        // ###################################################
        // ASSEMBLE SMOOTH NEW PATH
        // ###################################################
        double new_x, new_y;
        int smooth_range = 20;
        int reuse_prev_range = 15;
        // start with current car position in x/y
        //vector<double> prev_xy_planned = getXY(new_path[0][0], new_path[1][0], map_waypoints_s_upsampled, map_waypoints_x_upsampled, map_waypoints_y_upsampled);
        vector<double> prev_xy_planned;

        // reuse part of previous path, if applicable
        for(int i = 0; i < reuse_prev_range; i++) {
          prev_xy_planned = getXY_splines(new_path[0][i], new_path[1][i], spline_fit_s_to_x, spline_fit_s_to_y, spline_fit_s_to_dx, spline_fit_s_to_dy);
          if (smooth_path) {
            // re-use first point of previous path
            new_x = previous_path_x[i];
            new_y = previous_path_y[i];
            next_x_vals.push_back(new_x);
            next_y_vals.push_back(new_y);
          } else {
            next_x_vals.push_back(prev_xy_planned[0]);
            next_y_vals.push_back(prev_xy_planned[1]);
          }
        }

        // assemble rest of the path and smooth, if applicable
        for(int i = reuse_prev_range; i < new_path[0].size(); i++) {
          vector<double> xy_planned = getXY_splines(new_path[0][i], new_path[1][i], spline_fit_s_to_x, spline_fit_s_to_y, spline_fit_s_to_dx, spline_fit_s_to_dy);
          if (smooth_path) {
            double x_dif_planned =  xy_planned[0] - prev_xy_planned[0];
            double y_dif_planned =  xy_planned[1] - prev_xy_planned[1];
            new_x = new_x + x_dif_planned;
            new_y = new_y + y_dif_planned;

            double smooth_scale_fac = (smooth_range - (i - reuse_prev_range)) / smooth_range;
            if (i > smooth_range)
              smooth_scale_fac = 0.0;
            double smooth_x = (previous_path_x[i] * smooth_scale_fac) + (new_x * (1 - smooth_scale_fac));
            double smooth_y = (previous_path_y[i] * smooth_scale_fac) + (new_y * (1 - smooth_scale_fac));

            next_x_vals.push_back(smooth_x);
            next_y_vals.push_back(smooth_y);
            prev_xy_planned = xy_planned;

          } else {
            next_x_vals.push_back(xy_planned[0]);
            next_y_vals.push_back(xy_planned[1]);
          }
        }
      } else {
        for(int i = 0; i < previous_path_x.size(); i++) {
          next_x_vals.push_back(previous_path_x[i]);
          next_y_vals.push_back(previous_path_y[i]);
        }
      }
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // END - PATH PLANNING
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

      msgJson["next_x"] = next_x_vals;
      msgJson["next_y"] = next_y_vals;

      auto msg = "42[\"control\","+ msgJson.dump()+"]";

      //this_thread::sleep_for(chrono::milliseconds(1000));
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }

      }
      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
        size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char *message, size_t length) {
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
