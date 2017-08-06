#ifndef POLYTRAJECTORYGENERATOR_H
#define POLYTRAJECTORYGENERATOR_H

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include <random>
#include "Polynomial.h"
#include "Car.h"

using namespace std;

class TrajectoryGenerator {
  public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();

    vector<vector<double>> generate_trajectory(vector<double> const &start, double max_speed, double horizon, vector<Vehicle> const &vehicles);
    Polynomial jmt(vector<double> const &start, vector<double> const &goal, int t);
    void perturb_goal(vector<double> goal, vector<vector<double>> &goal_points, bool no_ahead=false);
    double logistic(double x);
    int closest_vehicle_in_lane(vector<double> const &start, int ego_lane_i, vector<Vehicle> const &vehicles);
    vector<int> closest_vehicle_in_lanes(vector<double> const &start, vector<Vehicle> const &vehicles);
    double calculate_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles, vector<vector<double>> &all_costs);
    double exceeds_speed_limit_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double exceeds_accel_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double exceeds_jerk_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double collision_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double traffic_buffer_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double efficiency_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double total_accel_d_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double total_accel_s_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double total_jerk_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double lane_depart_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    double traffic_ahead_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Vehicle> const &vehicles);
    string get_current_action();

  private:
    std::string _current_action = "straight";
    const double _car_width = 2.0;
    const double _car_length = 5.0;
    const double _car_col_width = 0.5 * _car_width;
    const double _car_col_length = 0.5 * _car_length;
    const double _col_buf_width = _car_width;
    const double _col_buf_length = 6 * _car_length;
    const int _goal_perturb_samples = 10;
    int _horizon = 0;
    const double _hard_max_vel_per_timestep = 0.00894 * 49.5; // 50 mp/h and a little buffer
    const double _hard_max_acc_per_timestep = 10.0 / 50.0; // 10 m/s
    const double _hard_max_jerk_per_timestep = 10.0 / 50.0; // 10 m/s
    double _max_dist_per_timestep = 0.0;
    double _delta_s_maxspeed = 0.0;
    std::default_random_engine _rand_generator;
    std::map<std::string, double> _cost_weights = {
      {"tr_buf_cost", 140.0},
      {"eff_cost", 110.0},
      {"acc_s_cost", 10.0},
      {"acc_d_cost", 10.0},
      {"jerk_cost", 10.0},
      {"lane_dep_cost", 0.05},
      {"traffic_cost", 10.0},
    };
};

#endif /* TRAJECTORYGENERATOR_H */
