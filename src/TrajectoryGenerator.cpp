#include "TrajectoryGenerator.h"

TrajectoryGenerator::TrajectoryGenerator() {
}

TrajectoryGenerator::~TrajectoryGenerator() {
}

double TrajectoryGenerator::exceeds_speed_limit_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_d(i) + traj.second.eval_d(i) > _hard_max_vel_per_timestep)
      return 1.0;
  }
  return 0.0;
}

double TrajectoryGenerator::exceeds_accel_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_double_d(i) + traj.second.eval_double_d(i) > _hard_max_acc_per_timestep)
      return 1.0;
  }
  return 0.0;
}

double TrajectoryGenerator::exceeds_jerk_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.eval_triple_d(i) + traj.second.eval_triple_d(i) > _hard_max_jerk_per_timestep)
      return 1.0;
  }
  return 0.0;
}

double TrajectoryGenerator::collision_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  for (int t = 0; t < _horizon; t++) {
    for (int i = 0; i < vehicles.size(); i++) {
      double ego_s = traj.first.eval(t);
      double ego_d = traj.second.eval(t);
      vector<double> traffic_state = vehicles[i].state_at(t);

      double dif_s = abs(traffic_state[0] - ego_s);
      double dif_d = abs(traffic_state[1] - ego_d);

      // make the envelope a little wider to stay "out of trouble"
      if ((dif_s <= _car_col_length * 5.0) && (dif_d <= _car_col_width * 3.0))
        return 1.0;
    }
  }
  return 0.0;
}

// adds cost for getting too close to another vehicle
double TrajectoryGenerator::traffic_buffer_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  double cost = 0.0;
  for (int i = 0; i < vehicles.size(); i++) {
    for (int t = 0; t < _horizon; t++) {
      double ego_s = traj.first.eval(t);
      double ego_d = traj.second.eval(t);
      vector<double> traffic_state = vehicles[i].state_at(t); // {s,d}

    double dif_s = traffic_state[0] - ego_s;
    // Ignore (potentially faster) vehicles from behind or that have fallen behind
    // Tried it and ego moved out of their way, often hitting slower traffic. Not fun.
    if (dif_s < -10)
      break;
    dif_s = abs(dif_s);
    double dif_d = abs(traffic_state[1] - ego_d);

    // if in the same lane and too close
    if ((dif_s <= _col_buf_length) && (dif_d <= _col_buf_width))
      cost += logistic(1 - (dif_s / _col_buf_length)) / _horizon;
    }
  }
  return cost;
}

// penalizes low average speeds compared to speed limit
double TrajectoryGenerator::efficiency_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  double s_dist = goal[0] - traj.first.eval(0);
  double max_dist = _delta_s_maxspeed;
  return abs(logistic((max_dist - s_dist) / max_dist)); // abs() because going faster is actually bad
}

double TrajectoryGenerator::total_accel_s_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    cost += abs(traj.first.eval_double_d(t));
  }
  return logistic(cost);
}

double TrajectoryGenerator::total_accel_d_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    cost += abs(traj.second.eval_double_d(t));
  }
  return logistic(cost);
}

double TrajectoryGenerator::total_jerk_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    cost += abs(traj.first.eval_triple_d(t));
    cost += abs(traj.second.eval_triple_d(t));
  }
  return logistic(cost);
}

double TrajectoryGenerator::lane_depart_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    double ego_d = traj.second.eval(t);
    double lane_marking_proximity = fmod(ego_d, 4);
    if (lane_marking_proximity > 2.0)
      lane_marking_proximity = abs(lane_marking_proximity - 4);
    if (lane_marking_proximity <= _car_col_width) // car touches middle lane
      cost += 1 - logistic(lane_marking_proximity);
  }
  return cost;
}

// nudges vehicle to proactively depart lanes with traffic ahead and prevent changing into busy lanes
double TrajectoryGenerator::traffic_ahead_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles) {
  double ego_s = traj.first.eval(0);
  double ego_d = traj.second.eval(0);
  double ego_d_end = traj.second.eval(_horizon);
  int look_ahead = 200;

  int fut_lane_i = 0;
  if (ego_d_end > 8) fut_lane_i = 2;
  else if (ego_d_end > 4) fut_lane_i = 1;
  int closest_veh_fut_i = closest_vehicle_in_lane({ego_s, ego_d_end}, fut_lane_i, vehicles);
  // if there is a vehicle in the lane the trajectory will take us to
  if (closest_veh_fut_i != -1) {
    vector<double> fut_traffic_s = vehicles[closest_veh_fut_i].get_s();
    float dif_s = fut_traffic_s[0] - ego_s;
    if (dif_s < look_ahead) {
      // don't switch into a lane with slower traffic ahead
      int cur_lane_i = 0;
      if (ego_d > 8) cur_lane_i = 2;
      else if (ego_d > 4) cur_lane_i = 1;
      int closest_veh_i = closest_vehicle_in_lane({ego_s, ego_d_end}, cur_lane_i, vehicles);
      // if there is a vehicle in the current lane AND make range a bit tighter
      if ((closest_veh_i != -1) && (dif_s < look_ahead / 2.0)) {
        vector<double> traffic_s = vehicles[closest_veh_i].get_s();
        double ego_s_vel = traj.first.eval_d(0);
        // traffic in planned lane clearly slower than in current?
        if (fut_traffic_s[1] < traffic_s[1] * 0.95)
          return 1000;
      }
      // end - don't switch into a lane with slower traffic ahead

      // no slower traffic in goal lane, so return regular cost
      return logistic(1 - (dif_s / look_ahead));
    }
  }
  return 0.0;
}


double TrajectoryGenerator::calculate_cost(pair<Polynomial, Polynomial> const &traj, vector<double> const &goal, vector<Car> const &vehicles, vector<vector<double>> &all_costs) {
  Polynomial s = traj.first;
  Polynomial d = traj.second;

  double cost = 0.0;
  // first situations that immediately make a trajectory infeasible
  double ex_sp_lim_cost = exceeds_speed_limit_cost(traj, goal, vehicles);
  double ex_acc_lim_cost = exceeds_accel_cost(traj, goal, vehicles);
  double ex_jerk_lim_cost = exceeds_jerk_cost(traj, goal, vehicles);
  double col_cost = collision_cost(traj, goal, vehicles);

  double infeasible_costs = ex_sp_lim_cost + ex_acc_lim_cost + ex_jerk_lim_cost + col_cost;
  if (infeasible_costs > 0.0) {
    all_costs.push_back({999999});
    return 999999;
  }

  double tr_buf_cost   = traffic_buffer_cost(traj, goal, vehicles) * _cost_weights["tr_buf_cost"];
  double eff_cost      = efficiency_cost(traj, goal, vehicles) * _cost_weights["eff_cost"];
  double acc_s_cost    = total_accel_s_cost(traj, goal, vehicles) * _cost_weights["acc_s_cost"];
  double acc_d_cost    = total_accel_d_cost(traj, goal, vehicles) * _cost_weights["acc_d_cost"];
  double jerk_cost     = total_jerk_cost(traj, goal, vehicles) * _cost_weights["jerk_cost"];
  double lane_dep_cost = lane_depart_cost(traj, goal, vehicles) * _cost_weights["lane_dep_cost"];
  double traffic_cost  = traffic_ahead_cost(traj, goal, vehicles) * _cost_weights["traffic_cost"];

  vector<double> cost_vec = {tr_buf_cost, eff_cost, acc_s_cost, acc_d_cost, jerk_cost, lane_dep_cost, traffic_cost};
  all_costs.push_back(cost_vec);

  cost = tr_buf_cost + eff_cost + acc_s_cost + acc_d_cost + jerk_cost + lane_dep_cost + traffic_cost;
  return cost;
}

// returns a value between 0 and 1 for x in the range [0, infinity]
// and -1 to 1 for x in the range [-infinity, infinity].
// approaches 1 at an input of around 5
double TrajectoryGenerator::logistic(double x) {
  return (2.0 / (1 + exp(-x)) - 1.0);
}

// searches for closest vehicle in current travel lane. Returns index and s-distance.
int TrajectoryGenerator::closest_vehicle_in_lane(vector<double> const &start, int ego_lane_i, vector<Car> const &vehicles) {
  int closest_i = -1;
  float min_s_dif = 999;
  for (int i = 0; i < vehicles.size(); i++) {
    vector<double> traffic_d = vehicles[i].get_d();
    int traffic_lane_i = 0;
    if (traffic_d[0] > 8) traffic_lane_i = 2;
    else if (traffic_d[0] > 4) traffic_lane_i = 1;

    if (ego_lane_i == traffic_lane_i) {
      vector<double> traffic_s = vehicles[i].get_s();
      float dif_s = traffic_s[0] - start[0];
      if ((dif_s > 0.0) && (dif_s < min_s_dif)) {
        closest_i = i;
        min_s_dif = dif_s;
      }
    }
  }
  return closest_i;
}

// searches for closest vehicle in current travel lane. Returns index and s-distance.
vector<int> TrajectoryGenerator::closest_vehicle_in_lanes(vector<double> const &start, vector<Car> const &vehicles) {
  vector<int> closest_veh_i(3);
  for (int i = 0; i < 3; i++)
    closest_veh_i[i] = closest_vehicle_in_lane(start, i, vehicles);
  return closest_veh_i;
}

string TrajectoryGenerator::get_current_action() {
  return _current_action;
}

// returns: trajectory for given number of timesteps (horizon) in Frenet coordinates
vector<vector<double>> TrajectoryGenerator::generate_trajectory(vector<double> const &start, double max_speed, double horizon, vector<Car> const &vehicles) {
  const vector<double> start_s = {start[0], start[1], start[2]};
  const vector<double> start_d = {start[3], start[4], start[5]};
  _horizon = horizon;
  _max_dist_per_timestep = 0.00894 * max_speed;

  _delta_s_maxspeed = _horizon * _max_dist_per_timestep;
  // rough way to make the car accelerate more smoothly from a complete stop
  if (start_s[1] < _max_dist_per_timestep / 1.75) {
    double vel_dif = _max_dist_per_timestep - start_s[1];
    _max_dist_per_timestep -= vel_dif * 0.6;
    _delta_s_maxspeed = _horizon * _max_dist_per_timestep;
  }

  // figure out current lane
  // 0: left, 1: middle, 2: right
  int cur_lane_i = 0;
  if (start_d[0] > 8) cur_lane_i = 2;
  else if (start_d[0] > 4) cur_lane_i = 1;

  cout << "ego local s: " << start_s[0] << " s_vel: " << start_s[1] << " d: " << start_d[0] << endl;

  vector<vector<double>> goal_points;
  vector<vector<double>> traj_goals; // s, s_dot, s_double_dot, d, d_dot, d_double_dot
  vector<double> traj_costs;

  // #########################################
  // FIND FEASIBLE NEXT STATES FROM:
  // - go straight
  // - go straight following leading vehicle
  // - lane change left
  // - lane change right
  // #########################################
  bool go_straight = true;
  bool go_straight_follow_lead = false;
  bool change_left = false;
  bool change_right = false;
  // get closest vehicle for each lane
  vector<int> closest_veh_i = closest_vehicle_in_lanes(start, vehicles);

  cout << "closest veh i " << closest_veh_i[cur_lane_i] << " - position s: " << vehicles[closest_veh_i[cur_lane_i]].get_s()[0] << " - position d: " << vehicles[closest_veh_i[cur_lane_i]].get_d()[0] << endl;

  if (closest_veh_i[cur_lane_i] != -1) {
    vector<double> closest_veh_s = vehicles[closest_veh_i[cur_lane_i]].get_s();
    // there is some traffic ahead
    if (abs(closest_veh_s[0] - start_s[0]) < 100) {
      change_left = true;
      change_right = true;
    }
    // there is a vehicle close ahead
    if (abs(closest_veh_s[0] - start_s[0]) < _col_buf_length) {
      go_straight = false;
      go_straight_follow_lead = true;
      change_left = true;
      change_right = true;
    }
  }

  // special case:
  // - vehicle is in inner/outer lane with traffic, middle lane has similar traffic
  //   but opposite lane is open. Need to nudge it towards middle lane which
  //   which is a transition it would otherwise not make
  bool prefer_mid_lane = false;
  if ((cur_lane_i == 0) || (cur_lane_i == 2)) {
    vector<double> closest_veh_s_curLane = vehicles[closest_veh_i[cur_lane_i]].get_s();
    vector<double> closest_veh_s_midLane = vehicles[closest_veh_i[1]].get_s();
    vector<double> closest_veh_s_opLane = vehicles[closest_veh_i[abs(cur_lane_i - 2)]].get_s();
    // traffic in current lane
    if (abs(closest_veh_s_curLane[0] - start_s[0]) <  2 * _col_buf_length) {
      // traffic in middle lane
      if (abs(closest_veh_s_midLane[0] - start_s[0]) <  2 * _col_buf_length) {
        // no traffic on opposite lane
        if (abs(closest_veh_s_opLane[0] - start_s[0]) >  4 * _col_buf_length) {
          prefer_mid_lane = true;
          if (cur_lane_i == 0)
            change_right = true;
          else
            change_left = true;
        }
      }
    }
  }

  // #########################################
  // GENERATE GOALPOINTS
  // #########################################
  // GO STRAIGHT
  if (go_straight) {
    double goal_s_pos = start_s[0] + _delta_s_maxspeed;
    double goal_s_vel = _max_dist_per_timestep;
    double goal_s_acc = 0.0;
    double goal_d_pos = 2 + 4 * cur_lane_i;
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    vector<vector<double>> goal_points_straight = {goal_vec};
    perturb_goal(goal_vec, goal_points_straight);
    // add to goal points
    goal_points.reserve(goal_points.size() + goal_points_straight.size());
    goal_points.insert(goal_points.end(),goal_points_straight.begin(),goal_points_straight.end());
  }

  // FOLLOW OTHER VEHICLE
  if (go_straight_follow_lead) {
    vector<double> lead_s = vehicles[closest_veh_i[cur_lane_i]].get_s();

    // "EMERGENCY BREAK ASSIST" to the drive assist
    // if much slower vehicle pulls into lane dangerously close in front of us,
    if (((lead_s[0] - start_s[0]) < _col_buf_length * 0.5) && (lead_s[1] < start_s[1] * 0.8)) {
      cout << "EMERGENCY" << endl;
      // reducing horizon to reduce speed faster
      _current_action = "emergency";
      _horizon = 120;
      // and hold lane - getting forced into other lane in a small horizon will exceed force limits
      change_left = false;
      change_right = false;
    }
    double goal_s_pos = start_s[0] + lead_s[1] * _horizon;
    double goal_s_vel = lead_s[1];
    double goal_s_acc = 0.0;
    double goal_d_pos = 2 + 4 * cur_lane_i;
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    vector<vector<double>> goal_points_follow = {goal_vec};
    perturb_goal(goal_vec, goal_points_follow);
    // add to goal points
    goal_points.reserve(goal_points.size() + goal_points_follow.size());
    goal_points.insert(goal_points.end(),goal_points_follow.begin(),goal_points_follow.end());
  }

  // CHANGE LANE LEFT
  if (change_left && (cur_lane_i != 0)) {
    double goal_s_pos = start_s[0] + _delta_s_maxspeed;
    double goal_s_vel = _max_dist_per_timestep;
    // less aggressive lane change if we are already following
    if (go_straight_follow_lead) {
      vector<double> lead_s = vehicles[closest_veh_i[cur_lane_i]].get_s();
      // but only if following closely
      if (lead_s[0] - start_s[0] < _col_buf_length * 0.5) {
        goal_s_pos = start_s[0] + lead_s[1] * _horizon;
        goal_s_vel = lead_s[1];
      }
    }
    double goal_s_acc = 0.0;
    double goal_d_pos = (2 + 4 * cur_lane_i) - 4;
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    vector<vector<double>> goal_points_left = {goal_vec};
    perturb_goal(goal_vec, goal_points_left, true);
    // add to goal points
    goal_points.reserve(goal_points.size() + goal_points_left.size());
    goal_points.insert(goal_points.end(),goal_points_left.begin(),goal_points_left.end());
  }

  // CHANGE LANE RIGHT
  if (change_right && (cur_lane_i != 2)) {
    double goal_s_pos = start_s[0] + _delta_s_maxspeed;
    double goal_s_vel = _max_dist_per_timestep;
    // less aggressive lane change if we are already following
    if (go_straight_follow_lead) {
      vector<double> lead_s = vehicles[closest_veh_i[cur_lane_i]].get_s();
      // but only if following closely
      if (lead_s[0] - start_s[0] < _col_buf_length * 0.5) {
        goal_s_pos = start_s[0] + lead_s[1] * _horizon;
        goal_s_vel = lead_s[1];
      }
    }
    double goal_s_acc = 0.0;
    double goal_d_pos = (2 + 4 * cur_lane_i) + 4;
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    vector<double> goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    vector<vector<double>> goal_points_right = {goal_vec};
    perturb_goal(goal_vec, goal_points_right);
    // add to goal points
    goal_points.reserve(goal_points.size() + goal_points_right.size());
    goal_points.insert(goal_points.end(),goal_points_right.begin(),goal_points_right.end());
  }
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // END - GENERATE GOALPOINTS
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  cout << "PLAN: ";
  if (go_straight)
    cout << " :GO STRAIGHT: ";
  if (go_straight_follow_lead)
    cout << " :FOLLOW LEAD: ";
  if (change_left)
    cout << " :CHANGE LEFT: ";
  if (change_right)
    cout << " :CHANGE RIGHT: ";
  cout << endl;

  // #########################################
  // JERK MINIMIZED TRAJECTORIES
  // #########################################
  vector<pair<Polynomial, Polynomial>> trajectory_coefficients;
  for (vector<double> goal : goal_points) {
    vector<double> goal_s = {goal[0], goal[1], goal[2]};
    vector<double> goal_d = {goal[3], goal[4], goal[5]};
    // ignore goal points that are out of bounds
    if ((goal[3] > 1.0) && (goal[3] < 11.0)) {
      Polynomial traj_s_poly = jmt(start_s, goal_s, _horizon);
      Polynomial traj_d_poly = jmt(start_d, goal_d, _horizon);
      trajectory_coefficients.push_back(std::make_pair(traj_s_poly, traj_d_poly));
      traj_goals.push_back({goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]});
    }
  }
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // END - JERK MINIMIZED TRAJECTORIES
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // ################################
  // COMPUTE COST FOR EACH TRAJECTORY
  // ################################
  vector<vector<double>> all_costs;
  for (int i = 0; i < trajectory_coefficients.size(); i++) {
    double cost = calculate_cost(trajectory_coefficients[i], traj_goals[i], vehicles, all_costs);
    // if appropriate, scale costs for trajectories going to the middle lane
    if (prefer_mid_lane && (abs(6 - trajectory_coefficients[i].second.eval(_horizon)) < 1.0))
      cost *= 0.5;
    traj_costs.push_back(cost);
  }

  // choose least-cost trajectory
  double min_cost = traj_costs[0];
  int min_cost_i = 0;
  for (int i = 1; i < trajectory_coefficients.size(); i++) {
    if (traj_costs[i] < min_cost) {
      min_cost = traj_costs[i];
      min_cost_i = i;
    }
  }
  // rare edge case: vehicle is stuck in infeasible trajectory (usually stuck close behind other car)
  if (min_cost == 999999) {
    double min_s = trajectory_coefficients[0].first.eval(_horizon);
    int min_s_i = 0;
    // find trajectory going straight with minimum s
    for (int i = 1; i < _goal_perturb_samples; i++) {
      if (trajectory_coefficients[i].first.eval(_horizon) < min_s){
        min_s = trajectory_coefficients[i].first.eval(_horizon);
        min_s_i = i;
      }
    }
    min_cost_i = min_s_i;
  }

  _current_action = "straight";
  if (min_cost_i > _goal_perturb_samples)
    _current_action = "lane_change";

  cout << "cost: " << traj_costs[min_cost_i] << " - i: " << min_cost_i << endl;
  cout << "traffic buffer cost: " << all_costs[min_cost_i][0] << endl;
  cout << "efficiency cost: " << all_costs[min_cost_i][1] << endl;
  cout << "acceleration s cost: " << all_costs[min_cost_i][2] << endl;
  cout << "acceleration d cost: " << all_costs[min_cost_i][3] << endl;
  cout << "jerk cost: " << all_costs[min_cost_i][4] << endl;
  cout << "lane depart cost: " << all_costs[min_cost_i][5] << endl;
  cout << "traffic cost: " << all_costs[min_cost_i][6] << endl;
  cout << "lowest cost traj goal s/d: " << goal_points[min_cost_i][0] << " : " << goal_points[min_cost_i][3] << endl;
  // ################################
  // COMPUTE VALUES FOR TIME HORIZON
  // ################################
  vector<double> traj_s(_horizon);
  vector<double> traj_d(_horizon);
  for(int t = 0; t < _horizon; t++) {
    traj_s[t] = trajectory_coefficients[min_cost_i].first.eval(t);
    traj_d[t] = trajectory_coefficients[min_cost_i].second.eval(t);
  }

  vector<vector<double>> new_traj(2);
  new_traj[0] = traj_s;
  new_traj[1] = traj_d;

  return new_traj;
}


// creates randomly generated variations of goal point
void TrajectoryGenerator::perturb_goal(vector<double> goal, vector<vector<double>> &goal_points, bool no_ahead) {
  double percentage_std_deviation = 0.1;
  std::normal_distribution<double> distribution_10_percent(0.0, percentage_std_deviation);
  vector<double> pert_goal(6);
  for (int i = 0; i < _goal_perturb_samples; i++) {
    double multiplier = distribution_10_percent(_rand_generator);
    if (no_ahead && (multiplier > 0.0))
      multiplier *= -1.0;
    pert_goal.at(0) = goal[0] + (_delta_s_maxspeed * multiplier);
    pert_goal.at(1) = goal[1] + (_max_dist_per_timestep * multiplier);
    pert_goal.at(2) = 0.0;

    multiplier = distribution_10_percent(_rand_generator);
    pert_goal.at(3) = goal[3] + multiplier;
    pert_goal.at(4) = 0.0;
    pert_goal.at(5) = 0.0;
    goal_points.push_back(pert_goal);
  }
}


Polynomial TrajectoryGenerator::jmt(vector<double> const &start, vector<double> const &goal, int t) {
  double T = double(t);
  double t_2 = pow(T, 2);
  double t_3 = pow(T, 3);
  double t_4 = pow(T, 4);
  double t_5 = pow(T, 5);
  Eigen::Matrix3d A;
  A << t_3,   t_4,    t_5,
    3*t_2, 4*t_3,  5*t_4,
    6*t,   12*t_2, 20*t_3;

  double b_0 = start[0] + start[1] * t + 0.5 * start[2] * t_2;
  double b_1 = start[1] + start[2] * t;
  double b_2 = start[2];
  Eigen::MatrixXd b(3,1);
  b << goal[0] - b_0, goal[1] - b_1, goal[2] - b_2;

  Eigen::MatrixXd c = A.inverse() * b;
  //Eigen::Vector3d c = A.colPivHouseholderQr().solve(b);
  vector<double> coeff = {start[0], start[1], 0.5*start[2], c.data()[0], c.data()[1], c.data()[2]};
  Polynomial result(coeff);
  return result;
}
