#ifndef CAR_H
#define CAR_H

#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

class Car {
  public:

    Car();
    Car(const Car& orig);

    virtual ~Vehicle();

    void set_frenet_pos(double pos_s, double pos_d);
    void set_frenet_motion(double vel_s, double acc_s, double vel_d, double acc_d);

    vector<double> get_s() const;
    vector<double> get_d() const;
    vector<double> state_at(double t) const;

    // format {s_vel, s_acc, d_vel, d_acc}
    vector<vector<double>> _future_states = {
      {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
      {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}
    };

  private:
    double _pos_s;
    double _pos_d;
    double _vel_s;
    double _vel_d;
    double _acc_s;
    double _acc_d;
};

#endif
