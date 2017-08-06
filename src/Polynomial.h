
#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

class Polynomial {
  public:
    Polynomial();
    Polynomial(const Polynomial& orig);
    Polynomial(vector<double> const &coefficients);
    virtual ~Polynomial();

    void set(vector<double> const &coefficients);
    double eval(double x) const;
    double eval_d(double x) const;
    double eval_double_d(double x) const;
    double eval_triple_d(double x) const;
    void print() const;


  private:
    vector<double> _coeff;
    vector<double> _coeff_d;
    vector<double> _coeff_double_d;
    vector<double> _coeff_triple_d;
};

#endif /* POLYNOMIAL_H */
