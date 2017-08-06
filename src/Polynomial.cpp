#include "Polynomial.h"

Polynomial::Polynomial() {
}

Polynomial::Polynomial(const Polynomial& orig) {
  _coeff = orig._coeff;
  _coeff_d = orig._coeff_d;
  _coeff_double_d = orig._coeff_double_d;
  _coeff_triple_d = orig._coeff_triple_d;
}

Polynomial::~Polynomial() {
}

Polynomial::Polynomial(vector<double> const &coefficients) {
  for (int i = 0; i < coefficients.size(); i++) {
    _coeff.push_back(coefficients[i]);
    if (i > 0) {
      double d = i * coefficients[i];
      _coeff_d.push_back(d);
      if (i > 1) {
        _coeff_double_d.push_back((i - 1) * d);
        if (i > 2) {
          _coeff_triple_d.push_back((i - 2) * d);
        }
      }
    }
  }
}

double Polynomial::eval(double x) const {
  double result = 0;
  for (int i = 0; i < _coeff.size(); i++) {
    result += _coeff[i] * pow(x, i);
  }
  return result;
}

double Polynomial::eval_d(double x) const {
  double result = 0;
  for (int i = 0; i < _coeff_d.size(); i++) {
    result += _coeff_d[i] * pow(x, i);
  }
  return result;
}

double Polynomial::eval_double_d(double x) const {
  double result = 0;
  for (int i = 0; i < _coeff_double_d.size(); i++) {
    result += _coeff_double_d[i] * pow(x, i);
  }
  return result;
}

double Polynomial::eval_triple_d(double x) const {
  double result = 0;
  for (int i = 0; i < _coeff_triple_d.size(); i++) {
    result += _coeff_triple_d[i] * pow(x, i);
  }
  return result;
}

void Polynomial::print() const {
  cout << "Polynomial Coefficients: "<< endl;
  for (double x : _coeff)
    cout << x << " :: ";
  cout << endl;
  cout << "Polynomial Derivative Coefficients:" << endl;
  for (double x : _coeff_d)
    cout << x << " :: ";
  cout << endl;
  cout << "Polynomial Double-D Coefficients:" << endl;
  for (double x : _coeff_double_d)
    cout << x << " :: ";
  cout << endl;
  cout << "Polynomial Triple-D Coefficients:" << endl;
  for (double x : _coeff_triple_d)
    cout << x << " :: ";
  cout << endl;
}
