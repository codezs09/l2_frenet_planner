#include "CubicPolynomial.h"

#include <Eigen/LU>
#include <cmath>

using namespace Eigen;

CubicPolynomial::CubicPolynomial(double xs, double vxs, double xe, double vxe,
                                 double t)
    : a0(xs), a1(vxs) {
  Matrix2d A;
  A << pow(t, 3), pow(t, 2), 3 * t * t, 2 * t;
  Vector2d B;
  B << xe - a0 - a1 * t, vxe - a1;
  Vector2d x = A.inverse() * B;
  a3 = x[0];
  a2 = x[1];
}

double CubicPolynomial::calc_point(double t) {
  return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3);
}

double CubicPolynomial::calc_first_derivative(double t) {
  return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2);
}

double CubicPolynomial::calc_second_derivative(double t) {
  return 2 * a2 + 6 * a3 * t;
}

double CubicPolynomial::calc_third_derivative(double t) { return 6 * a3; }