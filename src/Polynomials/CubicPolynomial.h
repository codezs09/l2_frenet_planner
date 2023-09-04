#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICPOLYNOMIAL_H

#include "Polynomial.h"

/**
 * @brief From (s0, s0_dot) to (s1, s1_dot) in time T
 */

class CubicPolynomial : public Polynomial {
 public:
  CubicPolynomial() = default;
  CubicPolynomial(double xs, double vxs, double xe, double vxe, double t);
  double calc_point(double t) override;
  double calc_first_derivative(double t) override;
  double calc_second_derivative(double t) override;
  double calc_third_derivative(double t) override;

 private:
  double a0, a1, a2, a3;
};

#endif  // FRENET_OPTIMAL_TRAJECTORY_CUBICPOLYNOMIAL_H
