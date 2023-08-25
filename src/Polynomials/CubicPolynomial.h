#ifndef FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H

/**
 * @brief From (s0, s0_dot) to (s1, s1_dot) in time T
 */

class CubicPolynomial {
 public:
  CubicPolynomial() = default;
  CubicPolynomial(double xs, double vxs, double xe, double vxe, double t);
  double calc_point(double t);
  double calc_first_derivative(double t);
  double calc_second_derivative(double t);
  double calc_third_derivative(double t);

 private:
  double a0, a1, a2, a3;
};

#endif  // FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
