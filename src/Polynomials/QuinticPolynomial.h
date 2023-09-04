#ifndef FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H

#include "Polynomial.h"

class QuinticPolynomial : public Polynomial {
 public:
  QuinticPolynomial() = default;
  QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe,
                    double axe, double t);
  double calc_point(double t) override;
  double calc_first_derivative(double t) override;
  double calc_second_derivative(double t) override;
  double calc_third_derivative(double t) override;

 private:
  double a0, a1, a2, a3, a4, a5;
};

#endif  // FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
