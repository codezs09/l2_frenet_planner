#ifndef FRENET_OPTIMAL_TRAJECTORY_POLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_POLYNOMIAL_H

class Polynomial {
 public:
  Polynomial() = default;
  virtual double calc_point(double t) = 0;
  virtual double calc_first_derivative(double t) = 0;
  virtual double calc_second_derivative(double t) = 0;
  virtual double calc_third_derivative(double t) = 0;
};

#endif  // FRENET_OPTIMAL_TRAJECTORY_POLYNOMIAL_H
