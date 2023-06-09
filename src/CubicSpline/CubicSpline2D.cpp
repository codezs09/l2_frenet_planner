#include "CubicSpline2D.h"
#include "utils/utils.h"

#include <algorithm>
#include <cmath>
#include <numeric>

using namespace std;
using namespace utils;

// Default constructor
CubicSpline2D::CubicSpline2D() = default;

// Construct the 2-dimensional cubic spline
CubicSpline2D::CubicSpline2D(const vector<double>& x, const vector<double>& y) {
  vector<vector<double>> filtered_points = remove_collinear_points(x, y);
  calc_s(filtered_points[0], filtered_points[1]);
  sx = CubicSpline1D(s, filtered_points[0]);
  sy = CubicSpline1D(s, filtered_points[1]);
}

// Calculate the s values for interpolation given x, y
void CubicSpline2D::calc_s(const vector<double>& x, const vector<double>& y) {
  int nx = x.size();
  vector<double> dx(nx);
  vector<double> dy(nx);
  adjacent_difference(x.begin(), x.end(), dx.begin());
  adjacent_difference(y.begin(), y.end(), dy.begin());
  dx.erase(dx.begin());
  dy.erase(dy.begin());

  double cum_sum = 0.0;
  s.push_back(cum_sum);
  for (int i = 0; i < nx - 1; i++) {
    cum_sum += norm(dx[i], dy[i]);
    s.push_back(cum_sum);
  }
  s.erase(unique(s.begin(), s.end()), s.end());
}

// Calculate the x position along the spline at given t
double CubicSpline2D::calc_x(double t) { return sx.calc_der0(t); }

// Calculate the deirvative of x along the spline at given t
double CubicSpline2D::calc_dx_over_ds(double t) { return sx.calc_der1(t); }

// Calculate the y position along the spline at given t
double CubicSpline2D::calc_y(double t) { return sy.calc_der0(t); }

double CubicSpline2D::calc_dy_over_ds(double t) { return sy.calc_der1(t); }

// Calculate the curvature along the spline at given t
double CubicSpline2D::calc_curvature(double t) {
  double dx = sx.calc_der1(t);
  double ddx = sx.calc_der2(t);
  double dy = sy.calc_der1(t);
  double ddy = sy.calc_der2(t);
  double k = (ddy * dx - ddx * dy) / pow(pow(dx, 2) + pow(dy, 2), 1.5);
  return k;
}

// Calculate the yaw along the spline at given t
double CubicSpline2D::calc_yaw(double t) {
  double dx = sx.calc_der1(t);
  double dy = sy.calc_der1(t);
  double yaw = atan2(dy, dx);
  return yaw;
}

double CubicSpline2D::find_s(double x, double y) {
  // TO-DO: use NLopt if necessary later on
  double s_lo = s.front();
  double s_hi = s.back();
  double ds = 10.0;

  double s_at_min_dist, min_dist;
  while (ds > 1e-3) {
    min_dist = std::numeric_limits<double>::max();
    s_at_min_dist = s_lo;
    for (double si = s_lo; si <= s_hi; si += ds) {
      double px = calc_x(si);
      double py = calc_y(si);
      double dist = utils::norm(x - px, y - py);
      if (dist < min_dist) {
        min_dist = dist;
        s_at_min_dist = si;
      }
    }
    // update s_lo and s_hi
    s_lo = std::max(s_lo, s_at_min_dist - 2 * ds);
    s_hi = std::min(s_hi, s_at_min_dist + 2 * ds);
    ds /= 10.0;
  }
  return s_at_min_dist;
}

// Remove any collinear points from given list of points by the triangle rule
vector<vector<double>> CubicSpline2D::remove_collinear_points(
    vector<double> x, vector<double> y) {
  vector<vector<double>> filtered_points;
  vector<double> x_, y_;
  x_.push_back(x[0]);
  x_.push_back(x[1]);
  y_.push_back(y[0]);
  y_.push_back(y[1]);
  for (size_t i = 2; i < x.size() - 1; i++) {
    bool collinear =
        are_collinear(x[i - 2], y[i - 2], x[i - 1], y[i - 1], x[i], y[i]);
    if (collinear) {
      continue;
    }
    x_.push_back(x[i]);
    y_.push_back(y[i]);
  }
  // make sure to add the last point in case all points are collinear
  x_.push_back(x.back());
  y_.push_back(y.back());
  filtered_points.push_back(x_);
  filtered_points.push_back(y_);
  return filtered_points;
}

// Determine if 3 points are collinear using the triangle area rule
bool CubicSpline2D::are_collinear(double x1, double y1, double x2, double y2,
                                  double x3, double y3) {
  double a = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);  // signed
  return fabs(a) <= 0.01;
}
