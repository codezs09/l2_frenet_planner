#include "Lane.h"

bool Lane::GetLaneBoundaries(WayPoints* left_boundary,
                             WayPoints* right_boundary) const {
  *left_boundary = left_boundary_;
  *right_boundary = right_boundary_;
  return true;
}

void Lane::SetLaneBoundaries(const WayPoints& left_boundary,
                             const WayPoints& right_boundary) {
  SetLeftLaneBoundary(left_boundary);
  SetRightLaneBoundary(right_boundary);
  CalculateDenseLaneBoundaries();
}

void Lane::SetLeftLaneBoundary(const WayPoints& left_boundary) {
  left_boundary_ = left_boundary;
  CalculateDenseLaneBoundaries(left_boundary_, &left_boundary_dense_);
}

void Lane::SetRightLaneBoundary(const WayPoints& right_boundary) {
  right_boundary_ = right_boundary;
  CalculateDenseLaneBoundaries(right_boundary_, &right_boundary_dense_);
}

void Lane::SetWayPoints(const WayPoints& wp, double lane_width) {
  wp_ = wp;
  lane_width_ = lane_width;
  CalculateLaneBoundariesFromWaypoints();
}

bool Lane::GetLeftLane(Lane* left_lane) {
  if (left_lane_ == nullptr) {
    return false;
  }
  *left_lane = *left_lane_;
  return true;
}

bool Lane::GetRightLane(Lane* right_lane) {
  if (right_lane_ == nullptr) {
    return false;
  }
  *right_lane = *right_lane_;
  return true;
}

void Lane::CalculateLaneBoundariesFromWaypoints() {
  left_boundary_[0].clear();
  left_boundary_[1].clear();
  right_boundary_[0].clear();
  right_boundary_[1].clear();
  double half_lane_width = lane_width_ / 2.0;

  CubicSpline2D csp(wp_[0], wp_[1]);

  for (std::size_t i = 0; i < wp_[0].size(); ++i) {
    double s = csp.find_s(wp_[0][i], wp_[1][i]);
    double x = csp.calc_x(s);
    double y = csp.calc_y(s);
    double yaw = csp.calc_yaw(s);

    left_boundary_[0].push_back(x - half_lane_width * sin(yaw));
    left_boundary_[1].push_back(y + half_lane_width * cos(yaw));

    right_boundary_[0].push_back(x + half_lane_width * sin(yaw));
    right_boundary_[1].push_back(y - half_lane_width * cos(yaw));
  }

  CalculateDenseLaneBoundaries();
}

void Lane::CalculateDenseLaneBoundaries() {
  CalculateDenseLaneBoundaries(left_boundary_, &left_boundary_dense_);
  CalculateDenseLaneBoundaries(right_boundary_, &right_boundary_dense_);
}

void Lane::CalculateDenseLaneBoundaries(const WayPoints& boundary,
                                        WayPoints* dense_boundary) {
  CubicSpline2D spline(boundary[0], boundary[1]);
  double s_lo = spline.s_lo();
  double s_hi = spline.s_hi();
  double ds = 1.0;
  dense_boundary->at(0).clear();
  dense_boundary->at(1).clear();
  for (double s = s_lo; s <= s_hi; s += ds) {
    double x = spline.calc_x(s);
    double y = spline.calc_y(s);
    dense_boundary->at(0).push_back(x);
    dense_boundary->at(1).push_back(y);
  }
}
