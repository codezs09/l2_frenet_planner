#include "Lane.h"

void Lane::SetWaypoints(const WayPoints& wp) { wp_ = wp; }

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
}

void Lane::SetLeftLaneBoundary(const WayPoints& left_boundary) {
  left_boundary_ = left_boundary;
}

void Lane::SetRightLaneBoundary(const WayPoints& right_boundary) {
  right_boundary_ = right_boundary;
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

  for (int i = 0; i < wp_[0].size(); ++i) {
    double s = csp.find_s(wp_[0][i], wp_[1][i]);
    double x = csp.calc_x(s);
    double y = csp.calc_y(s);
    double yaw = csp.calc_yaw(s);

    left_boundary_[0].push_back(x - half_lane_width * sin(yaw));
    left_boundary_[1].push_back(y + half_lane_width * cos(yaw));

    right_boundary_[0].push_back(x + half_lane_width * sin(yaw));
    right_boundary_[1].push_back(y - half_lane_width * cos(yaw));
  }
}