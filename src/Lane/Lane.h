#ifndef FRENETOPTIMALTRAJECTORY_LANE_H
#define FRENETOPTIMALTRAJECTORY_LANE_H

#include <cmath>
#include <vector>

#include "CubicSpline/CubicSpline2D.h"
#include "utils/utils.h"

using namespace std;
using namespace utils;

class Lane {
 public:
  Lane(const WayPoints& wp, double lane_width = 4.0, Lane* left_lane = nullptr,
       Lane* right_lane = nullptr)
      : wp_(wp),
        lane_width_(lane_width),
        left_lane_(left_lane),
        right_lane_(right_lane) {
    CalculateLaneBoundariesFromWaypoints();
  }
  Lane(const WayPoints& wp, double lane_width, Lane* left_lane,
       Lane* right_lane, const WayPoints& left_boundary,
       const WayPoints& right_boundary)
      : wp_(wp),
        lane_width_(lane_width),
        left_boundary_(left_boundary),
        right_boundary_(right_boundary),
        left_lane_(left_lane),
        right_lane_(right_lane) {}

  // copy constructor
  Lane(const Lane& other)
      : wp_(other.wp_),
        lane_width_(other.lane_width_),
        left_boundary_(other.left_boundary_),
        right_boundary_(other.right_boundary_),
        left_lane_(other.left_lane_),
        right_lane_(other.right_lane_) {}

  // copy assignment operator
  Lane& operator=(const Lane& other) {
    if (this != &other) {
      // copy and swap idiom
      Lane copy(other);
      std::swap(*this, copy);
    }
    return *this;
  }

  void SetWayPoints(const WayPoints& wp, double lane_width = 4.0);

  bool GetLaneBoundaries(WayPoints* left_boundary,
                         WayPoints* right_boundary) const;
  void SetLaneBoundaries(const WayPoints& left_boundary,
                         const WayPoints& right_boundary);
  void SetLeftLaneBoundary(const WayPoints& left_boundary);
  void SetRightLaneBoundary(const WayPoints& right_boundary);

  const WayPoints& GetWayPoints() const { return wp_; }
  double GetLaneWidth() const { return lane_width_; }
  bool GetLeftLane(Lane* left_lane);
  bool GetRightLane(Lane* right_lane);
  Lane* MutableLeftLane() { return left_lane_; }    // check nullptr before use
  Lane* MutableRightLane() { return right_lane_; }  // check nullptr before use

 private:
  void CalculateLaneBoundariesFromWaypoints();

  WayPoints wp_;
  WayPoints left_boundary_;
  WayPoints right_boundary_;

  double lane_width_;
  Lane* left_lane_ = nullptr;
  Lane* right_lane_ = nullptr;
};

#endif  // FRENETOPTIMALTRAJECTORY_LANE_H