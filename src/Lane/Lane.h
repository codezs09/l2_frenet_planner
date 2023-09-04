#ifndef FRENETOPTIMALTRAJECTORY_LANE_H
#define FRENETOPTIMALTRAJECTORY_LANE_H

#include <cmath>
#include <msgpack.hpp>
#include <vector>

#include "CubicSpline/CubicSpline2D.h"
#include "utils/utils.h"

using namespace std;
using namespace utils;

class Lane {
 public:
  Lane() = default;
  Lane(int lane_id, const WayPoints& wp, double lane_width = 4.0,
       Lane* left_lane = nullptr, Lane* right_lane = nullptr)
      : wp_(wp),
        lane_id_(lane_id),
        lane_width_(lane_width),
        left_lane_(left_lane),
        right_lane_(right_lane) {
    CalculateLaneBoundariesFromWaypoints();
  }
  Lane(int lane_id, const WayPoints& wp, double lane_width, Lane* left_lane,
       Lane* right_lane, const WayPoints& left_boundary,
       const WayPoints& right_boundary)
      : wp_(wp),
        left_boundary_(left_boundary),
        right_boundary_(right_boundary),
        lane_id_(lane_id),
        lane_width_(lane_width),
        left_lane_(left_lane),
        right_lane_(right_lane) {
    CalculateDenseLaneBoundaries();
  }

  // copy constructor
  Lane(const Lane& other)
      : wp_(other.wp_),
        left_boundary_(other.left_boundary_),
        right_boundary_(other.right_boundary_),
        lane_id_(other.lane_id_),
        lane_width_(other.lane_width_),
        left_boundary_dense_(other.left_boundary_dense_),
        right_boundary_dense_(other.right_boundary_dense_),
        left_lane_(other.left_lane_),
        right_lane_(other.right_lane_) {}

  // copy assignment operator
  Lane& operator=(const Lane& other) {
    if (this != &other) {
      wp_ = other.wp_;
      left_boundary_ = other.left_boundary_;
      right_boundary_ = other.right_boundary_;
      lane_id_ = other.lane_id_;
      lane_width_ = other.lane_width_;
      left_boundary_dense_ = other.left_boundary_dense_;
      right_boundary_dense_ = other.right_boundary_dense_;
      left_lane_ = other.left_lane_;
      right_lane_ = other.right_lane_;
    }
    return *this;
  }

  void SetWayPoints(const WayPoints& wp, double lane_width = 4.0);
  const WayPoints& GetWayPoints() const { return wp_; }

  void SetLaneId(int lane_id) { lane_id_ = lane_id; }
  int GetLaneId() const { return lane_id_; }

  bool GetLaneBoundaries(WayPoints* left_boundary,
                         WayPoints* right_boundary) const;
  void SetLaneBoundaries(const WayPoints& left_boundary,
                         const WayPoints& right_boundary);
  void SetLeftLaneBoundary(const WayPoints& left_boundary);
  void SetRightLaneBoundary(const WayPoints& right_boundary);

  double GetLaneWidth() const { return lane_width_; }

  void SetLeftLane(Lane* lane) { left_lane_ = lane; }
  bool GetLeftLane(Lane* left_lane);
  Lane* MutableLeftLane() { return left_lane_; }  // check nullptr before use

  void SetRightLane(Lane* lane) { right_lane_ = lane; }
  bool GetRightLane(Lane* right_lane);
  Lane* MutableRightLane() { return right_lane_; }  // check nullptr before use

  WayPoints wp_;
  WayPoints left_boundary_;
  WayPoints right_boundary_;

  int lane_id_;
  double lane_width_;

  // debug purposes
  WayPoints left_boundary_dense_;
  WayPoints right_boundary_dense_;

  MSGPACK_DEFINE(wp_, left_boundary_dense_, right_boundary_dense_, lane_id_,
                 lane_width_);

 private:
  void CalculateLaneBoundariesFromWaypoints();
  void CalculateDenseLaneBoundaries();
  void CalculateDenseLaneBoundaries(const WayPoints& boundary,
                                    WayPoints* dense_boundary);

  Lane* left_lane_ = nullptr;  // or change to left_lane_id
  Lane* right_lane_ = nullptr;
};

#endif  // FRENETOPTIMALTRAJECTORY_LANE_H