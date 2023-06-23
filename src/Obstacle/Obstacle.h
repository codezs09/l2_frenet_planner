#ifndef FRENETOPTIMALTRAJECTORY_OBSTACLE_H
#define FRENETOPTIMALTRAJECTORY_OBSTACLE_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <map>
#include <memory>
#include <msgpack.hpp>
#include <unordered_set>
#include <utility>
#include <vector>

#include "utils/geometry.h"
#include "utils/lookup_table_1d.h"
#include "utils/utils.h"

using namespace Eigen;
using namespace std;
using namespace utils;

class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(Pose pose, double length, double width, double obstacle_clearance)
      : Obstacle(pose, {0, 0, 0}, length, width, obstacle_clearance) {}
  Obstacle(Pose pose, Twist twist, double length, double width,
           double obstacle_clearance)
      : length_(length),
        width_(width),
        obstacle_clearance_(obstacle_clearance),
        pose_(pose),
        twist_(twist) {
    predict_poses_.insert(std::pair<double, Pose>(0.0, pose_));
  }

  // copy constructor
  Obstacle(const Obstacle &other)
      : length_(other.length_),
        width_(other.width_),
        obstacle_clearance_(other.obstacle_clearance_),
        lane_ids_(other.lane_ids_),
        pose_(other.pose_),
        twist_(other.twist_),
        predict_poses_(other.predict_poses_),
        predict_boxes_(other.predict_boxes_),
        tbl_time_to_speed_(new LookupTable1D(*(other.tbl_time_to_speed_))) {}

  // copy assignment operator
  Obstacle &operator=(const Obstacle &other) {
    if (this != &other) {
      // copy and swap idiom
      Obstacle copy(other);
      std::swap(*this, copy);
    }
    return *this;
  }

  void setPose(Pose pose) { pose_ = pose; }
  void setTwist(Twist twist) { twist_ = twist; }

  const Pose &getPose() const { return pose_; }
  const Twist &getTwist() const { return twist_; }

  void setSpeedLookupTable(const map<double, double> &spd_profile);
  void setSpeedLookupTable(const utils::LookupTable1D &tbl_time_to_speed);
  const utils::LookupTable1D &getSpeedLookupTable() const;

  map<double, Pose> getPredictPoses() const { return predict_poses_; }
  map<double, Pose> *mutablePredictPoses() { return &predict_poses_; }

  double getLength() const { return length_; }
  double getWidth() const { return width_; }
  double getClearence() const { return obstacle_clearance_; }

  bool isInLane(int lane_id) const { return (lane_ids_.count(lane_id) > 0); }
  void addLaneId(int lane_id) { lane_ids_.insert(lane_id); }
  void setLaneIds(unordered_set<int> lane_ids) {
    lane_ids_ = std::move(lane_ids);
  }
  void clearLaneIds() { lane_ids_.clear(); }
  const unordered_set<int> &getLaneIds() const { return lane_ids_; }

  // for use under Frenet frame
  bool predictPoses(double cur_timestamp, double max_duration, double dt);
  bool predictPoses(const map<double, double> &spd_profile,
                    double cur_timestamp, double max_duration, double dt);
  Pose getPredictPoseAtTimestamp(double timestamp);

  const vector<Box> &getPredictBoxes() const { return predict_boxes_; }

  // private: // set public for convenience to use msgpack
  double length_;  // parallel with yaw
  double width_;
  double obstacle_clearance_;
  std::unordered_set<int> lane_ids_;  // could occupy multiple lanes

  Pose pose_;
  Twist twist_;
  map<double, Pose> predict_poses_;  // map: timestapm -> pose
  vector<Box> predict_boxes_;

  MSGPACK_DEFINE(length_, width_, obstacle_clearance_, pose_, twist_,
                 predict_boxes_);

 private:
  void UpdatePredictBoxes();

  // for use under Frenet frame
  unique_ptr<LookupTable1D> tbl_time_to_speed_ = nullptr;
};

#endif  // FRENETOPTIMALTRAJECTORY_OBSTACLE_H
