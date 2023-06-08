#ifndef FRENETOPTIMALTRAJECTORY_OBSTACLE_H
#define FRENETOPTIMALTRAJECTORY_OBSTACLE_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <map>
#include <memory>
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
  Obstacle() = delete;
  Obstacle(Pose pose, double length, double width, double obstacle_clearance)
      : Obstacle(pose, {0, 0, 0}, length, width, obstacle_clearance) {}
  Obstacle(Pose pose, Twist twist, double length, double width,
           double obstacle_clearance)
      : pose_(pose),
        twist_(twist),
        length_(length),
        width_(width),
        obstacle_clearance_(obstacle_clearance) {
    predict_poses_.insert(std::pair<double, Pose>(0.0, pose_));
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

  // for use under Frenet frame
  bool predictPoses(double cur_timestamp, double max_duration, double dt);
  bool predictPoses(const map<double, double> &spd_profile,
                    double cur_timestamp, double max_duration, double dt);
  Pose getPredictPoseAtTimestamp(double timestamp);

  const vector<Box> &getPredictBoxes() const { return predict_boxes_; }

 private:
  void UpdatePredictBoxes();

  const double length_;  // parallel with yaw
  const double width_;
  const double obstacle_clearance_;

  Pose pose_;
  map<double, Pose> predict_poses_;  // map: timestapm -> pose
  Twist twist_;
  vector<Box> predict_boxes_;

  // for use under Frenet frame
  unique_ptr<LookupTable1D> tbl_time_to_speed_ = nullptr;
};

#endif  // FRENETOPTIMALTRAJECTORY_OBSTACLE_H
