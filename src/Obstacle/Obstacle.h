#ifndef FRENETOPTIMALTRAJECTORY_OBSTACLE_H
#define FRENETOPTIMALTRAJECTORY_OBSTACLE_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <map>
#include <memory>
#include <pair>
#include <vector>

#include "utils/lookup_table_1d.h"
#include "utils/utils.h"

using namespace Eigen;
using namespace std;
using namespace utils;

class Obstacle {
 public:
  // std::pair<Vector2f, Vector2f> bbox;

  Obstacle() = delete;
  Obstacle(Pose pose, double length, double width, double obstacle_clearance)
      : pose_(pose), obstacle_clearance_(obstacle_clearance) {
    predict_poses_.push_back({0.0, pose_});
  }
  Obstacle(Pose pose, Twist twist, double length, double width,
           double obstacle_clearance)
      : Obstacle(pose, length, width, obstacle_clearance), twist_(twist) {}

  void setTwist(Twist twist) { twist_ = twist; }

  void setSpeedLookupTable(const map<double, double> &spd_profile);
  void setSpeedLookupTable(const utils::LookupTable1D &tbl_time_to_speed);
  const utils::LookupTable1D &getSpeedLookupTable();

  map<double, Pose> getPredictPoses() const { return predict_poses_; }
  map<double, Pose> *mutablePredictPoses() { return &predict_poses_; }

  double getLength() const { return length_; }
  double getWidth() const { return width_; }
  double getClearence() const { return obstacle_clearance_; }

  bool isSegmentInObstacle(Vector2f &p1, Vector2f &p2);
  bool isPointNearObstacle(Vector2f &p, double radius);
  double getArea();

  // for use under Frenet frame
  bool predictPoses(double cur_timestamp, double max_duration, double dt);
  bool predictPoses(const map<double, double> &spd_profile,
                    double cur_timestamp, double max_duration, double dt);

 private:
  const double length_;  // parallel with yaw
  const double width_;
  const double obstacle_clearance_;

  Pose pose_;
  map<double, Pose> predict_poses_;  // map: timestapm -> pose
  Twist twist_;

  // for use under Frenet frame
  unique_ptr<LookupTable1D> tbl_time_to_speed_ = nullptr;
};

#endif  // FRENETOPTIMALTRAJECTORY_OBSTACLE_H
