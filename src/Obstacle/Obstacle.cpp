#include "Obstacle.h"

#include <QLine>
#include <iostream>

using namespace Eigen;
using namespace std;

void Obstacle::setSpeedLookupTable(const map<double, double> &spd_profile) {
  tbl_time_to_speed_ = make_unique<LookupTable1D>(spd_profile);
}

void Obstacle::setSpeedLookupTable(
    const utils::LookupTable1D &tbl_time_to_speed) {
  tbl_time_to_speed_ = make_unique<LookupTable1D>(tbl_time_to_speed);
}

const utils::LookupTable1D &Obstacle::getSpeedLookupTable() const {
  return *tbl_time_to_speed_;
}

void Obstacle::UpdatePredictBoxes() {
  predict_boxes_.clear();
  for (auto &p : predict_poses_) {
    predict_boxes_.push_back(
        utils::pose_to_box(p.second, length_, width_, obstacle_clearance_));
  }
}

bool Obstacle::predictPoses(double cur_timestamp, double max_duration,
                            double dt) {
  if (tbl_time_to_speed_ == nullptr) {
    cerr << "Error: speed profile not set yet for obstacles! " << endl;
    return false;
  }
  // assuming straight line motion without yaw change
  predict_poses_.clear();
  int num_steps = std::ceil(max_duration / dt);
  Pose p = pose_;
  predict_poses_.insert(std::pair<double, Pose>(cur_timestamp, p));
  for (int i = 1; i <= num_steps; ++i) {
    double t = cur_timestamp + i * dt;
    double v = tbl_time_to_speed_->get(t);
    p.x += v * dt * std::cos(p.yaw);
    p.y += v * dt * std::sin(p.yaw);
    predict_poses_.insert(std::pair<double, Pose>(t, p));
  }
  UpdatePredictBoxes();
  return true;
}

bool Obstacle::predictPoses(const map<double, double> &spd_profile,
                            double cur_timestamp, double max_duration,
                            double dt) {
  setSpeedLookupTable(spd_profile);
  return predictPoses(cur_timestamp, max_duration, dt);
}

Pose Obstacle::getPredictPoseAtTimestamp(double timestamp) {
  if (predict_poses_.empty() || predict_poses_.size() == 1) {
    return pose_;
  }
  for (const auto &p : predict_poses_) {
    if (p.first >= timestamp - 1e-6) {
      return p.second;
    }
  }
  return predict_poses_.rbegin()->second;
}
