#ifndef FRENETOPTIMALTRAJECTORY_OBSTACLE_H
#define FRENETOPTIMALTRAJECTORY_OBSTACLE_H

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "utils.h"

using namespace Eigen;
using namespace std;

class Obstacle {
 public:
  std::pair<Vector2f, Vector2f> bbox;
  Obstacle(Vector2f first_point, Vector2f second_point,
           double obstacle_clearance);
  bool isSegmentInObstacle(Vector2f &p1, Vector2f &p2);
  bool isPointNearObstacle(Vector2f &p, double radius);
  double getArea();

 private:
  unique_ptr<Pose> pose_frenet_ = nullptr;
  unique_ptr<Pose> pose_cartesian_ = nullptr;
  double length_;  // parallel with yaw
  double width_;
  vector<double> wx_;
  vector<double> wy_;
  vector<Pose> pred_poses_frenet_;  // prediction
  vector<Pose> pred_poses_cartesian_;
};

#endif  // FRENETOPTIMALTRAJECTORY_OBSTACLE_H
