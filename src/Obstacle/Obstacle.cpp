#include "Obstacle.h"

#include <QLine>
#include <iostream>

using namespace Eigen;
using namespace std;

// Obstacle::Obstacle(Vector2f first_point, Vector2f second_point, double
// obstacle_clearance)
// {
//     // Get topLeft and bottomRight points from the given points.
//     Vector2f tmp;
//     if (first_point.x() > second_point.x() && first_point.y() >
//     second_point.y()) {
//         tmp = first_point;
//         first_point = second_point;
//         second_point = tmp;
//     } else if (first_point.x() < second_point.x() && first_point.y() >
//     second_point.y()) {
//         float height = first_point.y() - second_point.y();
//         first_point.y() -= height;
//         second_point.y() += height;
//     } else if (first_point.x() > second_point.x() && first_point.y() <
//     second_point.y()) {
//         float length = first_point.x() - second_point.x();
//         first_point.x() -= length;
//         second_point.x() += length;
//     }
//     first_point.x() -= obstacle_clearance;
//     first_point.y() -= obstacle_clearance;
//     second_point.x() += obstacle_clearance;
//     second_point.y() += obstacle_clearance;

//     bbox.first.x() = first_point.x();
//     bbox.first.y() = first_point.y();
//     bbox.second.x() = second_point.x();
//     bbox.second.y() = second_point.y();
// }

void Obstacle::setSpeedLookupTable(const map<double, double> &spd_profile) {
  tbl_time_to_speed_ = make_unique<LookupTable1D>(spd_profile);
}

void Obstacle::setSpeedLookupTable(
    const utils::LookupTable1D &tbl_time_to_speed) {
  tbl_time_to_speed_ = make_unique<LookupTable1D>(tbl_time_to_speed);
}

const utils::LookupTable1D &Obstacle::getSpeedLookupTable() {
  return *tbl_time_to_speed_;
}

bool predictPoses(double cur_timestamp, double max_duration, double dt) {
  if (tbl_time_to_speed_ == nullptr) {
    cerr << "Error: speed profile not set yet for obstacles! " << endl;
    return false;
  }
  // assuming straight line motion without yaw change
  predict_poses_.clear();
  int num_steps = std::ceil(max_duration / dt);
  Pose p = pose_;
  predict_poses_.push_back({cur_timestamp, p});
  for (int i = 1; i <= num_steps; ++i) {
    double t = cur_timestamp + i * dt;
    double v = tbl_time_to_speed_->get(t);
    p.x() += v * dt * std::cos(p.yaw());
    p.y() += v * dt * std::sin(p.yaw());
    predict_poses_.push_back({t, p});
  }
  return true;
}

bool Obstacle::predictPoses(map<double, double> spd_profile,
                            double cur_timestamp, double max_duration,
                            double dt) {
  setSpeedLookupTable(spd_profile);
  return predictPoses(cur_timestamp, max_duration, dt);
}

// Determine whether given line segment intersects an obstacle
// Arguments:
//      p1: point 1 in the line segment
//      p2: point 2 in the line segment
// Returns:
//      whether given line segment intersects an obstacle
bool Obstacle::isSegmentInObstacle(Vector2f &p1, Vector2f &p2) {
  QLineF line_segment(p1.x(), p1.y(), p2.x(), p2.y());
  QPointF intersect_pt;
  float length = bbox.second.x() - bbox.first.x();
  float breadth = bbox.second.y() - bbox.first.y();
  QLineF lseg1(bbox.first.x(), bbox.first.y(), bbox.first.x() + length,
               bbox.first.y());
  QLineF lseg2(bbox.first.x(), bbox.first.y(), bbox.first.x(),
               bbox.first.y() + breadth);
  QLineF lseg3(bbox.second.x(), bbox.second.y(), bbox.second.x(),
               bbox.second.y() - breadth);
  QLineF lseg4(bbox.second.x(), bbox.second.y(), bbox.second.x() - length,
               bbox.second.y());
  QLineF::IntersectType x1 = line_segment.intersect(lseg1, &intersect_pt);
  QLineF::IntersectType x2 = line_segment.intersect(lseg2, &intersect_pt);
  QLineF::IntersectType x3 = line_segment.intersect(lseg3, &intersect_pt);
  QLineF::IntersectType x4 = line_segment.intersect(lseg4, &intersect_pt);
  // check for bounded intersection. IntersectType for bounded intersection
  // is 1.
  if (x1 == 1 || x2 == 1 || x3 == 1 || x4 == 1) {
    return true;
  }

  return false;
}

bool Obstacle::isPointNearObstacle(Vector2f &p, double radius) {
  double dist_to_ll, dist_to_lr, dist_to_ul, dist_to_ur;
  dist_to_ll =
      sqrt(pow(bbox.first.x() - p.x(), 2) + pow(bbox.first.y() - p.y(), 2));
  dist_to_lr =
      sqrt(pow(bbox.second.x() - p.x(), 2) + pow(bbox.first.y() - p.y(), 2));
  dist_to_ul =
      sqrt(pow(bbox.first.x() - p.x(), 2) + pow(bbox.second.y() - p.y(), 2));
  dist_to_ur =
      sqrt(pow(bbox.second.x() - p.x(), 2) + pow(bbox.second.y() - p.y(), 2));
  if (dist_to_ll <= radius || dist_to_lr <= radius || dist_to_ul <= radius ||
      dist_to_ur <= radius) {
    return true;
  }
  return false;
}
