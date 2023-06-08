#ifndef FRENETOPTIMALTRAJECTORY_CAR_H
#define FRENETOPTIMALTRAJECTORY_CAR_H

#include "utils/utils.h"

#include <memory>
#include <tuple>
#include <vector>

using namespace std;
using namespace utils;

// Lincoln MKZ configuration
const double VEHICLE_LENGTH = 4.93;
const double VEHICLE_WIDTH = 1.86;

class Car {
 public:
  Car() : length(VEHICLE_LENGTH), width(VEHICLE_WIDTH) {}
  Car(Pose pose_) : Car(), pose(pose_) {}
  Car(Pose pose_, Twist twist_) : Car(pose_), twist(twist_) {}
  Car(Pose pose_, Twist twist_, Accel accel_)
      : Car(pose_, twist_), accel(accel_) {}

  Box getBox() { return utils::pose_to_box(pose, length, width); }

  // bool getOutline(vector<Point> *outline);

  void setPose(Pose p) { pose = p; }
  const Pose &getPose() { return pose; }
  Pose *mutablePose() { return &pose; }

  void setTwist(Twist t) { twist = t; }
  const Twist &getTwist() { return twist; }
  Twist *mutableTwist() { return &twist; }

  void setAccel(Accel a) { accel = a; }
  const Accel &getAccel() { return accel; }
  Accel *mutableAccel() { return &accel; }

 private:
  double length;
  double width;
  Pose pose({0, 0, 0});    // pose w.r.t global frame
  Twist twist({0, 0, 0});  // velocity w.r.t vehicle frame
  Accel accel({0, 0, 0});  // acceleration w.r.t vehicle frame
};

#endif  // FRENETOPTIMALTRAJECTORY_CAR_H
