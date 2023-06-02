#ifndef FRENETOPTIMALTRAJECTORY_CAR_H
#define FRENETOPTIMALTRAJECTORY_CAR_H

#include "utils.h"

#include <memory>
#include <tuple>
#include <vector>

using namespace std;

// Lincoln MKZ configuration
const double VEHICLE_LENGTH = 4.93;
const double VEHICLE_WIDTH = 1.86;

class Car {
 public:
  Car() : length(VEHICLE_LENGTH), width(VEHICLE_WIDTH) {}
  Car(Pose pose_) : Car() { pose = make_unique<Pose>(pose_); }
  Car(Pose pose_, Twist twist_) : Car(pose_) {
    twist = make_unique<Twist>(twist_);
  }
  Car(Pose pose_, Twist twist_, Accel accel_) : Car(pose_, twist_) {
    accel = make_unique<Accel>(accel_);
  }

  bool getOutline(vector<Point> *outline);

  bool isPoseSet() { return (pose != nullptr); }
  void setPose(Pose p) { pose = make_unique<Pose>(p); }
  bool getPose(Pose *p);

  bool isTwistSet() { return (twist != nullptr); }
  void setTwist(Twist t) { twist = make_unique<Twist>(t); }
  bool getTwist(Twist *t);

  bool isAccelSet() { return (accel != nullptr); }
  void setAccel(Accel a) { accel = make_unique<Accel>(a); }
  bool getAccel(Accel *a);

 private:
  double length;
  double width;
  unique_ptr<Pose> pose = nulptr;     // pose w.r.t global frame
  unique_ptr<Twist> twist = nullptr;  // velocity w.r.t vehicle frame
  unique_ptr<Accel> accel = nullptr;  // acceleration w.r.t vehicle frame
};

#endif  // FRENETOPTIMALTRAJECTORY_CAR_H
