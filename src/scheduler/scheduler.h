// unused for now

#ifndef FRENETOPTIMALTRAJECTORY_SCHEDULER_H
#define FRENETOPTIMALTRAJECTORY_SCHEDULER_H

#include "car.h"

class PlanningScheduler {
 public:
  PlanningScheduler() {}
  void Init();

  void Run();

 private:
  void InitFrenetHyperParameters();
  void InitLanes();
  void InitEgoCar();
  void InitObstacles();

  void UpdateSensorMeasurements();
  Pose EstimateChangeOfPose(double speed_meas, double yaw_rate_meas);
  void UpdateEgoCarState();

  void RunLocalPlanning();
};

UpdateFrenetCoordinates(planning_init_point_local, wp_local, &fot_ic);

#endif  // FRENETOPTIMALTRAJECTORY_SCHEDULER_H