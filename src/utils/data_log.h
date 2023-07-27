#ifndef UTILS_DATA_LOG_H
#define UTILS_DATA_LOG_H

#include <fstream>
#include <iostream>
#include <msgpack.hpp>
#include <string>
#include <vector>

#include "Car/Car.h"
#include "FrenetOptimalTrajectory/FrenetPath.h"
#include "Lane/Lane.h"
#include "Obstacle/Obstacle.h"

using namespace std;

struct DataFrame {
  double timestamp;
  Car ego_car;
  FrenetPath best_frenet_path;
  vector<Lane> lanes;
  vector<Obstacle> obstacles;
  vector<FrenetPath> frenet_paths;  // best frenet path for each lane

  std::vector<Obstacle> obstacles_local;
  unordered_map<int, WayPoints> wp_lanes_local;
  Car planning_init_point_local;
  FrenetPath best_frenet_path_local;
  vector<FrenetPath> frenet_paths_local;  // best frenet path for each lane
  vector<vector<FrenetPath>>
      frenet_paths_local_all;  // all candidate trajectories at each lane and
                               // each d offset
  double speed_meas;
  double yaw_rate_meas;
  Pose pose_change_est;

  MSGPACK_DEFINE(timestamp, ego_car, best_frenet_path, lanes, obstacles,
                 frenet_paths, obstacles_local, wp_lanes_local,
                 planning_init_point_local, best_frenet_path_local,
                 frenet_paths_local, frenet_paths_local_all, speed_meas,
                 yaw_rate_meas, pose_change_est);
};

bool save_data(const std::string& filename,
               const std::vector<DataFrame>& data_frames);

#endif  // UTILS_DATA_LOG_H
