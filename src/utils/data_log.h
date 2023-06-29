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
  vector<FrenetPath> frenet_paths;

  MSGPACK_DEFINE(timestamp, ego_car, best_frenet_path, lanes, obstacles,
                 frenet_paths);
};

bool save_data(const std::string& filename,
               const std::vector<DataFrame>& data_frames);

#endif  // UTILS_DATA_LOG_H
