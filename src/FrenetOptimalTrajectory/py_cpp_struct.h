#ifndef FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#define FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#include <stddef.h>
#include <eigen3/Eigen/Dense>
#include <nlohmann/json.hpp>
#include <vector>
#include "utils/utils.h"

const size_t MAX_PATH_LENGTH = 100;

using namespace Eigen;
using namespace std;
using json = nlohmann::json;

struct FrenetInitialConditions {
  FrenetInitialConditions() = delete;
  FrenetInitialConditions(WayPoints& wp, vector<Obstacle>& obstacles_c)
      : wp(wp), obstacles_c(obstacles_c) {}

  double s;
  double s_d;
  double s_dd;
  double d;
  double d_d;
  double d_dd;
  double target_speed;

  // TO-DO: 以下两个fields特别是 obstacles_c 感觉可以单独拉出作为输入
  WayPoints& wp;                  // Cartesian coordinates
  vector<Obstacle>& obstacles_c;  // Cartesian coordinates

  // int nw;
  // double *o_llx;
  // double *o_lly;
  // double *o_urx;
  // double *o_ury;
  // int no;
};

struct FrenetReturnValues {
  int success;
  size_t path_length;
  double x_path[MAX_PATH_LENGTH];
  double y_path[MAX_PATH_LENGTH];
  double speeds[MAX_PATH_LENGTH];
  double ix[MAX_PATH_LENGTH];
  double iy[MAX_PATH_LENGTH];
  double iyaw[MAX_PATH_LENGTH];
  double d[MAX_PATH_LENGTH];
  double s[MAX_PATH_LENGTH];
  double speeds_x[MAX_PATH_LENGTH];
  double speeds_y[MAX_PATH_LENGTH];
  double params[MAX_PATH_LENGTH];
  double costs[MAX_PATH_LENGTH];
};

struct FrenetHyperparameters {
 public:
  double max_speed;
  double max_accel;
  double max_curvature;
  double max_road_width_l;
  double max_road_width_r;
  double d_road_w;
  double dt;
  double maxt;
  double mint;
  double d_t_s;
  double n_s_sample;
  double obstacle_clearance;
  double kd;
  double kv;
  double ka;
  double kj;
  double kt;
  double ko;
  double klat;
  double klon;
  double klane;
  int num_threads;

  int sensor_speed_offset;
  int sensor_speed_noise_std;
  int sensor_yaw_rate_offset;
  int sensor_yaw_rate_noise_std;

  void Init(json j) {
    max_speed = j["max_speed"];
    max_accel = j["max_accel"];
    max_curvature = j["max_curvature"];
    max_road_width_l = j["max_road_width_l"];
    max_road_width_r = j["max_road_width_r"];
    d_road_w = j["d_road_w"];
    dt = j["dt"];
    maxt = j["maxt"];
    mint = j["mint"];
    d_t_s = j["d_t_s"];
    n_s_sample = j["n_s_sample"];
    obstacle_clearance = j["obstacle_clearance"];
    kd = j["kd"];
    kv = j["kv"];
    ka = j["ka"];
    kj = j["kj"];
    kt = j["kt"];
    ko = j["ko"];
    klat = j["klat"];
    klon = j["klon"];
    klane = j["klane"];
    num_threads = j["num_threads"];
    sensor_speed_offset = j["sensor_speed_offset"];
    sensor_speed_noise_std = j["sensor_speed_noise_std"];
    sensor_yaw_rate_offset = j["sensor_yaw_rate_offset"];
    sensor_yaw_rate_noise_std = j["sensor_yaw_rate_noise_std"];
  }

  static FrenetHyperparameters& getInstance() {
    static FrenetHyperparameters instance;
    return instance;
  }
  static const FrenetHyperparameters& getConstInstance() {
    return getInstance();
  }

 private:
  FrenetHyperparameters() {}
  FrenetHyperparameters(FrenetHyperparameters const&) = delete;
  FrenetHyperparameters operator=(FrenetHyperparameters const&) = delete;
};
#endif  // FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
