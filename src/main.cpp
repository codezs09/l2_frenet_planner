#include "FrenetOptimalTrajectory/FrenetOptimalTrajectory.h"
#include "FrenetOptimalTrajectory/py_cpp_struct.h"
#include "FrenetPath.h"
#include "fot_wrapper.cpp"
#include "py_cpp_struct.h"
#include "utils/coordinate_utils.h"
#include "utils/utils.h"

#include <gflags/gflags.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <utility>

using namespace std;
using namespace utils;
using json = nlohmann::json;

DEFINE_string(scene_path,
              "/home/sheng/Project/l2_frenet_planner/"
              "config/scenes/one_lane_slow_down.json",
              "Path to scene config file");
DEFINE_string(hyper_path,
              "/home/sheng/Project/l2_frenet_planner/"
              "config/hyperparameters.json",
              "Path to hyperparameter config file");

double get_duration_ms(
    std::chrono::time_point<std::chrono::high_resolution_clock> end,
    std::chrono::time_point<std::chrono::high_resolution_clock> start) {
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  return static_cast<double>(duration.count()) / 1000.0;
}

bool InitFrenetHyperParameters() {
  json hyper_j;
  if (!LoadJsonFile(FLAGS_hyper_path, &hyper_j)) {
    return false;
  }
  FrenetHyperparameters::getInstance().Init(hyper_j);
  return true;
}

void UpdateFrenetCoordinates(const Car& car, const utils::WayPoints& wp,
                             FrenetInitialConditions* fot_ic) {
  Car car_f;
  utils::ToFrenet(car, wp, &car_f);
  fot_ic->s = car_f.getPose().x;
  fot_ic->s_d = car_f.getTwist().vx;
  fot_ic->s_dd = car_f.getAccel().ax;
  fot_ic->d = car_f.getPose().y;
  fot_ic->d_d = car_f.getTwist().vy;
  fot_ic->d_dd = car_f.getAccel().ay;
}

void InitFrenetInitialConditions(const Car& car, const json& scene_j,
                                 const utils::WayPoints& wp,
                                 FrenetInitialConditions* fot_ic) {
  UpdateFrenetCoordinates(car, wp, fot_ic);
  fot_ic->target_speed = scene_j["target_speed"];
}

void InitObstacles(const json& scene_j, const utils::WayPoints& wp,
                   vector<Obstacle>* obstacles) {
  const auto& fot_hp = FrenetHyperparameters::getConstInstance();
  // const double dt = fot_hp.dt;
  vector<Obstacle> obstacles_f;
  for (const auto& ob_j : scene_j["obs"]) {
    Pose ob_pose = {ob_j["pose"][0], ob_j["pose"][1], ob_j["pose"][2]};
    Obstacle ob(ob_pose, ob_j["length"], ob_j["width"],
                fot_hp.obstacle_clearance);
    if (ob_j.contains("speed_profile")) {
      std::map<double, double> spd_profile;
      for (const auto& spd_j : ob_j["speed_profile"]) {
        spd_profile[spd_j[0]] = spd_j[1];
      }
      ob.setSpeedLookupTable(spd_profile);
      // ob.predictPoses(0.0, fot_hp.maxt, dt);
      ob.setTwist({spd_profile.begin()->second, 0.0, 0.0});
    }
  }
  // convert to Cartesian coordinates
  for (const auto& ob_f : obstacles_f) {
    std::unique_ptr<Obstacle> ob_c = nullptr;
    utils::ToCartesian(ob_f, wp, ob_c);
    obstacles->push_back(std::move(*ob_c));
  }
}

void UpdateEgoCarNextState(const FrenetPath* best_frenet_path,
                           const WayPoints& wp, Car* ego_car) {
  // update ego car to next state
  double next_s = best_frenet_path->s[1];
  double next_d = best_frenet_path->d[1];
  double next_s_d = best_frenet_path->s_d[1];
  double next_d_d = best_frenet_path->d_d[1];
  double next_s_dd = best_frenet_path->s_dd[1];
  double next_d_dd = best_frenet_path->d_dd[1];
  double next_yaw_f = std::atan2(next_d_d, next_s_d);
  double next_yaw_d_f = (next_s_d * next_d_dd - next_d_d * next_s_dd) /
                        (next_s_d * next_s_d + next_d_d * next_d_d);
  Pose pose_c;
  Twist twist_c;
  Accel accel_c;
  ToCartesian({next_s, next_d, next_yaw_f}, {next_s_d, next_d_d, next_yaw_d_f},
              {next_s_dd, next_d_dd, 0.0}, wp, &pose_c, &twist_c, &accel_c);
  ego_car->setPose(pose_c);
  ego_car->setTwist(twist_c);
  ego_car->setAccel(accel_c);
}

void InitWaypoints(const json& scene_j, WayPoints* wp) {
  for (const auto& point : scene_j["wp"]) {
    (*wp)[0].push_back(point[0]);
    (*wp)[1].push_back(point[1]);
  }
}

Car InitEgoCar(const json& scene_j) {
  Pose ego_car_pose = {scene_j["pose"][0], scene_j["pose"][1],
                       scene_j["pose"][2]};
  Twist ego_car_twist = {scene_j["vel"][0], scene_j["vel"][1],
                         scene_j["vel"][2]};
  Car ego_car(ego_car_pose, ego_car_twist, {0, 0, 0});
  return ego_car;
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (!InitFrenetHyperParameters()) {
    return 1;
  }
  json scene_j;
  if (!LoadJsonFile(FLAGS_scene_path, &scene_j)) {
    cout << "Fail to load simulation scene: " << FLAGS_scene_path << endl;
    return 1;
  }

  utils::WayPoints wp;
  InitWaypoints(scene_j, &wp);
  Car ego_car = InitEgoCar(scene_j);

  vector<Obstacle> obstacles;
  InitObstacles(scene_j, wp, &obstacles);

  cout << "debug 12" << endl;

  FrenetInitialConditions fot_ic(wp, obstacles);
  InitFrenetInitialConditions(ego_car, scene_j, wp, &fot_ic);

  cout << "debug 43543f" << endl;

  const auto& fot_hp = FrenetHyperparameters::getConstInstance();
  const double TimeStep = fot_hp.dt;
  // logic similar to fot.py script
  int sim_loop = 200;
  double total_runtime = 0.0;  // [ms]
  double timestamp = 0.0;      // [s], simulation timestamp
  int i = 0;
  for (; i < sim_loop; ++i) {
    auto start = std::chrono::high_resolution_clock::now();

    // break if near goal
    if (utils::norm(ego_car.getPose().x - wp[0].back(),
                    ego_car.getPose().y - wp[1].back()) < 3.0) {
      break;
    }

    // update Frenet coordinate of ego car
    UpdateFrenetCoordinates(ego_car, wp, &fot_ic);
    // prediction on obstacles
    for (auto& ob : obstacles) {
      ob.predictPoses(timestamp, fot_hp.maxt, TimeStep);
    }

    // run frenet optimal trajectory
    FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(fot_ic, fot_hp);
    FrenetPath* best_frenet_path = fot.getBestPath();
    if (!best_frenet_path || !best_frenet_path->x.empty()) {
      cerr << "Fail to find a feasible path at timestamp: " << timestamp
           << endl;
      break;
    }

    auto plan_end = std::chrono::high_resolution_clock::now();
    double plan_duration = get_duration_ms(plan_end, start);

    // update
    timestamp += TimeStep;
    UpdateEgoCarNextState(best_frenet_path, wp, &ego_car);
    // update obstacle to next state
    for (Obstacle& ob : obstacles) {
      Pose ob_pose_next = ob.getPredictPoseAtTimestamp(timestamp);
      ob.setPose(ob_pose_next);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double cycle_duration = get_duration_ms(end, start);
    total_runtime += cycle_duration;

    cout << "Iteration: " << i << ", Simulation time: " << timestamp
         << " [s]. Plan runtime: " << plan_duration
         << " [ms], Cycle runtime: " << cycle_duration << " [ms]. " << endl;
  }
  cout << "Total runtime: " << total_runtime << " [ms] for # " << i
       << " iterations." << endl;

  return 0;
}