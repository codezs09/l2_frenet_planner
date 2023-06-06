#include "FrenetOptimalTrajectory.h"
#include "FrenetOptimalTrajectory/py_cpp_struct.h"
#include "FrenetPath.h"
#include "fot_wrapper.cpp"
#include "py_cpp_struct.h"
#include "utils/coordinate_utils.h"
#include "utils/utils.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

DEFINE_string(scene_path,
              "/home/sheng/Project/l2_frenet_planner/"
              "config/scenes/one_lane_slow_down.json",
              "Path to scene config file");
DEFINE_string(hyper_path,
              "/home/sheng/Project/l2_frenet_planner/hyperparameters.json",
              "Path to hyperparameter config file");

bool InitFrenetHyperParameters() {
  json hyper_j;
  if (!LoadJsonFile(FLAGS_hyper_path, &hyper_j)) {
    return false;
  }
  FrenetHyperparameters::getInstance().Init(hyper_j);
  return true;
}

bool InitFrenetInitialConditions(const Car& car, const json& scene_j,
                                 FrenetInitialConditions* fot_ic) {
  fot_ic->wp = scene_j["waypoints"];

  // convert ego car to frenet coordinates
  Car car_f;
  utils::ToFrenet(car, fot_ic->wp, &car_f);
  fot_ic->s = car_f.getPose().x;
  fot_ic->s_d = car_f.getTwist().vx;
  fot_ic->s_dd = car_f.getAccel().ax;
  fot_ic->d = car_f.getPose().y;
  fot_ic->d_d = car_f.getTwist().vy;
  fot_ic->d_dd = car_f.getAccel().ay;
  fot_ic->target_speed = scene_j["target_speed"];

  // initialize obstacles in frenet coordinates
  auto fot_hp = FrenetHyperparameters::getConstInstance();
  const double dt = fot_hp.dt;
  vector<Obstacle> obstacles_f;
  for (const auto& ob_j : scene_j["obs"]) {
    Pose ob_pose = {ob_j["pose"]};
    Obstacle ob(ob_pose, ob_j["length"], ob_j["width"],
                fot_hp.obstacle_clearance);
    if (ob_j.contains("speed_profile")) {
      std::map<double, double> spd_profile;
      for (const auto& spd_j : ob_j["speed_profile"]) {
        spd_profile[spd_j[0]] = spd_j[1];
      }
      ob.setSpeedLookupTable(spd_profile);
      ob.predictPoses(0.0, fot_hp.maxt, dt);
      ob.setTwist({spd_profile.front().second, 0.0, 0.0});
    }
  }
  // convert to Cartesian coordinates
  for (const auto& ob_f : obstacles_f) {
    std::unique_ptr<Obstacle> ob_c = nullptr;
    utils::ToCartesian(ob_f, fot_ic->wp, ob_c);
    fot_ic->obstacles_c.push_back(*ob_c);
  }

  return true;
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
  Car ego_car({scene_j["pose"]}, {scene_j["vel"]}, {0, 0, 0});

  FrenetInitialConditions fot_ic;
  if (!InitFrenetInitialConditions(ego_car, scene_j, &fot_ic)) {
    return 1;
  }

  const auto& fot_hp = FrenetHyperparameters::getConstInstance();
  const double TimeStep = fot_hp.dt;
  // logic similar to fot.py script
  int sim_loop = 200;
  double total_runtime = 0.0;
  double timestamp = 0.0;  // [s], simulation timestamp
  for (int i = 0; i < sim_loop; ++i) {
    cout << "Iteration: " << i << ", Simulation time: " << timestamp << " [s]"
         << endl;

    // update ego car's Frenet coordinate

    // prediction on obstacles

    // run frenet optimal trajectory
    FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(fot_ic, fot_hp);
    FrenetPath* best_frenet_path = fot.getBestPath();
    if (!best_frenet_path || !best_frenet_path->x.empty()) {
      cerr << "Fail to find a feasible path at timestamp: " << timestamp
           << endl;
      break;
    }

    // update
    timestamp += TimeStep;

    // update ego car to next state
    double next_s = best_frenet_path->s[1];
    double next_d = best_frenet_path->d[1];
    double next_s_d = best_frenet_path->s_d[1];
    double next_d_d = best_frenet_path->d_d[1];
    double next_s_dd = best_frenet_path->s_dd[1];
    double next_d_dd = best_frenet_path->d_dd[1];
    double next_yaw_f = std::atan2(d_d, s_d);
    double next_yaw_d_f = (s_d * d_dd - d_d * s_dd) / (s_d * s_d + d_d * d_d);
    Pose pose_c;
    Twist twist_c;
    Accel accel_c;
    ToCartesian({next_s, next_d, next_yaw_f},
                {next_s_d, next_d_d, next_yaw_d_f}, {next_s_dd, next_d_dd, 0.0},
                fot_ic.wp, &pose_c, &twist_c, &accel_c);
    ego_car.setPose(pose_c);
    ego_car.setTwist(twist_c);
    ego_car.setAccel(accel_c);

    // update obstacle to next state
    for (Obstacle& ob : fot_ic.obstacles_c) {
      Pose ob_pose_next = ob.getPredictPoseAtTimestamp(timestamp);
      ob.setPose(ob_pose_next);
    }
  }

  // run experiment
  FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
  FrenetPath* best_frenet_path = fot.getBestPath();
  if (best_frenet_path) {
    cout << "Success\n";
    return 1;
  }
  cout << "Failure\n";
  return 0;
}