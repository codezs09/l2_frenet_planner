#include "FrenetOptimalTrajectory.h"
#include "FrenetOptimalTrajectory/py_cpp_struct.h"
#include "FrenetPath.h"
#include "fot_wrapper.cpp"
#include "py_cpp_struct.h"
#include "utils/coordinate_utils.h"
#include "utils/utils.h"

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
  Pose pose_c;
  if (!car.getPose(&pose_c)) {
    cout << "Fail to get pose from ego car" << endl;
    return false;
  }
  Twist twist_c;
  if (!car.getTwist(&twist_c)) {
    cout << "Fail to get twist from ego car" << endl;
    return false;
  }
  Accel accel_c;
  if (!car.getAccel(&accel_c)) {
    cout << "Fail to get accel from ego car" << endl;
    return false;
  }
  Pose pose_f;
  Twist twist_f;
  Accel accel_f;
  utils::ToFrenet(pose_c, twist_c, accel_c, fot_ic->wp, &pose_f, &twist_f,
                  &accel_f);
  fot_ic->s = pose_f.x;
  fot_ic->s_d = twist_f.vx;
  fot_ic->s_dd = accel_f.ax;
  fot_ic->d = pose_f.y;
  fot_ic->d_d = twist_f.vy;
  fot_ic->d_dd = accel_f.ay;
  fot_ic->target_speed = scene_j["target_speed"];

  // initialize obstacles in frenet coordinates
  auto hyper_params = FrenetHyperparameters::getConstInstance();
  const double dt = hyper_params.dt;
  vector<Obstacle> obstacles_f;
  for (const auto& ob_j : scene_j["obs"]) {
    Pose ob_pose = {ob_j["pose"]};
    Obstacle ob(ob_pose, ob_j["length"], ob_j["width"],
                hyper_params.obstacle_clearance);
    if (ob_j.contains("speed_profile")) {
      std::map<double, double> spd_profile;
      for (const auto& spd_j : ob_j["speed_profile"]) {
        spd_profile[spd_j[0]] = spd_j[1];
      }
      ob.setSpeedLookupTable(spd_profile);
      ob.predictPoses(0.0, hyper_params.maxt, dt);
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

  const auto& frenet_hyper_parameters =
      FrenetHyperparameters::getConstInstance();
  const double TimeStep = frenet_hyper_parameters.dt;
  // logic similar to fot.py script
  int sim_loop = 200;
  double total_runtime = 0.0;
  double timestamp = 0.0;  // [s], simulation timestamp
  for (int i = 0; i < sim_loop; ++i) {
    cout << "Iteration: " << i << ", Simulation time: " << timestamp << " [s]"
         << endl;
    // prediction on obstacles

    // run frenet optimal trajectory
    RunFot(state, hyperparameters);

    // update states, ego and obstacles

    timestamp += TimeStep;
  }

  // double wx [25] = {132.67, 128.67, 124.67, 120.67, 116.67, 112.67, 108.67,
  //                104.67, 101.43,  97.77,  94.84,  92.89,  92.4 ,  92.4 ,
  //                92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.39,  92.39,
  //                92.39,  92.39,  92.39,  92.39};
  // double wy [25] = {195.14, 195.14, 195.14, 195.14, 195.14, 195.14, 195.14,
  //                195.14, 195.14, 195.03, 193.88, 191.75, 188.72, 185.32,
  //                181.32, 177.32, 173.32, 169.32, 165.32, 161.32, 157.32,
  //                153.32, 149.32, 145.32, 141.84};
  // double o_llx[1] = {92.89};
  // double o_lly[1] = {191.75};
  // double o_urx[1] = {92.89};
  // double o_ury[1] = {191.75};

  // // set up experiment
  // FrenetInitialConditions fot_ic = {
  //     34.6,
  //     7.10964962,
  //     -1.35277168,
  //     -1.86,
  //     0.0,
  //     10,
  //     wx,
  //     wy,
  //     25,
  //     o_llx,
  //     o_lly,
  //     o_urx,
  //     o_ury,
  //     1
  // };
  // FrenetHyperparameters fot_hp = {
  //     25.0,
  //     15.0,
  //     15.0,
  //     5.0,
  //     5.0,
  //     0.5,
  //     0.2,
  //     5.0,
  //     2.0,
  //     0.5,
  //     2.0,
  //     0.1,
  //     1.0,
  //     0.1,
  //     0.1,
  //     0.1,
  //     0.1,
  //     0.1,
  //     1.0,
  //     1.0,
  //     2 // num thread
  // };

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