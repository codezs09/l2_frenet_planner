#include "FrenetOptimalTrajectory.h"
#include "FrenetOptimalTrajectory/py_cpp_struct.h"
#include "FrenetPath.h"
#include "fot_wrapper.cpp"
#include "py_cpp_struct.h"
#include "utils.h"

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

bool InitFrenetInitialConditions(const Car& car, const WayPoints& wp,
                                 FrenetInitialConditions* fot_init_cond) {
  // find frenet coordinates
  double frenet_coords[5];  // s, s_d, s_dd, d, d_d, d_dd
  if (!to_frenet_coordinates(scene_j["s0"], car, wp, frenet_coords)) {
    cout << "Fail to find frenet coordinates for car" << endl;
    return false;
  }
  fot_init_cond->s0 = frenet_coords[0];
  fot_init_cond->c_speed = frenet_coords[1];

  // initialize obstacles

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
  WayPoints wp(scene_j["waypoints"]);

  FrenetInitialConditions fot_init_cond;
  if (!InitFrenetInitialConditions(ego_car, wp, &fot_init_cond)) {
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