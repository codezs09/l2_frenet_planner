#include "FrenetOptimalTrajectory/FrenetOptimalTrajectory.h"
#include "FrenetOptimalTrajectory/py_cpp_struct.h"
#include "FrenetPath.h"
#include "Lane/Lane.h"
#include "fot_wrapper.cpp"
#include "py_cpp_struct.h"
#include "utils/coordinate_utils.h"
#include "utils/data_log.h"
#include "utils/debug.h"
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

const std::string REPO_DIR = "/home/sheng/Projects/l2_frenet_planner/";

DEFINE_string(scene_path, REPO_DIR + "config/scenes/one_lane_slow_down.json",
              "Path to scene config file");
DEFINE_string(hyper_path, REPO_DIR + "config/hyperparameters.json",
              "Path to hyperparameter config file");
DEFINE_bool(store_data, false, "turn on flag to store running data.");
DEFINE_string(data_path, REPO_DIR + "build/data.bin",
              "Path to store running data.");

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

// void InitFrenetInitialConditions(const Car& car, const json& scene_j,
//                                  const utils::WayPoints& wp,
//                                  FrenetInitialConditions* fot_ic) {
//   UpdateFrenetCoordinates(car, wp, fot_ic);
// }

void InitWaypoints(const json& scene_j, WayPoints* wp) {
  for (const auto& point : scene_j["wp"]) {
    (*wp)[0].push_back(point[0]);
    (*wp)[1].push_back(point[1]);
  }
}

void InitObstacles(const json& scene_j, const vector<Lane>& lanes,
                   vector<Obstacle>* const obstacles) {
  utils::WayPoints ref_wp;
  InitWaypoints(scene_j, &ref_wp);

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
    obstacles_f.push_back(std::move(ob));
  }

  // convert to Cartesian coordinates
  // note: the obstacle initial pose defined in scene in Frenet frame is w.r.t.
  // ref_wp
  for (const auto& ob_f : obstacles_f) {
    std::unique_ptr<Obstacle> ob_c = nullptr;
    utils::ToCartesian(ob_f, ref_wp, ob_c);
    obstacles->push_back(std::move(*ob_c));
  }

  // set lane id for each obstacle as well.
  for (auto& ob : *obstacles) {
    ob.clearLaneIds();
    for (std::size_t i = 0; i < lanes.size(); ++i) {
      if (point_in_lane(lanes[i], ob.getPose().x, ob.getPose().y)) {
        ob.addLaneId(i);
      }
    }
    auto& lane_ids = ob.getLaneIds();
    vector<int> vec_lane_ids(lane_ids.begin(), lane_ids.end());
    cout << "ob lane_ids: " << utils::vector_to_str(vec_lane_ids) << endl;
  }
}

void UpdateEgoCarNextState(const FrenetPath* best_frenet_path,
                           const vector<Lane>& lanes, Car* ego_car) {
  const WayPoints& wp = lanes[best_frenet_path->lane_id].GetWayPoints();
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
  ego_car->setTargetLaneId(best_frenet_path->lane_id);
}

Car InitEgoCar(const json& scene_j, const vector<Lane>& lanes) {
  Pose ego_car_pose = {scene_j["pose"][0], scene_j["pose"][1],
                       scene_j["pose"][2]};
  Twist ego_car_twist = {scene_j["vel"][0], scene_j["vel"][1],
                         scene_j["vel"][2]};
  Car ego_car(ego_car_pose, ego_car_twist, {0, 0, 0});

  // initialize lane_id for ego car
  int lane_id = -1;
  for (std::size_t i = 0; i < lanes.size(); ++i) {
    if (point_in_lane(lanes[i], ego_car_pose.x, ego_car_pose.y)) {
      lane_id = i;
      break;
    }
  }
  ego_car.setCurLaneId(lane_id);
  ego_car.setTargetLaneId(lane_id);

  return ego_car;
}

void InitLanes(const json& scene_j, vector<Lane>* lanes) {
  utils::WayPoints ref_wp;
  InitWaypoints(scene_j, &ref_wp);

  const double lane_width = scene_j["lane_width"];
  const int num_lanes_left = scene_j["num_lanes_left"];
  const int num_lanes_right = scene_j["num_lanes_right"];
  const int total_lanes = num_lanes_left + num_lanes_right + 1;

  for (int i = -num_lanes_left; i <= num_lanes_right; ++i) {
    utils::WayPoints lane_wp;
    if (i == 0) {
      lane_wp = ref_wp;
    } else {
      utils::ShiftWaypoints(ref_wp, -i * lane_width,
                            &lane_wp);  // positive shift to the left
    }
    int lane_id = i + num_lanes_left;
    Lane lane(lane_id, lane_wp, lane_width);
    lanes->push_back(lane);
  }

  // lane associations
  for (std::size_t i = 0; i < lanes->size(); ++i) {
    if (i > 0) {
      (*lanes)[i].SetLeftLane(&(*lanes)[i - 1]);
    }
    if (i < lanes->size() - 1) {
      (*lanes)[i].SetLeftLane(&(*lanes)[i + 1]);
    }
  }

  // calculate lane boundaries
  utils::WayPoints left_lane_bound;
  utils::WayPoints right_lane_bound;
  for (int i = -num_lanes_left; i <= num_lanes_right; ++i) {
    if (i == -num_lanes_left) {
      utils::ShiftWaypoints(ref_wp, (-i + 0.5) * lane_width, &left_lane_bound);
    }
    utils::ShiftWaypoints(ref_wp, (-i - 0.5) * lane_width, &right_lane_bound);

    Lane& lane = (*lanes)[i + num_lanes_left];
    lane.SetLeftLaneBoundary(left_lane_bound);
    lane.SetRightLaneBoundary(right_lane_bound);

    left_lane_bound = right_lane_bound;
  }
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

  vector<Lane> lanes;
  InitLanes(scene_j, &lanes);

  Car ego_car = InitEgoCar(scene_j, lanes);

  vector<Obstacle> obstacles;
  InitObstacles(scene_j, lanes, &obstacles);

  double target_speed = scene_j["target_speed"];

  const auto& fot_hp = FrenetHyperparameters::getConstInstance();
  const double TimeStep = fot_hp.dt;
  int sim_loop = 200;
  double total_runtime = 0.0;  // [ms]
  double timestamp = 0.0;      // [s], simulation timestamp
  int i = 0;
  std::vector<DataFrame> data_frames;
  bool reach_goal = false;
  std::vector<FrenetPath>
      best_frenet_paths;  // store the best frenet path for each lane

  for (; i < sim_loop; ++i) {
    auto start = std::chrono::high_resolution_clock::now();

    // Loop each lane here and may initialize fot_ic for each lane
    best_frenet_paths.clear();
    for (const auto& lane : lanes) {
      WayPoints wp = lane.GetWayPoints();

      // break if near goal
      if (utils::norm(ego_car.getPose().x - wp[0].back(),
                      ego_car.getPose().y - wp[1].back()) < 3.0) {
        reach_goal = true;
        break;
      }

      FrenetInitialConditions fot_ic(wp, obstacles);
      fot_ic.target_speed = target_speed;

      // update Frenet coordinate of ego car
      UpdateFrenetCoordinates(ego_car, wp, &fot_ic);
      // prediction on obstacles
      for (auto& ob : obstacles) {
        std::unique_ptr<Obstacle> ob_f = nullptr;
        int ob_laneid = 0;
        if (!ob.getLaneIds().empty()) {
          ob_laneid = *(ob.getLaneIds().begin());
        }
        utils::ToFrenet(ob, lanes[ob_laneid].GetWayPoints(), ob_f);
        ob_f->predictPoses(timestamp, fot_hp.maxt, TimeStep);
        std::unique_ptr<Obstacle> ob_c = nullptr;
        utils::ToCartesian(*ob_f, lanes[ob_laneid].GetWayPoints(), ob_c);
        ob = std::move(*ob_c);
      }

      // run frenet optimal trajectory
      FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(fot_ic, fot_hp);
      FrenetPath* best_frenet_path_per_lane = fot.getBestPath();
      if (!best_frenet_path_per_lane || best_frenet_path_per_lane->x.empty()) {
        cerr << "Fail to find a feasible path at timestamp: " << timestamp
             << ", at lane: " << lane.GetLaneId() << endl;
      } else {
        // update cost for each frenet path based on lane
        best_frenet_path_per_lane->lane_id = lane.GetLaneId();
        best_frenet_path_per_lane->c_lane_change = std::abs(
            best_frenet_path_per_lane->lane_id - ego_car.getTargetLaneId());
        best_frenet_path_per_lane->cf +=
            fot_hp.klane * best_frenet_path_per_lane->c_lane_change;

        best_frenet_paths.push_back(std::move(*best_frenet_path_per_lane));
      }
    }
    if (reach_goal) {
      break;
    }

    // update cost for each frenet path based on lane
    // choose from best trajectory along each lane based on cost
    FrenetPath* best_frenet_path = nullptr;
    for (auto& fp : best_frenet_paths) {
      if (!best_frenet_path) {
        best_frenet_path = &fp;
      } else {
        if (fp.cf < best_frenet_path->cf) {
          best_frenet_path = &fp;
        }
      }
    }

    auto plan_end = std::chrono::high_resolution_clock::now();
    double plan_duration = get_duration_ms(plan_end, start);

    // save current frame data
    if (FLAGS_store_data) {
      DataFrame df;
      df.timestamp = timestamp;
      df.ego_car = ego_car;
      df.best_frenet_path = *best_frenet_path;
      df.lanes = lanes;
      df.obstacles = obstacles;
      df.frenet_paths = best_frenet_paths;
      data_frames.push_back(std::move(df));
    }

    // update
    timestamp += TimeStep;
    UpdateEgoCarNextState(best_frenet_path, lanes, &ego_car);
    // update obstacle to next state
    for (Obstacle& ob : obstacles) {
      Pose ob_pose_next = ob.getPredictPoseAtTimestamp(timestamp);
      ob.setPose(ob_pose_next);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double cycle_duration = get_duration_ms(end, start);
    total_runtime += cycle_duration;

    cout << "#" << i << ", simtime: " << timestamp
         << "[s]. Plan: " << plan_duration << "[ms], Cycle: " << cycle_duration
         << "[ms]. x=" << ego_car.getPose().x << ", y=" << ego_car.getPose().y
         << ", yaw=" << utils::rad2deg(ego_car.getPose().yaw)
         << "[deg]. vx=" << ego_car.getTwist().vx
         << ", vy=" << ego_car.getTwist().vy
         << ", w=" << utils::rad2deg(ego_car.getTwist().yaw_rate)
         << "[deg/s]. ax=" << ego_car.getAccel().ax << endl;
  }
  cout << "Total runtime: " << total_runtime << " [ms] for # " << i
       << " iterations." << endl;

  if (FLAGS_store_data) {
    if (save_data(FLAGS_data_path, data_frames)) {
      cout << "Data logging at: " << FLAGS_data_path << endl;
    }
  }
  return 0;
}