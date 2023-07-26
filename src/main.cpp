#include <gflags/gflags.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <utility>

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
DEFINE_bool(local_planning, true, "turn on flag to enable local planning.");

double get_duration_ms(
    std::chrono::time_point<std::chrono::high_resolution_clock> end,
    std::chrono::time_point<std::chrono::high_resolution_clock> start) {
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  return static_cast<double>(duration.count()) / 1000.0;
}

double round_to_tenth(double x) { return std::round(x * 10.0) / 10.0; }

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

void UpdateNextPlanningStateLocal(const FrenetPath* best_frenet_path_local,
                                  const WayPoints& wp_local,
                                  Car* next_planning_state_local) {
  // update ego car to next state
  double next_s = best_frenet_path_local->s[1];
  double next_d = best_frenet_path_local->d[1];
  double next_s_d = best_frenet_path_local->s_d[1];
  double next_d_d = best_frenet_path_local->d_d[1];
  double next_s_dd = best_frenet_path_local->s_dd[1];
  double next_d_dd = best_frenet_path_local->d_dd[1];
  double next_yaw_f = std::atan2(next_d_d, next_s_d);
  double next_yaw_d_f = (next_s_d * next_d_dd - next_d_d * next_s_dd) /
                        (next_s_d * next_s_d + next_d_d * next_d_d);
  Pose pose_c;
  Twist twist_c;
  Accel accel_c;
  ToCartesian({next_s, next_d, next_yaw_f}, {next_s_d, next_d_d, next_yaw_d_f},
              {next_s_dd, next_d_dd, 0.0}, wp_local, &pose_c, &twist_c,
              &accel_c);
  next_planning_state_local->setPose(pose_c);
  next_planning_state_local->setTwist(twist_c);
  next_planning_state_local->setAccel(accel_c);
  next_planning_state_local->setTargetLaneId(best_frenet_path_local->lane_id);
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

  // align ego heading w.r.t lane heading
  if (lane_id != -1) {
    const auto& wp = lanes[lane_id].GetWayPoints();
    double lane_heading =
        utils::wrap_angle(std::atan2(wp[1][1] - wp[1][0], wp[0][1] - wp[0][0]));
    ego_car.setPose({ego_car_pose.x, ego_car_pose.y, lane_heading});
  }

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

/**
 * @brief Estimate the change of pose based on speed and yaw rate
 *
 * @param speed_meas
 * @param yaw_rate_meas
 * @return Pose
 */
Pose EstimateChangeOfPose(double speed_meas, double yaw_rate_meas) {
  double delta_x = 0.0;
  double delta_y = 0.0;
  double delta_yaw = 0.0;

  const auto& fot_hp = FrenetHyperparameters::getConstInstance();
  constexpr double kSimuStep = 0.002;  // [s]
  const int kNumSteps = static_cast<int>(fot_hp.dt / kSimuStep);
  for (int i = 0; i < kNumSteps; ++i) {
    delta_x += speed_meas * std::cos(delta_yaw) * kSimuStep;
    delta_y += speed_meas * std::sin(delta_yaw) * kSimuStep;
    delta_yaw += yaw_rate_meas * kSimuStep;
  }
  return {delta_x, delta_y, delta_yaw};
}

void UpdateSensorMeasurements(const Car& ego_car, const Car& next_ego_car,
                              double* speed_meas, double* yaw_rate_meas) {
  const auto& fot_hp = FrenetHyperparameters::getConstInstance();
  const double TimeStep = fot_hp.dt;
  double position_change =
      utils::norm(next_ego_car.getPose().x - ego_car.getPose().x,
                  next_ego_car.getPose().y - ego_car.getPose().y);
  double yaw_change =
      utils::wrap_angle(next_ego_car.getPose().yaw - ego_car.getPose().yaw);

  *speed_meas = position_change / TimeStep;
  *yaw_rate_meas = yaw_change / TimeStep;

  // add offset and noise
  *speed_meas += utils::genGaussianNoise(
      fot_hp.sensor_speed_offset, fot_hp.sensor_speed_noise_std);  // [m/s]
  *yaw_rate_meas +=
      utils::genGaussianNoise(fot_hp.sensor_yaw_rate_offset,
                              fot_hp.sensor_yaw_rate_noise_std);  // [rad/s]
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
  int sim_loop = 50;
  double total_runtime = 0.0;  // [ms]
  double timestamp = 0.0;      // [s], simulation timestamp
  int i = 0;
  std::vector<DataFrame> data_frames;
  bool reach_goal = false;
  std::vector<FrenetPath>
      best_frenet_paths_local;  // store the best frenet path for each lane
  std::vector<std::vector<FrenetPath>> frenet_paths_local_all;

  Car planning_init_point_local = ego_car;
  if (FLAGS_local_planning) {
    planning_init_point_local.setPose({0, 0, 0});
  }

  double yaw_rate_meas = 0.0;
  double speed_meas = 0.0;
  unordered_map<int, WayPoints> wp_lanes_local;
  std::vector<Obstacle> obstacles_local;

  for (; i < sim_loop; ++i) {
    auto start = std::chrono::high_resolution_clock::now();

    // Loop each lane here and may initialize fot_ic for each lane
    best_frenet_paths_local.clear();
    frenet_paths_local_all.clear();
    frenet_paths_local_all.resize(lanes.size());
    for (const auto& lane : lanes) {
      WayPoints wp = lane.GetWayPoints();

      // break if near goal
      if (utils::norm(ego_car.getPose().x - wp[0].back(),
                      ego_car.getPose().y - wp[1].back()) < 3.0) {
        reach_goal = true;
        break;
      }

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

      // convert obstacles to local coordinate w.r.t. ego car
      obstacles_local.clear();
      WayPoints wp_local;
      if (FLAGS_local_planning) {
        ToLocal(obstacles, ego_car.getPose(), &obstacles_local);
        ToLocal(wp, ego_car.getPose(), &wp_local);
      } else {
        obstacles_local = obstacles;
        wp_local = wp;
      }
      wp_lanes_local[lane.GetLaneId()] = wp_local;

      // run frenet optimal trajectory
      FrenetInitialConditions fot_ic(wp_local, obstacles_local);
      fot_ic.target_speed = target_speed;

      // update Frenet coordinate of ego car
      UpdateFrenetCoordinates(planning_init_point_local, wp_local, &fot_ic);

      // local planning w.r.t. ego car
      FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(fot_ic, fot_hp);
      FrenetPath* best_frenet_path_per_lane = fot.getBestPath();
      if (!best_frenet_path_per_lane || best_frenet_path_per_lane->x.empty()) {
        cerr << "Fail to find a feasible path at timestamp: " << timestamp
             << ", at lane: " << lane.GetLaneId() << endl;
      } else {
        // update cost for each frenet path based on lane
        best_frenet_path_per_lane->lane_id = lane.GetLaneId();
        best_frenet_path_per_lane->c_lane_change =
            std::abs(best_frenet_path_per_lane->lane_id -
                     planning_init_point_local.getTargetLaneId());
        best_frenet_path_per_lane->cf +=
            fot_hp.klane * best_frenet_path_per_lane->c_lane_change;

        best_frenet_paths_local.push_back(*best_frenet_path_per_lane);
      }

      // store all frenet paths for each lane
      map<double, FrenetPath*> d_to_best_path_local;
      for (auto fp : fot.getFrenetPaths()) {
        double d = round_to_tenth(fp->d.back());
        if (d_to_best_path_local.count(d) == 0) {
          d_to_best_path_local[d] = fp;
        } else {
          if (fp->cf < d_to_best_path_local[d]->cf) {
            d_to_best_path_local[d] = fp;
          }
        }
      }

      for (const auto& d_fp : d_to_best_path_local) {
        d_fp.second->lane_id = lane.GetLaneId();
        frenet_paths_local_all[lane.GetLaneId()].push_back(*(d_fp.second));
      }
    }
    if (reach_goal) {
      break;
    }

    // choose from best trajectory along each lane based on cost
    FrenetPath* best_frenet_path_local = nullptr;
    for (auto& fp : best_frenet_paths_local) {
      if (!best_frenet_path_local) {
        best_frenet_path_local = &fp;
      } else {
        if (fp.cf < best_frenet_path_local->cf) {
          best_frenet_path_local = &fp;
        }
      }
    }

    auto plan_end = std::chrono::high_resolution_clock::now();
    double plan_duration = get_duration_ms(plan_end, start);

    // convert (x,y,yaw) of paths to global for debug purposes
    std::vector<FrenetPath> best_frenet_paths_global = best_frenet_paths_local;
    if (FLAGS_local_planning) {
      for (auto& fp : best_frenet_paths_global) {
        std::size_t fp_size = fp.x.size();
        for (std::size_t i = 0; i < fp_size; ++i) {
          Pose pose_l({fp.x[i], fp.y[i], fp.yaw[i]});
          Pose pose_g;
          ToGlobal(pose_l, ego_car.getPose(), &pose_g);
          fp.x[i] = pose_g.x;
          fp.y[i] = pose_g.y;
          fp.yaw[i] = pose_g.yaw;
        }
      }
    }

    FrenetPath* best_frenet_path_g = nullptr;
    for (auto& fp : best_frenet_paths_global) {
      if (!best_frenet_path_g) {
        best_frenet_path_g = &fp;
      } else {
        if (fp.cf < best_frenet_path_g->cf) {
          best_frenet_path_g = &fp;
        }
      }
    }

    // save current frame data
    if (FLAGS_store_data) {
      DataFrame df;
      df.timestamp = timestamp;
      df.ego_car = ego_car;
      df.best_frenet_path = *best_frenet_path_g;
      df.lanes = lanes;
      df.obstacles = obstacles;
      df.frenet_paths = best_frenet_paths_global;
      df.obstacles_local = obstacles_local;
      df.wp_lanes_local = wp_lanes_local;
      df.planning_init_point_local = planning_init_point_local;
      df.best_frenet_path_local = *best_frenet_path_local;
      df.frenet_paths_local = best_frenet_paths_local;
      df.frenet_paths_local_all = frenet_paths_local_all;
      data_frames.push_back(std::move(df));
    }

    // update
    timestamp += TimeStep;
    // update obstacle to next state
    for (Obstacle& ob : obstacles) {
      Pose ob_pose_next = ob.getPredictPoseAtTimestamp(timestamp);
      ob.setPose(ob_pose_next);
    }

    // update next planning state w.r.t. local frame
    Car next_planning_state_local = planning_init_point_local;
    UpdateNextPlanningStateLocal(
        best_frenet_path_local, wp_lanes_local[best_frenet_path_local->lane_id],
        &next_planning_state_local);

    // next frame
    Car next_ego_car =
        next_planning_state_local;  // set to next_planning_state_global
    if (FLAGS_local_planning) {
      ToGlobal(next_planning_state_local, ego_car.getPose(), &next_ego_car);
    }
    UpdateSensorMeasurements(ego_car, next_ego_car, &speed_meas,
                             &yaw_rate_meas);
    ego_car = next_ego_car;

    // update planning init point
    Pose pose_change_est = EstimateChangeOfPose(
        speed_meas, yaw_rate_meas);  // estimation of state change
    Car planning_init_point_wrt_last_frame = next_planning_state_local;
    // TODO: update Initial Planning Point in local frame
    planning_init_point_local = planning_init_point_wrt_last_frame;
    if (FLAGS_local_planning) {
      planning_init_point_local.setPose(
          planning_init_point_wrt_last_frame.getPose() - pose_change_est);
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