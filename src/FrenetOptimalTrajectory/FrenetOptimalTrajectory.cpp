#include "FrenetOptimalTrajectory.h"

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "utils/utils.h"

using namespace std;
using namespace utils;

// Compute the frenet optimal trajectory
FrenetOptimalTrajectory::FrenetOptimalTrajectory(
    const FrenetInitialConditions &fot_ic_,
    const FrenetHyperparameters &fot_hp_)
    : fot_ic(fot_ic_), fot_hp(fot_hp_) {
  auto start = chrono::high_resolution_clock::now();
  // parse the waypoints and obstacles
  mu = new mutex();

  // make sure best_frenet_path is initialized
  best_frenet_path = nullptr;

  // exit if not enough waypoints
  if (fot_ic.wp.empty() || fot_ic.wp[0].size() < 2) {
    return;
  }

  // construct spline path
  csp = new CubicSpline2D(fot_ic.wp[0], fot_ic.wp[1]);

  // calculate the trajectories
  if (fot_hp.num_threads == 0) {
    // calculate how to split computation across threads

    int total_di_iter = static_cast<int>(fot_ic.lane_width / fot_hp.d_road_w) +
                        1;  // account for the last index

    calc_frenet_paths(0, total_di_iter, false);

  } else {  // if threading
    threaded_calc_all_frenet_paths();
  }

  // select the best path
  double mincost = INFINITY;
  for (FrenetPath *fp : frenet_paths) {
    if (fp->cf <= mincost) {
      mincost = fp->cf;
      best_frenet_path = fp;
    }
  }
  auto end = chrono::high_resolution_clock::now();
  double run_time =
      chrono::duration_cast<chrono::nanoseconds>(end - start).count();
  run_time *= 1e-6;
  // cout << "Planning runtime " << run_time << "\n";
}

FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {
  delete mu;
  delete csp;
  for (FrenetPath *fp : frenet_paths) {
    delete fp;
  }
}

// Return the best path
FrenetPath *FrenetOptimalTrajectory::getBestPath() { return best_frenet_path; }

/*
 * Spawn threads to calculate frenet paths
 * Called when multithreading.
 */
void FrenetOptimalTrajectory::threaded_calc_all_frenet_paths() {
  vector<thread> threads;

  // calculate how to split computation across threads
  int num_di_iter = static_cast<int>(fot_ic.lane_width / fot_hp.d_road_w);
  num_di_iter = num_di_iter + 1;  // account for the last index

  int iter_di_index_range = static_cast<int>(num_di_iter / fot_hp.num_threads);

  for (int i = 0; i < fot_hp.num_threads; i++) {
    if (i != fot_hp.num_threads - 1) {
      threads.push_back(thread(&FrenetOptimalTrajectory::calc_frenet_paths,
                               this, i * iter_di_index_range,
                               (i + 1) * iter_di_index_range, true));
    } else {  // account for last thread edge case
      threads.push_back(thread(&FrenetOptimalTrajectory::calc_frenet_paths,
                               this, i * iter_di_index_range, num_di_iter,
                               true));
    }
  }

  // wait for all threads to finish computation
  for (auto &t : threads) {
    t.join();
  }
}

/*
 * Calculate frenet paths
 * If running we are multithreading,
 * We parallelize on the outer loop, in terms of di
 * Iterates over possible values of di, from start index to end index
 * (exclusive). Then, computes the actual di value for path planning.
 * Mutex is only enabled when we are multithreading.
 */
void FrenetOptimalTrajectory::calc_frenet_paths(int start_di_index,
                                                int end_di_index,
                                                bool multithreaded) {
  constexpr double kHorizon = 5.0;
  double ti, tv;
  double lateral_deviation, lateral_velocity, lateral_acceleration,
      lateral_jerk;
  double longitudinal_acceleration, longitudinal_jerk;
  FrenetPath *fp, *tfp;
  int num_paths = 0;
  int num_viable_paths = 0;
  // double valid_path_time = 0;

  // initialize di, with start_di_index
  double half_lane_width = fot_ic.lane_width / 2.0;
  double di = -half_lane_width + start_di_index * fot_hp.d_road_w;

  // generate path to each offset goal
  // note di goes up to but not including end_di_index*fot_hp.d_road_w
  while ((di < -half_lane_width + end_di_index * fot_hp.d_road_w) &&
         (di <= half_lane_width)) {
    ti = fot_hp.mint;
    // lateral motion planning
    while (ti <= fot_hp.maxt) {
      lateral_deviation = 0;
      lateral_velocity = 0;
      lateral_acceleration = 0;
      lateral_jerk = 0;

      fp = new FrenetPath();
      QuinticPolynomial lat_qp = QuinticPolynomial(
          fot_ic.d, fot_ic.d_d, fot_ic.d_dd, di, 0.0, 0.0, ti);

      // construct frenet path
      double t = 0;
      while (t <= kHorizon) {
        fp->t.push_back(t);
        if (t <= ti) {
          fp->d.push_back(lat_qp.calc_point(t));
          fp->d_d.push_back(lat_qp.calc_first_derivative(t));
          fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
          fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
          lateral_deviation += abs(lat_qp.calc_point(t));
          lateral_velocity += abs(lat_qp.calc_first_derivative(t));
          lateral_acceleration += abs(lat_qp.calc_second_derivative(t));
          lateral_jerk += abs(lat_qp.calc_third_derivative(t));
        } else {
          fp->d.push_back(di);  // end point
          fp->d_d.push_back(0.0);
          fp->d_dd.push_back(0.0);
          fp->d_ddd.push_back(0.0);
          lateral_deviation += abs(di);
          lateral_velocity += abs(0);
          lateral_acceleration += abs(0);
        }
        t += fot_hp.dt;
      }

      // Lonitudinal motion planning: velocity keeping mode
      tv = fot_ic.target_speed -
           fot_hp.d_t_s * fot_hp.n_s_sample;  // tv: sampling speed
      while (tv <= fot_ic.target_speed + fot_hp.d_t_s * fot_hp.n_s_sample) {
        longitudinal_acceleration = 0;
        longitudinal_jerk = 0;

        // copy frenet path
        tfp = new FrenetPath();
        tfp->set_lon_mode(utils::LonMotionMode::VelocityKeeping);
        tfp->t.assign(fp->t.begin(), fp->t.end());
        tfp->d.assign(fp->d.begin(), fp->d.end());
        tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
        tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
        tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());
        QuarticPolynomial lon_qp = QuarticPolynomial(
            fot_ic.s, fot_ic.s_d, fot_ic.s_dd, tv, 0.0,
            ti);  // 1. ic and ec lon_acc all zeros? 2. sampling time is same
                  // for both lat and lon?

        // longitudinal motion
        for (double tp : tfp->t) {
          if (tp <= ti) {
            tfp->s.push_back(lon_qp.calc_point(tp));
            tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));
            tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp));
            tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp));
            longitudinal_acceleration += abs(lon_qp.calc_second_derivative(tp));
            longitudinal_jerk += abs(lon_qp.calc_third_derivative(tp));
          } else {
            tfp->s.push_back(tfp->s.back() + tv * fot_hp.dt);
            tfp->s_d.push_back(tv);
            tfp->s_dd.push_back(0.0);
            tfp->s_ddd.push_back(0.0);
            longitudinal_acceleration += abs(0);
            longitudinal_jerk += abs(0);
          }
        }

        num_paths++;
        // delete if failure or invalid path
        bool success = tfp->to_global_path(csp);
        num_viable_paths++;
        if (!success) {
          // deallocate memory and continue
          delete tfp;
          tv += fot_hp.d_t_s;
          continue;
        }

        // auto start = chrono::high_resolution_clock::now();
        bool valid_path = tfp->is_valid_path(fot_ic.obstacles_c);
        // auto end = chrono::high_resolution_clock::now();
        // valid_path_time +=
        // chrono::duration_cast<chrono::nanoseconds>(end -
        // start).count();
        if (!valid_path) {
          // deallocate memory and continue
          delete tfp;
          tv += fot_hp.d_t_s;
          continue;
        }

        // lateral costs
        tfp->c_lateral_deviation = lateral_deviation;
        tfp->c_lateral_velocity = lateral_velocity;
        tfp->c_lateral_acceleration = lateral_acceleration;
        tfp->c_lateral_jerk = lateral_jerk;
        tfp->c_lateral = fot_hp.kd * tfp->c_lateral_deviation +
                         fot_hp.kv * tfp->c_lateral_velocity +
                         fot_hp.ka * tfp->c_lateral_acceleration +
                         fot_hp.kj * tfp->c_lateral_jerk;

        // longitudinal costs
        tfp->c_longitudinal_acceleration = longitudinal_acceleration;
        tfp->c_longitudinal_jerk = longitudinal_jerk;
        tfp->c_end_speed_deviation = abs(fot_ic.target_speed - tfp->s_d.back());
        tfp->c_time_taken = ti;
        tfp->c_longitudinal = fot_hp.ka * tfp->c_longitudinal_acceleration +
                              fot_hp.kj * tfp->c_longitudinal_jerk +
                              fot_hp.kt * tfp->c_time_taken +
                              fot_hp.k_ev * tfp->c_end_speed_deviation;

        // obstacle costs
        tfp->c_inv_dist_to_obstacles =
            tfp->inverse_distance_to_obstacles(fot_ic.obstacles_c);

        // final cost
        tfp->cf = fot_hp.klat * tfp->c_lateral +
                  fot_hp.klon * tfp->c_longitudinal +
                  fot_hp.ko * tfp->c_inv_dist_to_obstacles;

        if (multithreaded) {
          // added mutex lock to prevent threads competing to write to
          // frenet_path
          mu->lock();
          frenet_paths.push_back(tfp);
          mu->unlock();
        } else {
          frenet_paths.push_back(tfp);
        }

        tv += fot_hp.d_t_s;
      }

      // Lonitudinal motion planning: following/stopping mode
      double target_s_flw;
      std::vector<double> s_flw_vec;
      if (has_near_obstacle_front(&target_s_flw, &s_flw_vec)) {
        for (double s_flw : s_flw_vec) {
          // if (s_flw < fot_ic.s) {
          //   continue;
          // }
          longitudinal_acceleration = 0;
          longitudinal_jerk = 0;

          // copy frenet path
          tfp = new FrenetPath();
          tfp->set_lon_mode(utils::LonMotionMode::Following);
          tfp->t.assign(fp->t.begin(), fp->t.end());
          tfp->d.assign(fp->d.begin(), fp->d.end());
          tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
          tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
          tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());
          QuinticPolynomial lon_qp_flw = QuinticPolynomial(
              fot_ic.s, fot_ic.s_d, fot_ic.s_dd, s_flw, 0.0, 0.0, ti);

          for (double tp : tfp->t) {
            if (tp <= ti) {
              tfp->s.push_back(lon_qp_flw.calc_point(tp));
              tfp->s_d.push_back(lon_qp_flw.calc_first_derivative(tp));
              tfp->s_dd.push_back(lon_qp_flw.calc_second_derivative(tp));
              tfp->s_ddd.push_back(lon_qp_flw.calc_third_derivative(tp));
              longitudinal_acceleration +=
                  abs(lon_qp_flw.calc_second_derivative(tp));
              longitudinal_jerk += abs(lon_qp_flw.calc_third_derivative(tp));
            } else {
              tfp->s.push_back(s_flw);
              tfp->s_d.push_back(0.0);
              tfp->s_dd.push_back(0.0);
              tfp->s_ddd.push_back(0.0);
              longitudinal_acceleration += 0.0;
              longitudinal_jerk += 0.0;
            }
          }

          num_paths++;
          // delete if failure or invalid path
          bool success = tfp->to_global_path(csp);
          num_viable_paths++;
          if (!success) {
            // deallocate memory and continue
            delete tfp;
            continue;
          }

          bool valid_path = tfp->is_valid_path(fot_ic.obstacles_c);
          if (!valid_path) {
            delete tfp;
            continue;
          }

          // lateral costs
          tfp->c_lateral_deviation = lateral_deviation;
          tfp->c_lateral_velocity = lateral_velocity;
          tfp->c_lateral_acceleration = lateral_acceleration;
          tfp->c_lateral_jerk = lateral_jerk;
          tfp->c_lateral = fot_hp.kd * tfp->c_lateral_deviation +
                           fot_hp.kv * tfp->c_lateral_velocity +
                           fot_hp.ka * tfp->c_lateral_acceleration +
                           fot_hp.kj * tfp->c_lateral_jerk;

          // longitudinal costs
          tfp->c_longitudinal_acceleration = longitudinal_acceleration;
          tfp->c_longitudinal_jerk = longitudinal_jerk;
          tfp->c_end_s_deviation = abs(target_s_flw - tfp->s.back());

          tfp->c_time_taken = ti;
          tfp->c_longitudinal = fot_hp.ka * tfp->c_longitudinal_acceleration +
                                fot_hp.kj * tfp->c_longitudinal_jerk +
                                fot_hp.kt * tfp->c_time_taken +
                                fot_hp.k_es * tfp->c_end_s_deviation;

          // obstacle costs
          tfp->c_inv_dist_to_obstacles =
              tfp->inverse_distance_to_obstacles(fot_ic.obstacles_c);

          // final cost
          tfp->cf = fot_hp.klat * tfp->c_lateral +
                    fot_hp.klon * tfp->c_longitudinal +
                    fot_hp.ko * tfp->c_inv_dist_to_obstacles;

          if (multithreaded) {
            // added mutex lock to prevent threads competing to write to
            // frenet_path
            mu->lock();
            frenet_paths.push_back(tfp);
            mu->unlock();
          } else {
            frenet_paths.push_back(tfp);
          }
        }
      }

      ti += fot_hp.dt;
      // make sure to deallocate
      delete fp;
    }
    di += fot_hp.d_road_w;
  }
  // valid_path_time *= 1e-6;
  // cout << "NUM THREADS = " << fot_hp.num_threads << "\n"; // check if
  // Thread argument is passed down cout << "Found " << frenet_paths.size() <<
  // " valid paths out of " << num_paths << " paths; Valid path time " <<
  // valid_path_time << "\n";
}

bool FrenetOptimalTrajectory::has_near_obstacle_front(
    double *target_s_flw, vector<double> *s_flw_vec) {
  // si range [2.0:0.5:4.0] * max(v_front_car, 1.0) cost
  int idx_front_obstacle = -1;
  double min_s_front_obstacle = 1e9;
  for (int i = 0; i < fot_ic.obstacles_c.size(); i++) {
    const auto &ob_c = fot_ic.obstacles_c[i];
    if (ob_c.isInLane(fot_ic.lane_id)) {
      continue;  // not same lane, skip
    }
    // convert ob_c to frenet frame
    std::unique_ptr<Obstacle> ob_f = nullptr;
    utils::ToFrenet(ob_c, fot_ic.wp,
                    ob_f);  // local planning,
                            // 下这里得到的s可能是错的。可能要用别的办法。
    if (ob_f->getPose().x < fot_ic.s) {
      continue;  // behind ego car, skip
    }

    constexpr double kTimeGap = 20.0;
    double dist_threshold = kTimeGap * max(ob_f->getTwist().vx, fot_ic.s_d);
    if (ob_f->getPose().x - fot_ic.s < dist_threshold) {
      min_s_front_obstacle = ob_f->getPose().x;
      idx_front_obstacle = i;
    }
  }
  if (idx_front_obstacle == -1) {
    return false;
  }
  const auto &ob_c = fot_ic.obstacles_c[idx_front_obstacle];

  double time_gap_target = 3.0;  // 3-second driving rule
  double time_gap_lo = 2.0;
  double time_gap_hi = 4.0;
  double v_front_capped = max(ob_c.getTwist().vx, 1.0);
  *target_s_flw = min_s_front_obstacle - time_gap_target * v_front_capped;

  double t = time_gap_lo;
  while (t <= time_gap_hi) {
    double s_flw = min_s_front_obstacle - t * v_front_capped;
    if (s_flw > fot_ic.s) {
      s_flw_vec->push_back(s_flw);
    }
    t += fot_hp.d_t_s;
  }
  if (s_flw_vec->empty() || *target_s_flw < fot_ic.s) {
    s_flw_vec->push_back(fot_ic.s + 1e-6);
  }
  return true;
}
