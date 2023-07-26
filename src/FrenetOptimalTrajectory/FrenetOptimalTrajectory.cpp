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
  double t, ti, tv;
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
      t = 0;
      while (t <= ti) {
        fp->t.push_back(t);
        fp->d.push_back(lat_qp.calc_point(t));
        fp->d_d.push_back(lat_qp.calc_first_derivative(t));
        fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
        fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
        lateral_deviation += abs(lat_qp.calc_point(t));
        lateral_velocity += abs(lat_qp.calc_first_derivative(t));
        lateral_acceleration += abs(lat_qp.calc_second_derivative(t));
        lateral_jerk += abs(lat_qp.calc_third_derivative(t));
        t += fot_hp.dt;
      }

      // velocity keeping (only support velocity keeping mode)
      tv = fot_ic.target_speed -
           fot_hp.d_t_s * fot_hp.n_s_sample;  // tv: sampling speed
      while (tv <= fot_ic.target_speed + fot_hp.d_t_s * fot_hp.n_s_sample) {
        longitudinal_acceleration = 0;
        longitudinal_jerk = 0;

        // copy frenet path
        tfp = new FrenetPath();
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
          tfp->s.push_back(lon_qp.calc_point(tp));
          tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));
          tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp));
          tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp));
          longitudinal_acceleration += abs(lon_qp.calc_second_derivative(
              tp));  // if accumulated, then not fair under different ti
                     // sampling.
          longitudinal_jerk += abs(lon_qp.calc_third_derivative(tp));
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
                              fot_hp.kd * tfp->c_end_speed_deviation;

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
