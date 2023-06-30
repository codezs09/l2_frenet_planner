#ifndef COORDINATE_UTILS_H
#define COORDINATE_UTILS_H

#include <iostream>
#include <memory>

#include "Car/Car.h"
#include "CubicSpline/CubicSpline2D.h"
#include "Obstacle/Obstacle.h"
#include "utils/debug.h"
#include "utils/utils.h"

namespace utils {

/**
 * Utils functions to convert between Cartesian and Frenet coordinates
 */
void ToFrenet(const Pose& pose_c, const Twist& twist_c, const Accel& accel_c,
              WayPoints wp, Pose* pose_f, Twist* twist_f, Accel* accel_f);

void ToCartesian(const Pose& pose_f, const Twist& twist_f, const Accel& accel_f,
                 WayPoints wp, Pose* pose_c, Twist* twist_c, Accel* accel_c);

void ToFrenet(const Car& car_c, const WayPoints& wp, Car* car_f);
void ToCartesian(const Car& car_f, const WayPoints& wp, Car* car_c);

void ToFrenet(const Obstacle& ob_c, const WayPoints& wp,
              std::unique_ptr<Obstacle>& ob_f);
void ToCartesian(const Obstacle& ob_f, const WayPoints& wp,
                 std::unique_ptr<Obstacle>& ob_c);

/**
 * Utils functions to convert between global and local coordinates
 * w.r.t. reference point pose_ref
 */
void ToLocal(const Pose& pose_g, const Pose& pose_ref, Pose* pose_l);
void ToLocal(const Obstacle& ob_g, const Pose& pose_ref, Obstacle* ob_l);
void ToLocal(const vector<Obstacle>& obs_g, const Pose& pose_ref,
             vector<Obstacle>* obs_l);
void ToLocal(const WayPoints& wp_g, const Pose& pose_ref, WayPoints* wp_l);

void ToGlobal(const Pose& pose_l, const Pose& pose_ref, Pose* pose_g);
void ToGlobal(const Car& car_l, const Pose& pose_ref, Car* car_g);

/**
 * @brief Shift waypoints by offset
 *        offset is signed, positive for left side and negative for right side
 * @param ref_wp
 * @param offset
 * @param wp
 */
void ShiftWaypoints(const WayPoints& ref_wp, double offset, WayPoints* wp);

}  // namespace utils
#endif
