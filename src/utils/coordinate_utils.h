#ifndef COORDINATE_UTILS_H
#define COORDINATE_UTILS_H

#include <iostream>
#include <memory>

#include "Car/Car.h"
#include "CubicSpline/CubicSpline2D.h"
#include "Obstacle/Obstacle.h"
#include "utils/utils.h"

namespace utils {

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

}  // namespace utils
#endif
