#ifndef FRENET_OPTIMAL_TRAJECTORY_UTILS_H
#define FRENET_OPTIMAL_TRAJECTORY_UTILS_H

#include <cmath>
#include <nlohmann/json.hpp>
#include <tuple>
#include <vector>

using namespace std;
using json = nlohmann::json;

namespace utils {

typedef vector<vector<double>> WayPoints;

struct Pose {
  double x;    // [m]
  double y;    // [m]
  double yaw;  // [rad]
};
struct Twist {
  double vx;
  double vy;
  double yaw_rate;
};
struct Accel {
  double ax;
  double ay;
  double yaw_accel;
};

enum class CoordinateType {
  CARTESIAN = 0,  //
  FRENET = 1
};

inline double norm(double x, double y) { return sqrt(pow(x, 2) + pow(y, 2)); }

inline void as_unit_vector(tuple<double, double>& vec) {
  double magnitude = norm(get<0>(vec), get<1>(vec));
  if (magnitude > 0) {
    get<0>(vec) = get<0>(vec) / magnitude;
    get<1>(vec) = get<1>(vec) / magnitude;
  }
}

inline double dot(const tuple<double, double>& vec1,
                  const tuple<double, double>& vec2) {
  return get<0>(vec1) * get<0>(vec2) + get<1>(vec1) * get<1>(vec2);
}

inline double cross_prodcut(const tuple<double, double>& vec1,
                            const tuple<double, double>& vec2) {
  return get<0>(vec1) * get<1>(vec2) - get<1>(vec1) * get<0>(vec2);
}

bool LoadJsonFile(string scene_path, json* j);

}  // namespace utils

#endif  // FRENET_OPTIMAL_TRAJECTORY_UTILS_H
