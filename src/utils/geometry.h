#ifndef UTILS_GEOMETRY_H
#define UTILS_GEOMETRY_H

#include <algorithm>
#include <array>
#include <cmath>
#include <msgpack.hpp>
#include <utility>
#include <vector>

#include "Lane/Lane.h"
#include "utils/utils.h"

using namespace std;

namespace utils {

class Point {
 public:
  Point(double x = 0, double y = 0) : x(x), y(y) {}
  double x;  // [m]
  double y;  // [m]

  MSGPACK_DEFINE(x, y);
};

class Line {
 public:
  Point start;
  Point end;
  Line(const Point& start, const Point& end) : start(start), end(end) {}
  double Length() const;
};

using Corners = std::array<Point, 4>;

/**
 * @brief A box is a rectangular shape in 2D plane defined by its corners.
 * Note that the box is not necessarily aligned with the x and y axes;
 * and its corners should be defined in sequence, either clockwise or
 * anti-clockwise.
 */
class Box {
 public:
  Box() = default;
  Box(const Corners& corners) : corners(corners) {}
  Corners corners;

  vector<Line> getEdges() const;
  double DistanceTo(const Box& other) const;

  MSGPACK_DEFINE(corners);
};

bool is_collision(const Box& box_a, const Box& box_b);

double distance(const Point& point_a, const Point& point_b);
double distance(const Point& point, const Line& line);
double distance(const Box& box_a, const Box& box_b);

/**
 * @brief Rotate a point of a given angle (positive in anti-clockwise
 * direction).
 *
 * @param point
 * @param angle [rad]
 * @return Point
 */
Point rotate(const Point& point, double angle);

/**
 * @brief Convert a pose to a box.
 * Asumming the pose is at the tail of the box.
 * @param pose
 * @param length
 * @param width
 * @param clearance
 * @return Box
 */
Box pose_to_box(const Pose& pose, double length, double width,
                double clearance = 0);

/**
 * @brief Judge if a point is in a lane confined by its boundaries.
 *
 * @param lane
 * @param x
 * @param y
 * @return true
 * @return false
 */
bool point_in_lane(const Lane& lane, double x, double y);

}  // namespace utils

#endif  // UTILS_GEOMETRY_H