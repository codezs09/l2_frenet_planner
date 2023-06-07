#ifndef UTILS_GEOMETRY_H
#define UTILS_GEOMETRY_H

#include <array>
#include <cmath>
#include <vector>

#include "utils/utils.h"

using namespace std;

namespace utils {

class Point {
 public:
  Point(double x = 0, double y = 0) : x(x), y(y) {}
  double x;  // [m]
  double y;  // [m]
};

class Line {
 public:
  Point start;
  Point end;
  Line(const Point& start, const Point& end) : start(start), end(end) {}
  double Length() { return distance(start, end); }
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
  Box(const Corners& corners) : corners(corners) {}
  Corners corners;

  vector<Line> getEdges();
  double DistanceTo(const Box& other);
};

bool is_collision(const Box& box_a, const Box& box_b);

double distance(const Point& point_a, const Point& point_b);
double distance(const Point& point, const Line& line);
double distance(const Box& box_a, const Box& box_b);

}  // namespace utils

#endif  // UTILS_GEOMETRY_H