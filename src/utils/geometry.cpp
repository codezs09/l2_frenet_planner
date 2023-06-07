#include "geometry.h"

namespace utils {

double Box::DistanceTo(const Box& other) { return distance(*this, other); }

vector<Line> Box::getEdges() {
  vector<Line> edges;
  for (int i = 0; i < 4; ++i) {
    edges.push_back(Line(corners[i], corners[(i + 1) % 4]));
  }
  return edges;
}

bool is_collision(const Box& box_a, const Box& box_b) {
  // using projection method, i.e. Separating Axis Theorem
  std::vector<Line> axes;
  for (int i = 0; i < 2; ++i) {
    axes.push_back(Line(box_a.corners[i], box_a.corners[(i + 1) % 4]));
    axes.push_back(Line(box_b.corners[i], box_b.corners[(i + 1) % 4]));
  }

  for (auto& a : axes) {
    vector<double, double> a_unit_vec = {(a.end.x - a.start.x) / a.Length(),
                                         (a.end.y - a.start.y) / a.Length()};

    double min1 = std::numeric_limits<double>::max();
    double max1 = std::numeric_limits<double>::min();
    for (auto& c : box_a.corners) {
      double projection = dot({c.x, c.y}, a_unit_vec);
      min1 = std::min(min1, projection);
      max1 = std::max(max1, projection);
    }
    double min2 = std::numeric_limits<double>::max();
    double max2 = std::numeric_limits<double>::min();
    for (auto& c : box_b.corners) {
      double projection = dot({c.x, c.y}, a_unit_vec);
      min2 = std::min(min2, projection);
      max2 = std::max(max2, projection);
    }

    if (max1 < min2 || max2 < min1) {
      return false;  // if there is a gap, return no collision
    }
    return true;  // all projections overlap
  }
}

double distance(const Point& point_a, const Point& point_b) {
  return norm(point_a.x - point_b.x, point_a.y - point_b.y);
}

double distance(const Point& point, const Line& line) {
  // using projection method
  double line_length = line.Length();
  vector<double, double> line_unit_vec = {
      (line.end.x - line.start.x) / line_length,
      (line.end.y - line.start.y) / line_length};
  vector<double, double> sp_vec = {
      point.x - line.start.x, point.y - line.start.y};  // line start to point

  double projection = dot(sp_vec, line_unit_vec);
  if (projection < 0) {
    return distance(point, line.start);
  } else if (projection > line_length) {
    return distance(point, line.end);
  } else {
    return std::fabs(cross_product(sp_vec, line_unit_vec));
  }
}

double distance(const Box& box_a, const Box& box_b) {
  // The idea is to check the distance of each corner of box_a to box_b,
  // and vice versa.
  if (is_collision(box_a, box_b)) {
    return 0;
  }
  double min_distance = std::numeric_limits<double>::max();
  for (const Point& c : box_a.corners) {
    for (auto edge : box_b.getEdges()) {
      min_distance = std::min(min_distance, distance(c, edge));
    }
  }
  for (const Point& c : box_b.corners) {
    for (auto edge : box_a.getEdges()) {
      min_distance = std::min(min_distance, distance(c, edge));
    }
  }
  return min_distance;
}

}  // namespace utils