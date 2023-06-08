#include "geometry.h"

namespace utils {

double Line::Length() const { return distance(start, end); }

double Box::DistanceTo(const Box& other) const {
  return distance(*this, other);
}

vector<Line> Box::getEdges() const {
  vector<Line> edges;
  for (int i = 0; i < 4; ++i) {
    edges.push_back(Line(corners[i], corners[(i + 1) % 4]));
  }
  return edges;
}

bool is_collision(const Box& box_a, const Box& box_b) {
  // quick check to avoid unnecessary computation below
  auto get_range_x = [](Box b) -> std::pair<double, double> {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    for (auto& c : b.corners) {
      min_x = std::min(min_x, c.x);
      max_x = std::max(max_x, c.x);
    }
    return {min_x, max_x};
  };
  auto range_x_a = get_range_x(box_a);
  auto range_x_b = get_range_x(box_b);
  if (range_x_a.first > range_x_b.second ||
      range_x_b.first > range_x_a.second) {
    return false;  // gap in x direction
  }

  auto get_range_y = [](Box b) -> std::pair<double, double> {
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
    for (auto& c : b.corners) {
      min_y = std::min(min_y, c.y);
      max_y = std::max(max_y, c.y);
    }
    return {min_y, max_y};
  };
  auto range_y_a = get_range_y(box_a);
  auto range_y_b = get_range_y(box_b);
  if (range_y_a.first > range_y_b.second ||
      range_y_b.first > range_y_a.second) {
    return false;  // gap in y direction
  }

  // Otherwise using projection method, i.e. Separating Axis Theorem
  std::vector<Line> axes;
  for (int i = 0; i < 2; ++i) {
    axes.push_back(Line(box_a.corners[i], box_a.corners[(i + 1) % 4]));
    axes.push_back(Line(box_b.corners[i], box_b.corners[(i + 1) % 4]));
  }

  for (auto& a : axes) {
    tuple<double, double> a_unit_vec = {(a.end.x - a.start.x) / a.Length(),
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
  }
  return true;  // all projections overlap
}

double distance(const Point& point_a, const Point& point_b) {
  return norm(point_a.x - point_b.x, point_a.y - point_b.y);
}

double distance(const Point& point, const Line& line) {
  // using projection method
  double line_length = line.Length();
  tuple<double, double> line_unit_vec = {
      (line.end.x - line.start.x) / line_length,
      (line.end.y - line.start.y) / line_length};
  tuple<double, double> sp_vec = {
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

Point rotate(const Point& point, double angle) {
  double x = point.x * std::cos(angle) - point.y * std::sin(angle);
  double y = point.x * std::sin(angle) + point.y * std::cos(angle);
  return {x, y};
}

Box pose_to_box(const Pose& pose, double length, double width,
                double clearance) {
  Corners corners = {
      Point(length, width / 2),
      Point(length, -width / 2),
      Point(0.0, -width / 2),
      Point(0.0, width / 2),
  };
  for (auto& c : corners) {
    c = rotate(c, pose.yaw);
    c.x += pose.x;
    c.y += pose.y;
  }
  return corners;
}

}  // namespace utils