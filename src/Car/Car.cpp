#include "Car.h"

// // Compute the outline of the car given its current pose
// bool Car::getOutline(vector<Point>* outline) {
//   double x = pose.x;
//   double y = pose.y;
//   double yaw = pose.yaw;

//   double tail_x, tail_y, head_x, head_y;
//   vector<double> tail_l, tail_r;
//   vector<double> head_l, head_r;

//   tail_x = x - cos(yaw) * length * 0.5;
//   tail_y = y - sin(yaw) * length * 0.5;
//   tail_l.push_back(tail_x + cos(yaw + M_PI_2) * width / 2.0);
//   tail_l.push_back(tail_y + sin(yaw + M_PI_2) * width / 2.0);
//   tail_r.push_back(tail_x + cos(yaw - M_PI_2) * width / 2.0);
//   tail_r.push_back(tail_y + sin(yaw - M_PI_2) * width / 2.0);

//   head_x = x + cos(yaw) * length * 0.5;
//   head_y = y + sin(yaw) * length * 0.5;
//   head_l.push_back(head_x + cos(yaw + M_PI_2) * width / 2.0);
//   head_l.push_back(head_y + sin(yaw + M_PI_2) * width / 2.0);
//   head_r.push_back(head_x + cos(yaw - M_PI_2) * width / 2.0);
//   head_r.push_back(head_y + sin(yaw - M_PI_2) * width / 2.0);

//   outline->push_back(tail_l);
//   outline->push_back(tail_r);
//   outline->push_back(head_r);
//   outline->push_back(head_l);
//   outline->push_back(tail_l);
//   return true;
// }