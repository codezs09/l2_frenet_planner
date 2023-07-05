#include "coordinate_utils.h"

#include "utils.h"

using namespace utils;

int main(int argc, char** argv) {
  Pose p_global = {54.2365, 134.57, 3.12414};

  Pose p_ref = {13.642, 153.0, 1.234};

  cout << "Raw Global: " << p_global.x << ", " << p_global.y << ", "
       << p_global.yaw << endl;

  Pose p_local;
  ToLocal(p_global, p_ref, &p_local);

  cout << "Local: " << p_local.x << ", " << p_local.y << ", " << p_local.yaw
       << endl;

  Pose p_global2;
  ToGlobal(p_local, p_ref, &p_global2);

  cout << "Global: " << p_global2.x << ", " << p_global2.y << ", "
       << p_global2.yaw << endl;

  return 0;
}