#include <gtest/gtest.h>

#include "coordinate_utils.h"
#include "utils.h"

using namespace utils;

TEST(GlobalLocalConversionTest, GlobalToLocalToGlobal) {
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

  EXPECT_DOUBLE_EQ(p_global.x, p_global2.x);
  EXPECT_DOUBLE_EQ(p_global.y, p_global2.y);
  EXPECT_DOUBLE_EQ(p_global.yaw, p_global2.yaw);
}

TEST(CartesianFrenetConversionTest, CartesianToFrenetToCartesian) {
  WayPoints wp;
  wp[0] = {0, 50, 100, 150};
  wp[1] = {0, 10, 10, 20};

  // Cartesian
  Pose pose_c = {64.58, 12.1, 0.174533};
  Twist twist_c = {10, 0.1, 0.2};
  Accel accel_c = {0.5, 0.2, -0.3};

  Pose pose_f;
  Twist twist_f;
  Accel accel_f;

  ToFrenet(pose_c, twist_c, accel_c, wp, &pose_f, &twist_f, &accel_f);

  Pose pose_c2;
  Twist twist_c2;
  Accel accel_c2;

  ToCartesian(pose_f, twist_f, accel_f, wp, &pose_c2, &twist_c2, &accel_c2);

  EXPECT_NEAR(pose_c.x, pose_c2.x, 1.0e-2);
  EXPECT_NEAR(pose_c.y, pose_c2.y, 1.0e-5);
  EXPECT_NEAR(pose_c.yaw, pose_c2.yaw, 1.0e-5);
  EXPECT_NEAR(twist_c.vx, twist_c2.vx, 1.0e-5);
  EXPECT_NEAR(twist_c.vy, twist_c2.vy, 1.0e-5);
  EXPECT_NEAR(twist_c.yaw_rate, twist_c2.yaw_rate, 1.0e-5);
  EXPECT_NEAR(accel_c.ax, accel_c2.ax, 1.0e-5);
  EXPECT_NEAR(accel_c.ay, accel_c2.ay, 1.0e-5);
  EXPECT_NEAR(accel_c.yaw_accel, accel_c2.yaw_accel, 1.0e-5);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}