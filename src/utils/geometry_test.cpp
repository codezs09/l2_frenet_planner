#include "geometry.h"

#include <gtest/gtest.h>

#include "utils.h"

using namespace utils;

TEST(CollisionCheckTest, IsCollision) {
  vector<double> ob_pts_x = {38.72, 38.18, 33.11, 33.65};
  vector<double> ob_pts_y = {-3.94, -6.28, -5.11, -2.77};

  vector<double> ego_pts_x = {34.50, 34.18, 29.33, 29.65};
  vector<double> ego_pts_y = {0.97, -0.86, -0.01, 1.82};

  Corners ob_pts = {
      Point(ob_pts_x[0], ob_pts_y[0]), Point(ob_pts_x[1], ob_pts_y[1]),
      Point(ob_pts_x[2], ob_pts_y[2]), Point(ob_pts_x[3], ob_pts_y[3])};
  Corners ego_pts = {
      Point(ego_pts_x[0], ego_pts_y[0]), Point(ego_pts_x[1], ego_pts_y[1]),
      Point(ego_pts_x[2], ego_pts_y[2]), Point(ego_pts_x[3], ego_pts_y[3])};

  Box ob_box(ob_pts);
  Box ego_box(ego_pts);

  EXPECT_FALSE(is_collision(ob_box, ego_box));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}