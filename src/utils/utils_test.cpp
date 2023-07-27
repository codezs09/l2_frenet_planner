#include "utils.h"

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>

using namespace utils;

TEST(GenGaussianNoiseTest, group1) {
  double mean = 1.0;
  double std_dev = 2.0;
  int num_samples = 10000;

  double mean_act = 0.0;
  vector<double> noise_vec(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    double noise = genGaussianNoise(mean, std_dev);
    noise_vec[i] = noise;

    mean_act = mean_act * i / (i + 1) + noise / (i + 1);

    // std::cout << noise << " ";
  }

  double sum_err_square = 0.0;
  for (int i = 0; i < num_samples; ++i) {
    sum_err_square += (noise_vec[i] - mean_act) * (noise_vec[i] - mean_act);
  }
  double std_dev_act = sqrt(sum_err_square / num_samples);

  std::cout << "\nMean: " << mean << ",  Mean Actual: " << mean_act
            << std::endl;
  std::cout << "\nStd Dev: " << std_dev << ",  Std Dev Actual: " << std_dev_act
            << std::endl;

  EXPECT_NEAR(mean_act, mean, 0.1);
  EXPECT_NEAR(std_dev_act, std_dev, 0.1);
}

TEST(GenGaussianNoiseTest, group2) {
  double mean = -0.2;
  double std_dev = 0.2;
  int num_samples = 10000;

  double mean_act = 0.0;
  vector<double> noise_vec(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    double noise = genGaussianNoise(mean, std_dev);
    noise_vec[i] = noise;

    mean_act = mean_act * i / (i + 1) + noise / (i + 1);

    // std::cout << noise << " ";
  }

  double sum_err_square = 0.0;
  for (int i = 0; i < num_samples; ++i) {
    sum_err_square += (noise_vec[i] - mean_act) * (noise_vec[i] - mean_act);
  }
  double std_dev_act = sqrt(sum_err_square / num_samples);

  std::cout << "\nMean: " << mean << ",  Mean Actual: " << mean_act
            << std::endl;
  std::cout << "\nStd Dev: " << std_dev << ",  Std Dev Actual: " << std_dev_act
            << std::endl;

  EXPECT_NEAR(mean_act, mean, 0.01);
  EXPECT_NEAR(std_dev_act, std_dev, 0.01);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}