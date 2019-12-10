/**
 * @file        main.cpp
 * @author      Ashwin Kuruttukulam
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam
 * @date        Dec 6, 2019
 * @brief       Main test file
 */
#include <ros/ros.h>
#include <gtest/gtest.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "kndTest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
