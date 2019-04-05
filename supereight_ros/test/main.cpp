//
// Created by anna on 05/04/19.
//
#include <ros/ros.h>
#include <ros/console.h>
//#include "supereight_ros/supereight_ros.hpp"
#include <gtest/gtest.h>



int main(int argc, char **argv) {
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Error);
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test");
  return RUN_ALL_TESTS();
}
