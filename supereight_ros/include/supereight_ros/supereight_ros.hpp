//
// Created by anna on 04/04/19.
//

/**
* @brief
* @param
* @returns
**/
#ifndef ASL_WS_SUPEREIGHT_ROS_HPP
#define ASL_WS_SUPEREIGHT_ROS_HPP

#include <stdint.h>
#include <vector>
#include <sstream>
#include <string>
#include <cstring>
#include <iomanip>
#include <ctime>
#include <ratio>
#include <chrono>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>

#include <se/DenseSLAMSystem.h>
#include <se/perfstats.h>
#include <se/config.h>
//#include <se/default_parameters.h>

#include <supereight_ros/CircularBuffer.hpp>

namespace se {

class SupereightNode{
 public:
  SupereightNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
//  SupereightNode(const ros::NodeHandle& nh, const ros:: NodeHandle& nh_private,
//      Configuration& config);

  virtual  ~SupereightNode(){}

  //void initializeConfig(const ros::NodeHandle& nh_private);

 private:
/**
* @brief Sets up publishing and subscribing, should only be called from
* constructor
**/
  //void setupRos();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // get pipeline with map
  std::shared_ptr<DenseSLAMSystem> pipeline_ = nullptr;

  // input buffer
  //CircularBuffer<geometry_msgs::TransformStamped> vicon_buffer_ =
  //std::deque<sensor_msgs::Image> image_buffer_;

  // timing
  std::deque<std::chrono::time_point<std::chrono::system_clock>> stop_watch_;
  uint64_t image_time_stamp_;

  int frame_;
  uint16_t* input_depth_ = nullptr;

  //Configuration& config_;
  Eigen::Vector2i computation_size_;

  // Publisher
  ros::Publisher occupancy_map_pub_;


  // Subscriber
  ros::Subscriber image_depth_sub_;
  ros::Subscriber vicon_sub_;
};

} // namespace se

#endif //ASL_WS_SUPEREIGHT_ROS_HPP
