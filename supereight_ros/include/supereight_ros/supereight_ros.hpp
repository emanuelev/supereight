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
#include <glog/logging.h>


#include <se/DenseSLAMSystem.h>
#include <se/perfstats.h>
#include <se/config.h>
#include <se/thirdparty/vector_types.h>
#include <se/interface.h>
//#include <se/default_parameters.h>

#include <supereight_ros/CircularBuffer.hpp>

namespace se {

// use this struct to pass the map around
struct MapBuffer{
  uint16_t *inputDepth = nullptr;
 uchar3 *inputRGB = nullptr;
 uchar4 * depthRender = nullptr;
 uchar4 * trackRender = nullptr;
 uchar4 * volumeRender = nullptr;
 DepthReader *reader = nullptr;
 DenseSLAMSystem *pipeline = nullptr;
};


class SupereightNode{
 public:
  SupereightNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
//  SupereightNode(const ros::NodeHandle& nh, const ros:: NodeHandle& nh_private,
//      Configuration& config);

  virtual  ~SupereightNode(){}

/**
* @brief reads configuration from YAML file to supereight_config which is
 * def in supereight/se/config.h ined
 * @param[in] filePath from launch file
**/
  void readConfigFile(const std::string& filePath);
/**
* @brief sets configuration to nodehandle from launch file
 *map settings,
**/
  void setNodeParam(const ros::NodeHandle& nh_private);

  // void mapUpdate
  //
 private:


  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // pipeline configuration
  Configuration supereight_config_;
  // get pipeline with map
  std::shared_ptr<DenseSLAMSystem> pipeline_ = nullptr;

  // Publisher
  ros::Publisher occupancy_map_pub_;


  // Subscriber
  ros::Subscriber image_depth_sub_;
  ros::Subscriber vicon_sub_;

  //TODO
  // insert Callback functions here

  // input buffer
  //CircularBuffer<geometry_msgs::TransformStamped> vicon_buffer_ =
  //std::deque<sensor_msgs::Image> image_buffer_;

  // timing
  std::deque<std::chrono::time_point<std::chrono::system_clock>> stop_watch_;
  uint64_t image_time_stamp_;

  int frame_;
  uint16_t* input_depth_ = nullptr;


  Eigen::Vector2i computation_size_;
/**
* @brief Sets up publishing and subscribing, should only be called from
* constructor
**/
  void setupRos();
};

} // namespace se

#endif //ASL_WS_SUPEREIGHT_ROS_HPP
