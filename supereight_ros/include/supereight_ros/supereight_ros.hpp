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
#include <queue>
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
#include <time.h>

// supereight package
#include <se/DenseSLAMSystem.h>
#include <se/perfstats.h>
#include <se/config.h>
#include <se/thirdparty/vector_types.h>
#include <se/interface.h>

// supereight_ros headers
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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SupereightNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
//  SupereightNode(const ros::NodeHandle& nh, const ros:: NodeHandle& nh_private,
//      Configuration& config);

  virtual  ~SupereightNode(){}

/**
* @brief sets configuration from YAML file to nodehandle
 * definitions, see supereight/se/config.h
**/
  void setSupereightConfig(const ros::NodeHandle& nh_private);

/**
 * @brif prints configuration parameters of the supereight denseSLAM pipeline
 * @param config
 */
  void printSupereightConfig(const Configuration& config);
  // void mapUpdate
  //

  //TODO
  // insert Callback functions here
  // public variables
  Eigen::Vector3f init_pose_;
 private:
 /**
* @brief Sets up publishing and subscribing, should only be called from
* constructor
**/
  void setupRos();


  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  std::string frame_id_;

  // get pipeline with map
  std::shared_ptr<DenseSLAMSystem> pipeline_ = nullptr;
  std::shared_ptr<se::Octree<OFusion>> octree_ = nullptr;
  // pipeline configuration
  Configuration supereight_config_;
  Eigen::Vector2i image_size_;
  int frame_;
  uint16_t* input_depth_ = nullptr;
  Eigen::Vector2i computation_size_;

  // Subscriber
  ros::Subscriber image_depth_sub_;
  ros::Subscriber vicon_sub_;
  ros::Subscriber image_vicon_sub_;

  //Publisher
  ros::Publisher image_vicon_pub_;

  //Visualization
  ros::Publisher map_marker_pub_;
  ros::Publisher voxel_based_marker_pub_;
  ros::Publisher voxel_based_marker_array_pub_;
  ros::Publisher block_based_marker_pub_;
  ros::Publisher block_based_marker_array_pub_;
  ros::Publisher occupancy_map_pub_;

  /**
  * buffer and quque for incoming data streams, in case the matching can't
  * be immediately done.
  */
  CircularBuffer<geometry_msgs::TransformStamped> vicon_buffer_;
  std::queue<sensor_msgs::Image> image_queue_;

  // timing
  std::queue<std::chrono::time_point<std::chrono::system_clock>> stop_watch_;
  uint64_t image_time_stamp_;


};

} // namespace se

#endif //ASL_WS_SUPEREIGHT_ROS_HPP
