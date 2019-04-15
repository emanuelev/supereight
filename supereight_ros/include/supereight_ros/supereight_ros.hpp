//
// Created by anna on 04/04/19.
//

/**
* @brief
* @param
* @returns
**/

#ifndef SUPEREIGHT_ROS_HPP
#define SUPEREIGHT_ROS_HPP



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

#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <glog/logging.h>
#include <time.h>
#include <std_msgs/String.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>

//#include <mav_visualization/helpers.h>

// supereight package
#include <se/DenseSLAMSystem.h>
#include <se/perfstats.h>
#include <se/config.h>
#include <se/thirdparty/vector_types.h>
#include <se/interface.h>
#include <se/node_iterator.hpp>
#include <se/utils/morton_utils.hpp>
// supereight_ros headers
#include <supereight_ros/CircularBuffer.hpp>
#include <supereight_ros/ImagePose.h> //message



namespace se {

// use this struct to pass the map around
struct MapBuffer {
  uint16_t *inputDepth = nullptr;
  uchar3 *inputRGB = nullptr;
  uchar4 *depthRender = nullptr;
  uchar4 *trackRender = nullptr;
  uchar4 *volumeRender = nullptr;
  DepthReader *reader = nullptr;
  DenseSLAMSystem *pipeline = nullptr;
};

template <typename T>
class SupereightNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SupereightNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
      : nh_(nh),
      nh_private_(nh_private),
      pose_buffer_(500),
      image_size_{640,480},
      frame_(0),
      frame_id_("map"),
      occupied_voxels_sum_(0){
    // Configure supereight_config with default values
    setSupereightConfig(nh_private);
    input_depth_ = (uint16_t*) malloc(sizeof(uint16_t) * image_size_.x() *
      image_size_.y());
    init_pose_ = supereight_config_.initial_pos_factor.cwiseProduct
      (supereight_config_.volume_size);
    computation_size_ = image_size_ / supereight_config_.compute_size_ratio;

    pipeline_ = std::shared_ptr<DenseSLAMSystem>(new DenseSLAMSystem(
      Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
      Eigen::Vector3i::Constant(static_cast<int>(supereight_config_.volume_resolution.x())),
      Eigen::Vector3f::Constant(supereight_config_.volume_size.x()),
      init_pose_,
      supereight_config_.pyramid,
      supereight_config_)
    );

    pipeline_->getMap(octree_);
    setupRos();
    std::cout << "map publication mode block " << pub_block_based_  << ", map"
                                                                       " "<<
      pub_wo_map_update_<< std::endl;
    res_ = (double) (pipeline_->getModelDimensions())[0] / (double)
       (pipeline_->getModelResolution())[0];

  }
//  SupereightNode(const ros::NodeHandle& nh, const ros:: NodeHandle& nh_private,
//      Configuration& config);

  virtual  ~SupereightNode() {}

/**
* @brief sets configuration from YAML file to nodehandle
 * definitions, see supereight/se/config.h
**/
  void setSupereightConfig(const ros::NodeHandle &nh_private);

/**
 * @brif prints configuration parameters of the supereight denseSLAM pipeline
 * @param config
 */
  void printSupereightConfig(const Configuration &config);
  // void mapUpdate
  //

  // public variables
  Eigen::Vector3f init_pose_;

  // Visualization object
//  Visualization visualizer_;
 private:
  /**
 * @brief Sets up publishing and subscribing, should only be called from
 * constructor
 **/
  void setupRos();

/**
 * @brief adds images to image queue
 * @param image_msg
 */
  void imageCallback(const sensor_msgs::ImageConstPtr &image_msg);

  /**
   * @brief adds pose from ground truth file or recorded pose to pose buffer
   * @param pose_msg
   */
  void poseCallback(const geometry_msgs::TransformStamped::ConstPtr &pose_msg);

  /**
   * @brief aligns the pose with the image and calls the supereight denseslam
   * pipeline
   * @param image_pose_msg
   */
  void fusionCallback(const supereight_ros::ImagePose::ConstPtr &
  image_pose_msg);

  /**
   * @brief loads the occpuancy map and publishs it to a ros topic
   * @param updated_blocks
   */
  void visualizeMapOFusion(std::vector<Eigen::Vector3i> updated_blocks);

  // TODO: change SDF visualization to be block based
  /**
   * @brief loads the SDF map and publishs it to a ros topic
   * @param updated_blocks
   */
  void visualizeMapSDF(std::vector<Eigen::Vector3i> occupied_voxels,
                       std::vector<Eigen::Vector3i> freed_voxels,
                       std::vector<Eigen::Vector3i> updated_blocks);
  /**
   * @brief       Calculates the fraction a given sample is located between the closest pre and post sample
   *
   * @param[in]   pre_time_stamp    Closest pre vicon time stamp to image
   * @param[in]   post_time_stamp   Closest post vicon time stamp to image
   * @param[in]   img_time_stamp    Image time stamp
   *
   * @return      Fraction
   */
  double calculateAlpha(int64_t pre_time_stamp,
                        int64_t post_time_stamp,
                        int64_t img_time_stamp);

  /**
   * @brief      Linear 3D interpolation
   *
   * @param[in]  pre_vector3D  The pre vector 3D
   * @param[in]  post_vector3D      The post vector 3D
   * @param[in]  alpha              Fraction
   *
   * @return     Interpolated translation
   */
  Eigen::Vector3d interpolateVector(const Eigen::Vector3d &pre_vector3D,
                                    const Eigen::Vector3d &post_vector3D,
                                    double alpha);

  /**
   * @brief      Slerp interpolation for quaterniond
   *
   * @param[in]  pre_orientation       The previous orientation
   * @param[in]  post_orientation      The post orientation
   * @param[in]  alpha                 Fraction
   *
   * @return     Interpolated orientation
   */
  Eigen::Quaterniond interpolateOrientation(const Eigen::Quaterniond
                                            &pre_orientation,
                                            const Eigen::Quaterniond &post_orientation,
                                            double alpha);

  /**
   * @brief      Interpolation for transformations
   *
   * @param[in]  pre_transformation       The previous transformation
   * @param[in]  post_transformation      The post transformation
   * @param[in]  alpha                    Fraction
   *
   * @return     Interpolated transformation
   */
  Eigen::Matrix4d interpolatePose(
      const geometry_msgs::TransformStamped &pre_transformation,
      const geometry_msgs::TransformStamped &post_transformation,
      int64_t img_time_stamp);

  /* Taken from https://github.com/ethz-asl/volumetric_mapping */
  std_msgs::ColorRGBA percentToColor(double h) ;


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
  uint16_t *input_depth_ = nullptr;
  Eigen::Vector2i computation_size_;
  double res_;
  int occupied_voxels_sum_;

  // Subscriber
  ros::Subscriber image_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber image_pose_sub_;

  //Publisher
  ros::Publisher image_pose_pub_;

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
  CircularBuffer<geometry_msgs::TransformStamped> pose_buffer_;
  std::deque<sensor_msgs::Image> image_queue_;

  // timing
  std::deque<std::chrono::time_point<std::chrono::system_clock>> stop_watch_;
  uint64_t image_time_stamp_;

// voxel blockwise update for visualization
  std::map<int, std::vector<Eigen::Vector3i>> voxel_block_map_;


  // block based visualization
  bool pub_wo_map_update_ = true;
  bool pub_block_based_marker_ = false;
  bool pub_block_based_marker_array_ = true;
  bool pub_block_based_ = pub_block_based_marker_ ||
      pub_block_based_marker_array_;
};

} // namespace se


#include "supereight_ros/supereight_ros_impl.hpp"
#endif // SUPEREIGHT_ROS_HPP
