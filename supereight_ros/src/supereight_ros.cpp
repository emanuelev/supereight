//
// Created by anna on 04/04/19.
//
#include "supereight_ros/supereight_ros.hpp"


#include <iostream>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <se/default_parameters.h>

namespace se {

SupereightNode::SupereightNode(const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      vicon_buffer_(500),
      image_size_{640,480},
      frame_(0),
      frame_id_("map"){
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



}
//
void SupereightNode::setupRos(){

  // Subscriber
//  image_depth_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 100, &DataSynchronizer::imageDepthCallback, this);
//  vicon_sub_ = nh_.subscribe("/vicon/d435/d435", 100, &DataSynchronizer::viconCallback, this);
//  image_vicon_sub_ = nh_.subscribe("/image_pose", 300, &DataSynchronizer::fusionCallback, this);

  // Publisher
//  image_vicon_pub_ = nh_.advertise<ptp_ros::ImagePose>("/image_pose", 1000);

  // Visualization
//  map_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("map_based_marker", 1);
//  voxel_based_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("voxel_based_marker", 1);
//  voxel_based_marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("voxel_based_marker_array", 1);
//  block_based_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("block_based_marker", 1);
//  block_based_marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("block_based_marker_array", 1);


}


// Read config.yaml into Configuration class
void SupereightNode::setSupereightConfig(const ros::NodeHandle& nh_private) {

  nh_private.param<int>("compute_size_ratio",
                        supereight_config_.compute_size_ratio,
                        default_compute_size_ratio);

  nh_private.param<int>("tracking_rate",
                        supereight_config_.tracking_rate,
                        default_tracking_rate);

  nh_private.param<int>("integration_rate",
                        supereight_config_.integration_rate,
                        default_integration_rate);

  nh_private.param<int>("integration_rate",
                        supereight_config_.rendering_rate,
                        default_rendering_rate);

  std::vector<int> volume_resolution_vector;
  if (nh_private.getParam("volume_resolution", volume_resolution_vector)) {
    for (unsigned int i = 0; i < volume_resolution_vector.size(); i++) {
      supereight_config_.volume_resolution[i] = volume_resolution_vector[i];
    }
  } else {
    supereight_config_.volume_resolution = default_volume_resolution;
  }

  std::vector<float> volume_size_vector;
  if (nh_private.getParam("volume_size", volume_size_vector)) {
    for (unsigned int i = 0; i < volume_size_vector.size(); i++) {
      supereight_config_.volume_size[i] = volume_size_vector[i];
    }
  } else {
    supereight_config_.volume_size = default_volume_size;
  }

  std::vector<float> initial_pos_factor_vector;
  if (nh_private.getParam("initial_pos_factor", initial_pos_factor_vector)) {
    for (unsigned int i = 0; i < initial_pos_factor_vector.size(); i++) {
      supereight_config_.initial_pos_factor[i] = initial_pos_factor_vector[i];
    }
  } else {
    supereight_config_.initial_pos_factor = default_initial_pos_factor;
  }

  std::vector<int> pyramid;
  if (!nh_private.getParam("pyramid", pyramid)) {
    supereight_config_.pyramid.clear();
    for (int i = 0; i < DEFAULT_ITERATION_COUNT; i++) {
      supereight_config_.pyramid.push_back(default_iterations[i]);
    }
  }

  nh_private.param<std::string>("dump_volume_file",
                                supereight_config_.dump_volume_file,
                                default_dump_volume_file);

  nh_private.param<std::string>("input_file",
                                supereight_config_.input_file,
                                default_input_file);

  nh_private.param<std::string>("log_file",
                                supereight_config_.log_file,
                                default_log_file);

  nh_private.param<std::string>("groundtruth_file",
                                supereight_config_.groundtruth_file,
                                default_groundtruth_file);

  std::vector<float> gt_transform_vector;
  nh_private.getParam("gt_transform", gt_transform_vector);
  if (nh_private.getParam("gt_transform", gt_transform_vector)) {
    for (unsigned int i = 0; i < std::sqrt(gt_transform_vector.size()); i++) {
      for (unsigned int j = 0; j < std::sqrt(gt_transform_vector.size()); j++) {
        supereight_config_.gt_transform(i, j) = gt_transform_vector[i * 4 + j];
      }
    }
  } else {
    supereight_config_.gt_transform = default_gt_transform;
  }

  std::vector<float> camera_vector;
  if (!nh_private.getParam("camera", camera_vector)) {
    ros::shutdown();
  }
  for (unsigned int i = 0; i < camera_vector.size(); i++) {
    supereight_config_.camera[i] = camera_vector[i];
  }
  supereight_config_.camera_overrided = true;

  /**
   * The TSDF truncation bound. Values of the TSDF are assumed to be in the
   * interval ±mu. See Section 3.3 of \cite NewcombeISMAR2011 for more
   * details.
   *  <br>\em Default: 0.1
   */
  nh_private.param<float>("mu",
                          supereight_config_.mu,
                          default_mu);

  nh_private.param<int>("fps",
                        supereight_config_.fps,
                        default_fps);

  nh_private.param<bool>("blocking_read",
                         supereight_config_.blocking_read,
                         default_blocking_read);

  nh_private.param<float>("icp_threshold",
                          supereight_config_.icp_threshold,
                          default_icp_threshold);

  nh_private.param<bool>("no_gui",
                         supereight_config_.no_gui,
                         default_no_gui);

  nh_private.param<bool>("render_volume_fullsize",
                         supereight_config_.render_volume_fullsize,
                         default_render_volume_fullsize);

  nh_private.param<bool>("bilateral_filter",
                         supereight_config_.bilateralFilter,
                         default_bilateralFilter);

  nh_private.param<bool>("coloured_voxels",
                         supereight_config_.coloured_voxels,
                         default_coloured_voxels);

  nh_private.param<bool>("multi_resolution",
                         supereight_config_.multi_resolution,
                         default_multi_resolution);

  nh_private.param<bool>("bayesian",
                         supereight_config_.bayesian,
                         default_bayesian);

  std::vector<int> input_size_vector;
  if (nh_private.getParam("input_size", input_size_vector)) {
    for (unsigned int i = 0; i < input_size_vector.size(); i++) {
      image_size_[i] = input_size_vector[i];
    }
  }
};

void SupereightNode::printSupereightConfig(const Configuration& config) {
  std::cout << "compute_size_ratio = " << config.compute_size_ratio << std::endl;
  std::cout << "tracking_rate = " << config.tracking_rate << std::endl;
  std::cout << "integration_rate = " << config.integration_rate << std::endl;
  std::cout << "rendering_rate = " << config.rendering_rate << std::endl;
  std::cout << "volume_resolution = \n" << config.volume_resolution << std::endl;
  std::cout << "volume_size = \n" << config.volume_size << std::endl;
  std::cout << "initial_pos_factor = \n" << config.initial_pos_factor << std::endl;
  std::cout << "pyramid = \n" << config.pyramid[0] << " " << config.pyramid[1] << " " << config.pyramid[2] << std::endl;
  std::cout << "dump_volume_file = " << config.dump_volume_file << std::endl;
  std::cout << "input_file = " << config.input_file << std::endl;
  std::cout << "log_file = " << config.log_file << std::endl;
  std::cout << "groundtruth_file = " << config.groundtruth_file << std::endl;
  std::cout << "gt_transform = \n" << config.gt_transform << std::endl;
  std::cout << "camera = \n" << config.camera << std::endl;
  std::cout << "camera_overrided = " << config.camera_overrided << std::endl;
  std::cout << "mu = " << config.mu << std::endl;
  std::cout << "fps = " << config.fps << std::endl;
  std::cout << "blocking_read = " << config.blocking_read << std::endl;
  std::cout << "icp_threshold = " << config.icp_threshold << std::endl;
  std::cout << "no_gui = " << config.no_gui << std::endl;
  std::cout << "render_volume_fullsize = " << config.render_volume_fullsize << std::endl;
  std::cout << "bilateral_filter = " << config.bilateralFilter << std::endl;
  std::cout << "coloured_voxels = " << config.coloured_voxels << std::endl;
  std::cout << "multi_resolution = " << config.multi_resolution << std::endl;
  std::cout << "bayesian = " << config.bayesian << std::endl;
}
} // namespace se






