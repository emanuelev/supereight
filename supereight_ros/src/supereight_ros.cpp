//
// Created by anna on 04/04/19.
//
#include "supereight_ros/supereight_ros.hpp"

namespace se {

SupereightNode::SupereightNode(const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private){

}

} // namespace se
//
//SupereightNode::SupereightNode(const ros::NodeHandle& nh, const
//    ros::NodeHandle& nh_private,
//    Configuration& config) :
//    nh_(nh),
//    nh_private_(nh_private),
//    config_(config),
//    computation_size_(pipeline_->getComputationResolution()),
//    frame_(0) {
//
//  input_depth_ = (uint16_t*) malloc(sizeof(uint16_t) * computation_size_.x()*computation_size_.y());
//  setupRos();
//}


//void SupereightNode::initializeConfig(const ros::NodeHandle& nh_private){
//  int a = 1;
//}
//
//void SupereightNode::setupRos(){
//  occupancy_map_pub_;
//
//  // Subscriber
//  image_depth_sub_;
//  vicon_sub_;
//}
// Read config.yaml into Configuration class
//void initalizeConfig(Configuration &config, Eigen::Vector2i &input_size) {
//  ros::NodeHandle nh_private("~");
//  nh_private.param<int>("compute_size_ratio",
//                        config.compute_size_ratio,
//                        default_compute_size_ratio);
//
//  nh_private.param<int>("tracking_rate",
//                        config.tracking_rate,
//                        default_tracking_rate);
//
//  nh_private.param<int>("integration_rate",
//                        config.integration_rate,
//                        default_integration_rate);
//
//  nh_private.param<int>("integration_rate",
//                        config.rendering_rate,
//                        default_rendering_rate);
//
//  std::vector<int> volume_resolution_vector;
//  if (nh_private.getParam("volume_resolution", volume_resolution_vector)) {
//    for (unsigned int i = 0; i < volume_resolution_vector.size(); i++) {
//      config.volume_resolution[i] = volume_resolution_vector[i];
//    }
//  } else {
//    config.volume_resolution = default_volume_resolution;
//  }
//
//  std::vector<float> volume_size_vector;
//  if (nh_private.getParam("volume_size", volume_size_vector)) {
//    for (unsigned int i = 0; i < volume_size_vector.size(); i++) {
//      config.volume_size[i] = volume_size_vector[i];
//    }
//  } else {
//    config.volume_size = default_volume_size;
//  }
//
//  std::vector<float> initial_pos_factor_vector;
//  if (nh_private.getParam("initial_pos_factor", initial_pos_factor_vector)) {
//    for (unsigned int i = 0; i < initial_pos_factor_vector.size(); i++) {
//      config.initial_pos_factor[i] = initial_pos_factor_vector[i];
//    }
//  } else {
//    config.initial_pos_factor = default_initial_pos_factor;
//  }
//
//  std::vector<int> pyramid;
//  if (!nh_private.getParam("pyramid", pyramid)) {
//    config.pyramid.clear();
//    for (int i = 0; i < DEFAULT_ITERATION_COUNT; i++) {
//      config.pyramid.push_back(default_iterations[i]);
//    }
//  }
//
//  nh_private.param<std::string>("dump_volume_file",
//                                config.dump_volume_file,
//                                default_dump_volume_file);
//
//  nh_private.param<std::string>("input_file",
//                                config.input_file,
//                                default_input_file);
//
//  nh_private.param<std::string>("log_file",
//                                config.log_file,
//                                default_log_file);
//
//  nh_private.param<std::string>("groundtruth_file",
//                                config.groundtruth_file,
//                                default_groundtruth_file);
//
//  std::vector<float> gt_transform_vector;
//  nh_private.getParam("gt_transform", gt_transform_vector);
//  if (nh_private.getParam("gt_transform", gt_transform_vector)) {
//    for (unsigned int i = 0; i < std::sqrt(gt_transform_vector.size()); i++) {
//      for (unsigned int j = 0; j < std::sqrt(gt_transform_vector.size()); j++) {
//        config.gt_transform(i, j) = gt_transform_vector[i * 4 + j];
//      }
//    }
//  } else {
//    config.gt_transform = default_gt_transform;
//  }
//
//  std::vector<float> camera_vector;
//  if (!nh_private.getParam("camera", camera_vector)) {
//    ros::shutdown();
//  }
//  for (unsigned int i = 0; i < camera_vector.size(); i++) {
//    config.camera[i] = camera_vector[i];
//  }
//  config.camera_overrided = true;
//
//  nh_private.param<float>("mu",
//                          config.mu,
//                          default_mu);
//
//  nh_private.param<int>("fps",
//                        config.fps,
//                        default_fps);
//
//  nh_private.param<bool>("blocking_read",
//                         config.blocking_read,
//                         default_blocking_read);
//
//  nh_private.param<float>("icp_threshold",
//                          config.icp_threshold,
//                          default_icp_threshold);
//
//  nh_private.param<bool>("no_gui",
//                         config.no_gui,
//                         default_no_gui);
//
//  nh_private.param<bool>("render_volume_fullsize",
//                         config.render_volume_fullsize,
//                         default_render_volume_fullsize);
//
//  nh_private.param<bool>("bilateral_filter",
//                         config.bilateralFilter,
//                         default_bilateral_filter);
//
//  nh_private.param<bool>("coloured_voxels",
//                         config.coloured_voxels,
//                         default_coloured_voxels);
//
//  nh_private.param<bool>("multi_resolution",
//                         config.multi_resolution,
//                         default_multi_resolution);
//
//  nh_private.param<bool>("bayesian",
//                         config.bayesian,
//                         default_bayesian);
//
//  std::vector<int> input_size_vector;
//  if (nh_private.getParam("input_size", input_size_vector)) {
//    for (unsigned int i = 0; i < input_size_vector.size(); i++) {
//      input_size[i] = input_size_vector[i];
//    }
//  } else {
//    input_size = default_input_size;
//  }
//};

