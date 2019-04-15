//
// Created by anna on 12/04/19.
//

#ifndef SUPEREIGHT_SUPEREIGHT_ROS_IMPL_HPP
#define SUPEREIGHT_SUPEREIGHT_ROS_IMPL_HPP

#include <iostream>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <se/default_parameters.h>
#include <supereight_ros/supereight_ros.hpp>
namespace se {

template <typename T>
void SupereightNode<T>::setupRos() {

  // Subscriber
  image_sub_ = nh_.subscribe
      ("/camera/aligned_depth_to_color/image_raw", 100,
       &SupereightNode::imageCallback, this);
  pose_sub_ = nh_.subscribe("/vicon/d435/d435", 100,
                             &SupereightNode::poseCallback, this);
    // Publisher
  image_pose_pub_ = nh_.advertise<supereight_ros::ImagePose>("/image_pose",
      1000);
  image_pose_sub_ = nh_.subscribe("/image_pose", 300,
                                   &SupereightNode::fusionCallback, this);


  // Visualization
  map_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("map_based_marker", 1);
  block_based_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("block_based_marker", 1);
  block_based_marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("block_based_marker_array", 1);


}

// Read config.yaml into Configuration class
template <typename T>
void SupereightNode<T>::setSupereightConfig(const ros::NodeHandle &nh_private) {

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
                         supereight_config_.bilateral_filter,
                         default_bilateral_filter);

  nh_private.param<bool>("coloured_voxels",
                         supereight_config_.coloured_voxels,
                         default_coloured_voxels);

  nh_private.param<bool>("multi_resolution",
                         supereight_config_.multi_resolution,
                         default_multi_resolution);

  nh_private.param<bool>("bayesian",
                         supereight_config_.bayesian,
                         default_bayesian);

  std::vector<int> image_size_vector;
  if (nh_private.getParam("input_size", image_size_vector)) {
    for (unsigned int i = 0; i < image_size_vector.size(); i++) {
      image_size_[i] = image_size_vector[i];
    }
  }
};

// print configuration to terminal
template <typename T>
void SupereightNode<T>::printSupereightConfig(const Configuration &config) {
  std::cout << "compute_size_ratio = " << config.compute_size_ratio
            << std::endl;
  std::cout << "tracking_rate = " << config.tracking_rate << std::endl;
  std::cout << "integration_rate = " << config.integration_rate << std::endl;
  std::cout << "rendering_rate = " << config.rendering_rate << std::endl;
  std::cout << "volume_resolution = \n" << config.volume_resolution
            << std::endl;
  std::cout << "volume_size = \n" << config.volume_size << std::endl;
  std::cout << "initial_pos_factor = \n" << config.initial_pos_factor
            << std::endl;
  std::cout << "pyramid = \n" << config.pyramid[0] << " " << config.pyramid[1]
            << " " << config.pyramid[2] << std::endl;
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
  std::cout << "render_volume_fullsize = " << config.render_volume_fullsize
            << std::endl;
  std::cout << "bilateral_filter = " << config.bilateral_filter << std::endl;
  std::cout << "coloured_voxels = " << config.coloured_voxels << std::endl;
  std::cout << "multi_resolution = " << config.multi_resolution << std::endl;
  std::cout << "bayesian = " << config.bayesian << std::endl;
}

template <typename T>
void SupereightNode<T>::imageCallback(const sensor_msgs::ImageConstPtr &
image_msg) {
  image_queue_.push_back(*image_msg);
  if (image_queue_.size() == 1) {
    image_time_stamp_ = ros::Time(image_queue_.front().header.stamp).toNSec();

  }
}

template <typename T>
void SupereightNode<T>::poseCallback(const
                                   geometry_msgs::TransformStamped::ConstPtr
                                   &pose_msg) {
  pose_buffer_.put(*pose_msg);
  while (image_queue_.size() > 0
      && (ros::Time(pose_msg->header.stamp).toNSec() > image_time_stamp_)) {

    supereight_ros::ImagePose image_pose_msg;

    image_pose_msg.image = image_queue_.front();

    geometry_msgs::TransformStamped pre_pose;
    geometry_msgs::TransformStamped post_pose;

    pose_buffer_.get(image_time_stamp_, pre_pose, post_pose);

    image_pose_msg.pre_pose = pre_pose;
    image_pose_msg.post_pose = post_pose;

    image_pose_pub_.publish(image_pose_msg);

    image_queue_.pop_front();

    if (image_queue_.size() > 0)
      image_time_stamp_ = ros::Time(image_queue_.front().header.stamp).toNSec();
  }
}

template <typename T>
void SupereightNode<T>::fusionCallback(const
    supereight_ros::ImagePose::ConstPtr& image_pose_msg) {
  bool integrated, raycasted;

  // Convert ROS message to cv image
  cv_bridge::CvImagePtr bridge;
  try {
      bridge = cv_bridge::toCvCopy(image_pose_msg->image, "32FC1");
  } catch (cv_bridge::Exception& e) {
      std::cout << "Failed to transform depth image." << std::endl;
  }


  for (int y = 0; y < image_size_.y(); y++) {
    for (int x = 0; x < image_size_.x(); x++) {
      input_depth_[x + image_size_.x() * y] = static_cast<uint16_t>
          (bridge->image.at<float>(x + image_size_.x() * y));
    }
  }

  Eigen::Matrix4d gt_pose = interpolatePose(image_pose_msg->pre_pose,
                                            image_pose_msg->post_pose,
                                            ros::Time(image_pose_msg->image.header.stamp).toNSec());

  pipeline_->setPose(gt_pose.cast<float>());

  pipeline_->preprocessing(input_depth_,
                          image_size_,
                          supereight_config_.bilateral_filter);

  Eigen::Vector4f camera = supereight_config_.camera / (supereight_config_.compute_size_ratio);

  std::vector<Eigen::Vector3i> occupied_voxels;
  std::vector<Eigen::Vector3i> freed_voxels;
  std::vector<Eigen::Vector3i> updated_blocks;

  integrated = pipeline_->integration(camera, supereight_config_
      .integration_rate, supereight_config_.mu, frame_, &updated_blocks);

//  std::cout << "occupied_voxels = " << occupied_voxels.size() << std::endl;
//  std::cout << "freed_voxels = " << freed_voxels.size() << std::endl;
//  std::cout << "updated_blocks = " << updated_blocks.size() << std::endl;

  if(std::is_same<FieldType, OFusion>::value) {
    visualizeMapOFusion( updated_blocks);
  } else if(std::is_same<FieldType, SDF>::value) {
    visualizeMapSDF(occupied_voxels, freed_voxels, updated_blocks);
  }

  frame_++;
}



template <typename T>
void SupereightNode<T>::visualizeMapOFusion(std::vector<Eigen::Vector3i> updated_blocks) {
  // publish every N-th frame
  int N_frame_pub = 1;

  pipeline_->getMap(octree_);
  node_iterator<T> node_it(*octree_);

  if (pub_wo_map_update_) {
    visualization_msgs::Marker map_marker_msg;

    std::vector<Eigen::Vector3i> occupied_voxels = node_it.getOccupiedVoxels();

    map_marker_msg.header.frame_id = frame_id_;
    map_marker_msg.ns = frame_id_;
    map_marker_msg.id = 0;
    map_marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
    map_marker_msg.scale.x = res_;
    map_marker_msg.scale.y = res_;
    map_marker_msg.scale.z = res_;
    map_marker_msg.action = visualization_msgs::Marker::ADD;
    map_marker_msg.color.r = 1.0f;
    map_marker_msg.color.g = 1.0f;
    map_marker_msg.color.b = 0.0f;
    map_marker_msg.color.a = 1.0;

    for (const auto& occupied_voxel : occupied_voxels) {
      geometry_msgs::Point cube_center;

      cube_center.x = ((double)occupied_voxel[0] + 0.5) * res_;
      cube_center.y = ((double)occupied_voxel[1] + 0.5) * res_;
      cube_center.z = ((double)occupied_voxel[2] + 0.5) * res_;

      map_marker_msg.points.push_back(cube_center);
    }

    if (frame_%N_frame_pub == 0) {
      map_marker_pub_.publish(map_marker_msg);
    }
  }

  if (pub_block_based_) {
    visualization_msgs::Marker voxel_block_marker;
    voxel_block_marker.header.frame_id = frame_id_;
    voxel_block_marker.ns = frame_id_;
    voxel_block_marker.type = visualization_msgs::Marker::CUBE_LIST;
    voxel_block_marker.scale.x = res_;
    voxel_block_marker.scale.y = res_;
    voxel_block_marker.scale.z = res_;
    voxel_block_marker.action = visualization_msgs::Marker::ADD;
    voxel_block_marker.color.r = 0.0f;
    voxel_block_marker.color.g = 0.0f;
    voxel_block_marker.color.b = 1.0f;
    voxel_block_marker.color.a = 1.0;

    visualization_msgs::MarkerArray voxel_block_marker_array_msg;
    visualization_msgs::Marker voxel_block_marker_msg = voxel_block_marker;

    if (pub_block_based_marker_) {
      voxel_block_marker_msg.id = 0;
      voxel_block_marker_msg.color.r = 1.0f;
      voxel_block_marker_msg.color.g = 0.0f;
      voxel_block_marker_msg.color.b = 0.0f;
      voxel_block_marker_msg.color.a = 1.0;
    }

    if (frame_ % N_frame_pub == 0) {

      for (const auto &updated_block : updated_blocks) {
        int morten_code = (int) compute_morton(updated_block[0],
                                               updated_block[1],
                                               updated_block[2]);

        std::vector<Eigen::Vector3i> occupied_block_voxels =
            node_it.getOccupiedVoxels(0.5, updated_block);

        if (pub_block_based_marker_array_) {
          voxel_block_marker.id = morten_code;
          voxel_block_marker.points.clear();

          for (const auto &occupied_voxel : occupied_block_voxels) {
            geometry_msgs::Point cube_center;
            cube_center.x =
                (static_cast<double>(occupied_voxel[0]) + 0.5) * res_;
            cube_center.y =
                (static_cast<double>(occupied_voxel[1]) + 0.5) * res_;
            cube_center.z =
                (static_cast<double>(occupied_voxel[2]) + 0.5) * res_;
            voxel_block_marker.points.push_back(cube_center);
          }
          voxel_block_marker_array_msg.markers.push_back(voxel_block_marker);
        }

        if (pub_block_based_marker_) {
          voxel_block_map_[morten_code] = occupied_block_voxels;
        }

      }

      if (pub_block_based_marker_array_) {
        block_based_marker_array_pub_.publish(voxel_block_marker_array_msg);
      }
    }

    if (pub_block_based_marker_) {
      for (auto voxel_block = voxel_block_map_.begin(); voxel_block !=
          voxel_block_map_.end(); voxel_block++) {
        for (const auto &occupied_voxel : voxel_block->second) {
          geometry_msgs::Point cube_center;

          cube_center.x = ((double) occupied_voxel[0] + 0.5) * res_;
          cube_center.y = ((double) occupied_voxel[1] + 0.5) * res_;
          cube_center.z = ((double) occupied_voxel[2] + 0.5) * res_;

          voxel_block_marker_msg.points.push_back(cube_center);
        }
      }
      block_based_marker_pub_.publish(voxel_block_marker_msg);
    }
  }
};

template <typename T>
void SupereightNode<T>::visualizeMapSDF(std::vector<Eigen::Vector3i>
    occupied_voxels,
                                          std::vector<Eigen::Vector3i> freed_voxels,
                                          std::vector<Eigen::Vector3i> updated_blocks) {
  // publish every N-th frame
  int N_frame_pub = 1;

  pipeline_->getMap(octree_);
  node_iterator<T> node_it(*octree_);



  if (pub_wo_map_update_) {
    visualization_msgs::Marker map_marker_msg;

    std::vector<Eigen::Vector3i> surface_voxels = node_it.getSurfaceVoxels();

    map_marker_msg.header.frame_id = frame_id_;
    map_marker_msg.ns = frame_id_;
    map_marker_msg.id = 0;
    map_marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
    map_marker_msg.scale.x = res_;
    map_marker_msg.scale.y = res_;
    map_marker_msg.scale.z = res_;
    map_marker_msg.action = visualization_msgs::Marker::ADD;
    map_marker_msg.color.r = 1.0f;
    map_marker_msg.color.g = 1.0f;
    map_marker_msg.color.b = 0.0f;
    map_marker_msg.color.a = 1.0;

    for (const auto& surface_voxel : surface_voxels) {
      geometry_msgs::Point cube_center;

      cube_center.x = ((double)surface_voxel[0] + 0.5) * res_;
      cube_center.y = ((double)surface_voxel[1] + 0.5) * res_;
      cube_center.z = ((double)surface_voxel[2] + 0.5) * res_;

      map_marker_msg.points.push_back(cube_center);
    }

    if (frame_%N_frame_pub == 0) {
      map_marker_pub_.publish(map_marker_msg);
    }
  }

  // if (pub_voxel_based) {

  //   visualization_msgs::Marker voxel_marker;
  //   voxel_marker.header.frame_id = "map";
  //   voxel_marker.ns = "map";
  //   voxel_marker.type = visualization_msgs::Marker::CUBE_LIST;
  //   voxel_marker.scale.x = res_;
  //   voxel_marker.scale.y = res_;
  //   voxel_marker.scale.z = res_;
  //   voxel_marker.action = visualization_msgs::Marker::ADD;
  //   voxel_marker.color.r = 0.0f;
  //   voxel_marker.color.g = 1.0f;
  //   voxel_marker.color.b = 0.0f;
  //   voxel_marker.color.a = 1.0;

  //   visualization_msgs::MarkerArray voxel_based_occupied_marker_array_msg;

  //   for (const auto& occupied_voxel : occupied_voxels) {
  //     voxel_marker.points.clear();

  //     geometry_msgs::Point cube_center;

  //     cube_center.x = ((double)occupied_voxel[0] + 0.5) * res_;
  //     cube_center.y = ((double)occupied_voxel[1] + 0.5) * res_;
  //     cube_center.z = ((double)occupied_voxel[2] + 0.5) * res_;

  //     voxel_marker.id = (int) compute_morton(occupied_voxel[0], occupied_voxel[1], occupied_voxel[2]);
  //     voxel_marker.points.push_back(cube_center);

  //     if (pub_voxel_based_marker) {
  //       voxel_based_marker_pub_ .publish(voxel_marker);
  //     }

  //     if (pub_voxel_based_marker_array) {
  //       voxel_based_occupied_marker_array_msg.markers.push_back(voxel_marker);
  //     }
  //   }

  //   if (pub_voxel_based_marker_array) {
  //     voxel_based_marker_array_pub_.publish(voxel_based_occupied_marker_array_msg);
  //   }

  //   visualization_msgs::MarkerArray voxel_based_freed_marker_array_msg;

  //   for (const auto& freed_voxel : freed_voxels) {
  //     voxel_marker.action = visualization_msgs::Marker::DELETE;
  //     voxel_marker.id = (int) compute_morton(freed_voxel[0], freed_voxel[1], freed_voxel[2]);

  //     if (pub_voxel_based_marker) {
  //       voxel_based_marker_pub_ .publish(voxel_marker);
  //     }

  //     if (pub_voxel_based_marker_array) {
  //       voxel_based_freed_marker_array_msg.markers.push_back(voxel_marker);
  //     }
  //   }

  //   if (pub_voxel_based_marker_array) {
  //     voxel_based_marker_array_pub_.publish(voxel_based_freed_marker_array_msg);
  //   }
  // }

  // if (pub_block_based) {
  //   visualization_msgs::Marker voxel_block_marker;
  //   voxel_block_marker.header.frame_id = "map";
  //   voxel_block_marker.ns = "map";
  //   voxel_block_marker.type = visualization_msgs::Marker::CUBE_LIST;
  //   voxel_block_marker.scale.x = res_;
  //   voxel_block_marker.scale.y = res_;
  //   voxel_block_marker.scale.z = res_;
  //   voxel_block_marker.action = visualization_msgs::Marker::ADD;
  //   voxel_block_marker.color.r = 0.0f;
  //   voxel_block_marker.color.g = 0.0f;
  //   voxel_block_marker.color.b = 1.0f;
  //   voxel_block_marker.color.a = 1.0;

  //   visualization_msgs::MarkerArray voxel_block_marker_array_msg;
  //   visualization_msgs::Marker voxel_block_marker_msg = voxel_block_marker;

  //   if (pub_block_based_marker) {
  //     voxel_block_marker_msg.id = 0;
  //     voxel_block_marker_msg.color.r = 1.0f;
  //     voxel_block_marker_msg.color.g = 0.0f;
  //     voxel_block_marker_msg.color.b = 0.0f;
  //     voxel_block_marker_msg.color.a = 1.0;
  //   }

  //   if ((pub_block_based_marker_array || pub_block_based_marker) && (frame_%N_frame_pub == 0)) {

  //     for (const auto& updated_block : updated_blocks) {
  //       int morten_code = (int) compute_morton(updated_block[0], updated_block[1], updated_block[2]);

  //       std::vector<Eigen::Vector3i,Eigen::aligned_allocator<Eigen::Vector3i>> occupied_block_voxels =
  //           node_it.getOccupiedVoxels(0.5, updated_block);;

  //       if (pub_block_based_marker_array) {
  //         voxel_block_marker.id = morten_code;
  //         voxel_block_marker.points.clear();

  //         for (const auto& occupied_voxel : occupied_block_voxels) {
  //           geometry_msgs::Point cube_center;
  //           cube_center.x = ((double)occupied_voxel[0] + 0.5) * res_;
  //           cube_center.y = ((double)occupied_voxel[1] + 0.5) * res_;
  //           cube_center.z = ((double)occupied_voxel[2] + 0.5) * res_;

  //           voxel_block_marker.points.push_back(cube_center);
  //         }
  //         voxel_block_marker_array_msg.markers.push_back(voxel_block_marker);
  //       }

  //       if (pub_block_based_marker) {
  //         voxel_block_map[morten_code] = occupied_block_voxels;
  //       }

  //     }

  //     if (pub_block_based_marker_array) {
  //       block_based_marker_array_pub_.publish(voxel_block_marker_array_msg);
  //     }
  //   }

  //   if (pub_block_based_marker) {
  //     for (auto voxel_block = voxel_block_map.begin(); voxel_block != voxel_block_map.end(); voxel_block++) {
  //       for (const auto& occupied_voxel : voxel_block->second) {
  //         geometry_msgs::Point cube_center;

  //         cube_center.x = ((double)occupied_voxel[0] + 0.5) * res_;
  //         cube_center.y = ((double)occupied_voxel[1] + 0.5) * res_;
  //         cube_center.z = ((double)occupied_voxel[2] + 0.5) * res_;

  //         voxel_block_marker_msg.points.push_back(cube_center);
  //       }
  //     }
  //     block_based_marker_pub_.publish(voxel_block_marker_msg);
  //   }
  // }
};


template <typename T>
double SupereightNode<T>::calculateAlpha(int64_t pre_time_stamp,
                      int64_t post_time_stamp,
                      int64_t img_time_stamp) {
  double alpha = (double) (img_time_stamp - pre_time_stamp)
      / (post_time_stamp - pre_time_stamp);
  return alpha;
}

template <typename T>
Eigen::Vector3d SupereightNode<T>::interpolateVector(const Eigen::Vector3d &pre_vector3D,
                                  const Eigen::Vector3d &post_vector3D,
                                  double alpha) {
  return pre_vector3D + alpha * (post_vector3D - pre_vector3D);
}

template <typename T>
Eigen::Quaterniond SupereightNode<T>::interpolateOrientation(const Eigen::Quaterniond
                                          &pre_orientation,
                                          const Eigen::Quaterniond &post_orientation,
                                          double
                                          alpha) {
  Eigen::Quaterniond
      int_orientation = pre_orientation.slerp(alpha, post_orientation);
  return int_orientation;
}

template <typename T>
Eigen::Matrix4d SupereightNode<T>::interpolatePose(
    const geometry_msgs::TransformStamped &pre_transformation,
    const geometry_msgs::TransformStamped &post_transformation,
    int64_t img_time_stamp) {

  double alpha =
      calculateAlpha(ros::Time(pre_transformation.header.stamp).toNSec(),
                     ros::Time(post_transformation.header.stamp).toNSec(),
                     img_time_stamp);

  Eigen::Vector3d pre_translation(pre_transformation.transform.translation.x,
                                  pre_transformation.transform.translation.y,
                                  pre_transformation.transform.translation.z);
  Eigen::Vector3d post_translation(post_transformation.transform.translation.x,
                                   post_transformation.transform.translation.y,
                                   post_transformation.transform.translation.z);

  Eigen::Quaterniond pre_rotation(pre_transformation.transform.rotation.w,
                                  pre_transformation.transform.rotation.x,
                                  pre_transformation.transform.rotation.y,
                                  pre_transformation.transform.rotation.z);
  Eigen::Quaterniond post_rotation(post_transformation.transform.rotation.w,
                                   post_transformation.transform.rotation.x,
                                   post_transformation.transform.rotation.y,
                                   post_transformation.transform.rotation.z);

  Eigen::Vector3d inter_translation =
      interpolateVector(pre_translation, post_translation, alpha);
  Eigen::Quaterniond inter_rotation =
      interpolateOrientation(pre_rotation, post_rotation, alpha);

  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 1>(0, 3) = inter_translation;
  pose.block<3, 3>(0, 0) = inter_rotation.toRotationMatrix();

  return pose;
}

/* Taken from https://github.com/ethz-asl/volumetric_mapping */
template <typename T>
std_msgs::ColorRGBA SupereightNode<T>::percentToColor(double h) {
  /* Helen's note: direct copy from OctomapProvider. */
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  /* blend over HSV-values (more colors) */

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  /* if i is even */
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}
} // namespace se

#endif //SUPEREIGHT_SUPEREIGHT_ROS_IMPL_HPP
