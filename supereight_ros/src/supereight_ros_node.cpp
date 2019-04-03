//#include <ros/ros.h>

#include "supereight_ros/supereight_ros.hpp"

int main(int argc, char** argv){
  using namespace se;
  // initialize ROS nod
  ros::init(argc, argv, "supereight_ros_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  SupereightNode node(nh, nh_private);
  // Configure config file with default values
//  Configuration config;
//  Eigen::Vector2i input_size;
//
//
//  initalizeConfig(config, input_size);
//
//  init_pose = config.initial_pos_factor.cwiseProduct(config.volume_size);
//
//  Eigen::Vector2i computation_size = input_size / config.compute_size_ratio;
//  pipeline = std::shared_ptr<DenseSLAMSystem>(new DenseSLAMSystem(
//          Eigen::Vector2i(computation_size.x(), computation_size.y()),
//          Eigen::Vector3i::Constant(static_cast<int>(config.volume_resolution.x())),
//          Eigen::Vector3f::Constant(config.volume_size.x()),
//          init_pose,
//          config.pyramid, config)
//  );
//  // pipeline = new DenseSLAMSystem(
//  //         Eigen::Vector2i(computation_size.x(), computation_size.y()),
//  //         Eigen::Vector3i::Constant(static_cast<int>(config.volume_resolution.x())),
//  //         Eigen::Vector3f::Constant(config.volume_size.x()),
//  //         init_pose,
//  //         config.pyramid, config);
//
//  DataSynchronizer ds(nh, pipeline, config);

  std::cout << "FINISHED" << std::endl;

  ros::spin();

  // delete pipeline;
  return 0;
}