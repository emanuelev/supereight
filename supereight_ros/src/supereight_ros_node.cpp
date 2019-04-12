#include "supereight_ros/supereight_ros.hpp"

int main(int argc, char** argv){
  using namespace se;
  // initialize ROS nod
  ros::init(argc, argv, "supereight_ros_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  SupereightNode <OFusion>node(nh, nh_private);


  std::cout << "FINISHED" << std::endl;

  ros::spin();

  // delete pipeline;
  return 0;
}
