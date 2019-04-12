/**
 * Probabilistic Trajectory Planning ROS wrapper.
 *
 * Copyright (C) 2018 Imperial College London.
 * Copyright (C) 2018 ETH Zürich.
 *
 * @todo LICENSE
 * 
 * @original file motion_planning.cpp
 * @original autor Marius Grimm
 * @original data May 11, 2017
 *
 * @file file ptp_static.cpp
 * @author Nils Funk
 * @date August, 2018
 */

#include <iostream>
#include <vector>

#include <glog/logging.h>
#include "supereight_ros/map_visualizer.hpp"


namespace se {

    MapVisualizer::MapVisualizer(const ros::NodeHandle& nh,
                                 const OccupancyWorld::Ptr ow) :
        nh_(nh),
        ow_(ow),
        visualizer_(parameter.visualization) {
      // Create ROS publisher
      pub_octree_occupied_ = nh_.advertise<visualization_msgs::MarkerArray>(
              parameter_.pub_topic_octree_occupied,
              parameter_.pub_queue_size_octree_occupied);
    }

    void MapVisualizer::generateMapMarkerArray() {
      /* Publish octree occupied and free nodes as visualization_msgs for
         visualization in rviz */
      std::string frame_id = "map";
      visualizer_.generateMarkerArray(ow_, frame_id,
                                      &occupied_map_nodes_, &free_map_nodes_);
    }

    bool MapVisualizer::visualizeMap() {
      generateMapMarkerArray();
      pub_octree_occupied_.publish(occupied_map_nodes_);
      pub_octree_free_.publish(free_map_nodes_);
      return true;
    }

} // namespace se

