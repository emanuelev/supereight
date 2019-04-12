/**
 * Probabilistic Trajectory Planning top level parameter structure.
 *
 * Copyright (C) 2018 Imperial College London.
 * Copyright (C) 2018 ETH Zürich.
 *
 * @todo LICENSE
 * 
 * @original file motion_planning.hpp
 * @original autor Marius Grimm
 * @original data May 11, 2017
 *
 * @file file ptp_static.hpp
 * @author Nils Funk
 * @date August, 2018
 */


#ifndef SUPEREIGHT_ROS_MAP_VISUALIZER_HPP
#define SUPEREIGHT_ROS_MAP_VISUALIZER_HPP

#include <eigen3/Eigen/Dense>


#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "ptp/common.hpp"

//#include "mav_tube_trajectory_generation/trajectory_sampling.h"
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_visualization/helpers.h>
//#include "mav_tube_trajectory_generation/polynomial_optimization_linear.h"
//#include "mav_tube_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_tube_trajectory_generation/trajectory.h"
//#include "mav_tube_trajectory_generation/timing.h"

#include "ptp_ros/visualization.hpp"
#include <ptp/OccupancyWorld.hpp>

namespace SE {

// Default values
    static const std::string kDefaultPubTopicOctreeOccupied = "octree_occupied";

    static constexpr int kDefaultPubQueueSizeOctreeOccupied  = 1;

    class MapVisualizer {
    public:
        /**
         * The Constructor.
         * @param [in] nh Ros NodeHandle.
         * @param [in] parameter Top level parameter structure with all parameters.
         */
        MapVisualizer(const ros::NodeHandle &nh, 
                  const ParameterBag &parameter);

        /** Delete copy constructor. */
        MapVisualizer(const PTPStatic &) = delete;

        /** Default Destructor. */
        virtual ~MapVisualizer() = default;

        bool visualize_map();

    private:
        void generateMapMarkerArray();
        visualization_msgs::MarkerArray occupied_map_nodes_, free_map_nodes_;

        OccupancyWorld::Ptr ow_ = NULL;

        Visualization visualizer_;
        
        ros::NodeHandle nh_; ///> Ros NodeHandle
        ros::Publisher pub_octree_occupied_;
    };

} // namespace SE

#endif //SUPEREIGHT_ROS_MAP_VISUALIZER_HPP
