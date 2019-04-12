/**
 * Motion Planning, Visualization of octomap, path, trajectory, ... .
 *
 * Copyright (C) 2017 Imperial College London.
 * Copyright (C) 2017 ETH Zürich.
 *
 * @todo LICENSE
 *
 * @file visualization.hpp
 *
 * @author Marius Grimm
 * @date May 26, 2017
 */

#ifndef SUPEREIGHT_ROS_VISUALIZATION_HPP
#define SUPEREIGHT_ROS_VISUALIZATION_HPP

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

//#include "ptp_ros/visualization_bag.hpp"
//#include <ptp/OccupancyWorld.hpp>
//#include <ptp/common.hpp>
//#include <ptp/Header.hpp>
//#include <ptp/Path.hpp>

namespace se {

// Default values
static const std::string kOutputPrefixVis = "[VISUALIZATION]: ";

struct VisualizationBag {
  bool publish_octree;  ///> Publish the octree environment
  bool convert_map;  ///> Convert octree-lib-wip to octomap

  bool visualize_min_z; ///> Visualize from a certain height
  bool visualize_max_z; ///> Visualize up to a certain height
  double min_z; ///> Minimum height for visualization
  double max_z; ///> Maximum height for visualization

  bool visualize_bounded_sdf; ///> Visualize only a subset of the whole sdf
  std::vector<double> min_bound_sdf; ///> Lower x, y and z bounding box values
  std::vector<double> max_bound_sdf; ///> Upper x, y and z bounding box values

  bool visualize_one_sdf_layer_z; ///> Visualize only one sdf layer
  int visulization_layer_z; ///> Number of layer to visualize
  double transparancy; ///> Transparancy of visualized SDF
  bool visualize_collision_potential; ///> Visualize the collision potential or the real esdf values
};

class Visualization {
 public:
  /** The Constructor. */
  Visualization(const VisualizationBag& parameter);

  Visualization() {};

  /** Delete copy constructor. */
  Visualization(const Visualization&) = delete;

  /** Default Destructor. */
  virtual ~Visualization() = default;

  void setParameter(const VisualizationBag& parameter) {
    parameter_ = parameter;
  }

  /**
   * Generate marker arrays for the occupied and free nodes in an Octree
   * @param [in] world The octree environment
   * @param [in] frame_id Frame id of the markers
   * @param [out] occupied_nodes Occupied nodes of the octomap
   * @param [out] free_nodes Free nodes of the octomap
   *
   * @note Modified from https://github.com/ethz-asl/volumetric_mapping
   */
  void generateMarkerArray(const OccupancyWorld::Ptr& ow,
                           const std::string& frame_id,
                           visualization_msgs::MarkerArray* occupied_nodes,
                           visualization_msgs::MarkerArray* free_nodes);

  void generateMapMarkerArray(const OccupancyWorld::Ptr&       ow, 
                              const std::string&               frame_id,
                              visualization_msgs::Marker* occupied_voxels);
//
//  void generateVoxelTubeMarkerArray(const OccupancyWorld::Ptr& ow_,const std::string& frame_id, const Path<kDim>::Ptr path,
//                               visualization_msgs::MarkerArray* tube_nodes);
//
//  void generateControlPointTubeMarkerArray(const OccupancyWorld::Ptr& ow_,const std::string& frame_id, const Path<kDim>::Ptr path,
//                                visualization_msgs::MarkerArray* tube_nodes);
//  void generateSafetyTubeMarkerArray(const OccupancyWorld::Ptr& ow_,const std::string& frame_id, const Path<kDim>::Ptr path,
//                                     PlanningParameter pp, visualization_msgs::MarkerArray* tube_nodes);

  /**
   * Get percentage of height values between min and max height of map
   * @param [in] z Height value to be evaluated
   * @param [in] min_z Minimum height of the map
   * @param [in] max_z Maximum height of the map
   *
   * @note Taken from https://github.com/ethz-asl/volumetric_mapping
   */
  double colorizeMapByHeight(double z, double min_z, double max_z) const;

  /**
   * Convert the percentagewise height to color
   * @param [in] h percentage
   *
   * @note Taken from https://github.com/ethz-asl/volumetric_mapping
   */
  std_msgs::ColorRGBA percentToColor(double h) const;

  /**
   * Helper function to create a std_msgs::ColorRGBA
   * @param [in] r Red color intensity
   * @param [in] g Green color intensity
   * @param [in] b Blue color intensity
   * @param [in] a Transparancy of color
   *
   * @note Taken from https://github.com/ethz-asl/volumetric_mapping
   */
  inline std_msgs::ColorRGBA createColorRGBA(float r, float g,
                                             float b, float a) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
  }

  /**
   * Visualization of an occupancy map.
   * @param [in] collision_map Voxel-grid storing the occupancy.
   * @param [in] frame Frame id of the output marker.
   * @param [in] alpha Transparancy of the output marker.
   *
   * @return Marker for visualization of the occupancy map.
   *
   * @note Modified from https://github.com/UM-ARM-Lab/sdf_tools.
   */

 private:
  VisualizationBag parameter_; ///> Struct storing parameter for visualization.
};

} // namespace se

#endif //SUPEREIGHT_ROS_VISUALIZATION_HPP
