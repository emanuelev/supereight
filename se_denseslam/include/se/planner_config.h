//
// Created by anna on 28/06/19.
//

#ifndef SUPEREIGHT_PLANNER_CONFIG_H
#define SUPEREIGHT_PLANNER_CONFIG_H

#include <vector>
#include <string>

enum PlannerType {
  kRrtConnect = 0, kRrtStar, kInformedRrtStar, kBitStar
};
struct Planning_Configuration {
  /**
   * EXPLORATION
   */

  /**
   * number of candidate views to be evaluated
   */
  int num_cand_views;
  /**
   * distance [m] from frontier wall away. will be converted to voxel distance
   */
  float robot_safety_radius;

  /**
   * horizontal field of view from gazebo model for depth sensor
   * https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_description/urdf/component_snippets.xacro
   */
  int fov_hor;

  /**
   * deg
   */
  int dphi;
  int dtheta;

  bool clear_sphere_for_planning;

  /**
   * [m] set all voxel inside the sphere to voxel state free
   */
  float clearance_radius;

  /**
  * [m] height boundaries
  */
  float height_max;
  float height_min;

  /**
  * [m]
  */
  float skeleton_sample_precision;

  /**
  * [s] ompl solving time
  */
  float ompl_solving_time;

  int min_loop_for_termination;
/**
* [vx] num voxel in a voxel block
*/
  int frontier_cluster_size;

  PlannerType planner_type;
  float ceiling_height;
  float local_minima_radius;

  float planner_dist_max;

  float max_yaw_rate;

  float max_rrt_edge_length;

  bool yaw_optimization;

  int random_generator_seed;
};

inline Planning_Configuration getDefaultPlanningConfig() {
  Planning_Configuration config;
  config.num_cand_views = 20;
  config.robot_safety_radius = 0.3f;
  config.fov_hor = 120;
  config.dphi = 10;
  config.dtheta = 10;
  config.clear_sphere_for_planning = true;
  config.clearance_radius = 1.0f;
  config.height_max = 2.3f;
  config.height_min = 1.0f;
  config.skeleton_sample_precision = 0.05;
  config.ompl_solving_time = 0.5;
  config.min_loop_for_termination = 10;
  config.frontier_cluster_size = 12;
  config.planner_type = kInformedRrtStar;
  config.local_minima_radius = 1.f;
  config.ceiling_height = 3.0f;
  config.planner_dist_max = 2.f;
  config.max_yaw_rate = 0.52;
  config.max_rrt_edge_length = 1.0f;
  config.yaw_optimization = true;
  config.random_generator_seed = 13;
  return config;
}
#endif //SUPEREIGHT_PLANNER_CONFIG_H
