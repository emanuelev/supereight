//
// Created by anna on 28/06/19.
//

#ifndef SUPEREIGHT_PLANNER_CONFIG_H
#define SUPEREIGHT_PLANNER_CONFIG_H

#include <vector>
#include <string>


struct Planning_Configuration{
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
  int fov_hor ;


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

  int frontier_cluster_size;
};

inline Planning_Configuration getDefaultPlanningConfig(){
  Planning_Configuration config;
  config.num_cand_views = 20;
  config.robot_safety_radius = 0.5;
  config.fov_hor = 120;
  config.dphi = 10;
  config.dtheta = 10;
  config.clear_sphere_for_planning = true;
  config.clearance_radius = 1.0f;
  config.height_max = 2.5f;
  config.height_min = 1.5f;
  config.skeleton_sample_precision = 0.05;
  config.ompl_solving_time = 2.5;
  config.min_loop_for_termination = 10;
  config.frontier_cluster_size = 20;
  return config;
}
#endif //SUPEREIGHT_PLANNER_CONFIG_H
