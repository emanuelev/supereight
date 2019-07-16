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
  float cand_view_safety_radius;

  /**
   * horizontal field of view from gazebo model for depth sensor
   * https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_description/urdf/component_snippets.xacro
   */
  float fov_hor ;
  /**
   * [m]
   */
  float dr;
  /**
   * deg
   */
  int dphi;
  int dtheta;

};

inline Planning_Configuration getDefaultPlanningConfig(){
  Planning_Configuration config;
  config.num_cand_views = 20;
  config.cand_view_safety_radius = 0.5;
  config.fov_hor = 2.0;
  config.dr = 0.1;
  config.dphi = 10;
  config.dtheta = 10;
  return config;
}
#endif //SUPEREIGHT_PLANNER_CONFIG_H
