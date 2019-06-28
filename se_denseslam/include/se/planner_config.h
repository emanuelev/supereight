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
  float cand_view_offset;

};

inline Planning_Configuration getDefaultPlanningConfig(){
  Planning_Configuration config;
  config.num_cand_views = 20;
  config.cand_view_offset = 0.5;
  return config;
}
#endif //SUPEREIGHT_PLANNER_CONFIG_H
