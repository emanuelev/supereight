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


};

inline Planning_Configuration getDefaultPlanningConfig(){
  Planning_Configuration config;
  config.num_cand_views = 20;

  return config;
}
#endif //SUPEREIGHT_PLANNER_CONFIG_H
