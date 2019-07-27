/**
 * sets up the exploration pipeline
 */
#ifndef EXPLORATION_LIBRARY_H
#define EXPLORATION_LIBRARY_H

#include <glog/logging.h>
#include <string>

#include "occupancy_world.hpp"
#include "planning_parameter.hpp"
#include "path_planner_ompl.hpp"

namespace se {
namespace exploration {


template<typename FieldType>
class ExplorationPipeline {
 public:
  typedef std::shared_ptr<ExplorationPipeline> Ptr;
  // some functions
  //*.bt binary files
  ExplorationPipeline() {};
  ExplorationPipeline(const std::string &config_path);

  virtual ~ExplorationPipeline() {};
  // initialize the world etc.
  /**
*Read the configuration and parameters from file.
*@param [in] file_path Path to file.
*@return true if file has been read successfully.
*/
  bool ReadPipelineConfigFile(const std::string &file_path);

 private:
  // some functions

  // some variables
  OccupancyWorld::Ptr occupancy_map_ = nullptr;
  Octree<FieldType> octree_;
  ProbCollisionChecker<FieldType> probabilistic_collision_checker_ ;
  std::shared_ptr<PathPlannerOmpl<FieldType> > path_planner_ompl_ ;

  PlanningParameter planning_params_;

  std::string map_path_;
  Eigen::Vector3d start_pos_ = {0.8, 8.8, 0.8};
  Eigen::Vector3d goal_pos_ = {8.6, 0.8, 2.0};

};

template<typename FieldType>
ExplorationPipeline<FieldType>::ExplorationPipeline(const std::string &config_path) {

  ReadPipelineConfigFile(config_path);
  // setup occupancy map
  occupancy_map_ = std::shared_ptr<OccupancyWorld>(new OccupancyWorld());
  DLOG(INFO) << "setup occupancy map";


  // load planner parameters
  planning_params_.ReadPlannerConfigFile(config_path);
//  rrt_->ReadPlannerConfigFile(config_path);

  // load map
  DLOG(INFO) << map_path_;
  occupancy_map_->ReadSupereightMultilevel(map_path_);
  DLOG(INFO) << "map loaded";

  // setup collision checker
  probabilistic_collision_checker_ = ProbCollisionChecker<FieldType>(octree_, planning_params_);

  // set up rrt ompl object
  DLOG(INFO) << "collision checker set";
  path_planner_ompl_ =
      PathPlannerOmpl<FieldType>(octree_, probabilistic_collision_checker_, planning_params_);
  DLOG(INFO) << "ompl object created";

  // Initialize view position with yaw
  // fetch


  /// Plan straight-line path
  if (!path_planner_ompl_->planPath(start_pos_, goal_pos_)) {
    LOG(WARNING) << "Could not plan path";
  } else {
    LOG(INFO) << "Succesfully planned straight-line path";
  }// evaluate possible candidates

  // visualize
}
template<typename FieldType>
bool ExplorationPipeline<FieldType>::ReadPipelineConfigFile(const std::string &file_path) {
  DLOG(INFO) << file_path;
  try {
    YAML::Node parameter = YAML::LoadFile(file_path);

    map_path_ = parameter["map_path"].as<std::string>();

    DLOG(INFO) << "Pipeline parameter read.";
  } catch (const std::exception &ex) {
    LOG(ERROR) << "Failed to read pipeline parameter!", ex.what();
    return false;
  }
  return true;
}
}
}
#endif