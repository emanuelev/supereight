//
// Created by anna on 18/04/19.
//

#ifndef EXPLORATION_WS_PLANNER_OMPL_HPP
#define EXPLORATION_WS_PLANNER_OMPL_HPP
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <glog/logging.h>

#include "../occupancy_world.hpp"
#include "se/ompl/ompl_setup.hpp"
#include "se/utils/support_structs.hpp"

namespace se {
namespace exploration {

struct PlanningParameter {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Default constructor*/
  PlanningParameter()
      :
      robot_radius_(0.4),
      safety_radius_(0.2),
      solving_time_(1.0),
      simplify_solution_(true),
      skeleton_sample_precision_(0.05),
      use_skeleton_check_(true),
      use_state_validity_checker_(true),
      volume_precision_factor_(2),
      keep_straight_line_path_(true),
      print_trajectory_info_(true) {}

  float robot_radius_;  //[voxel] radius of bounding sphere of UAV
  float solving_time_;  // [s]
  bool simplify_solution_;
  bool ompl_failed_ = false;
  float safety_radius_;
  float min_control_point_radius_;
  float skeleton_sample_precision_;
  bool guarante_all_control_pts_save_;
  bool use_skeleton_check_;
  bool use_state_validity_checker_;
  int path_objective_;
  float time_path_simplification_;
  int volume_precision_factor_;
  bool treat_unknown_as_occupied_;
  bool keep_straight_line_path_;
  // Parameter for trajectory optimization
  float v_max;
  float a_max;
  bool print_trajectory_info_;

  /**
 * Read the configuration and parameters from file.
 * @param [in] file_path Path to file.
 * @return true if file has been read successfully.
 */
  bool ReadPlannerConfigFile(const std::string &file_path) {

    DLOG(INFO) << file_path;
    try {
      YAML::Node parameter = YAML::LoadFile(file_path);
      // Parameter for global planning
      //    std::vector<double > start =
      //    parameter["start"].as<std::vector<double>>();
      //    std::vector<double> goal = parameter["goal"].as<std::vector<double
      //    >>();
      //      Eigen::Vector3d(start.data());
      //    goal = Eigen::Vector3d(goal[0], goal[1], goal[2]);
      safety_radius_ = parameter["safety_radius"].as<float>();
      min_control_point_radius_ = parameter["min_control_point_radius"].as<float>();
      robot_radius_ = parameter["robot_radius"].as<float>();
      skeleton_sample_precision_ = parameter["skeleton_sample_precision"].as<float>();
      guarante_all_control_pts_save_ = parameter["guarante_all_control_pts_save"].as<bool>();
      use_skeleton_check_ = parameter["use_skeleton_check"].as<bool>();
      use_state_validity_checker_ = parameter["use_state_validity_checker"].as<bool>();
      path_objective_ = parameter["path_objective"].as<int>();
      solving_time_ = parameter["solving_time"].as<float>();
      time_path_simplification_ = parameter["time_path_simplification"].as<float>();
      volume_precision_factor_ = parameter["volume_precision_factor"].as<int>();
      treat_unknown_as_occupied_ = parameter["treat_unknown_as_occupied"].as<bool>();
      keep_straight_line_path_ = parameter["keep_straight_line_path"].as<bool>();
      // Parameter for trajectory optimization
      v_max = parameter["v_max"].as<float>();
      a_max = parameter["a_max"].as<float>();
      DLOG(INFO) << "planning parameter read.";
    } catch (const std::exception &ex) {
      LOG(ERROR) << "Failed to read parameter!", ex.what();
    }

    return true;
  }

};
}
}
#endif  // EXPLORATION_WS_PLANNER_OMPL_HPP
