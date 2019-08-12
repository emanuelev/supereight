/**
 * Probabilistic Trajectory Safe Flight Corridor Generator.
 *
 * Copyright (C) 2018 Imperial College London.
 * Copyright (C) 2018 ETH Zürich.
 *
 * @todo LICENSE
 *
 *
 * @file file PlanningParameter.hpp
 * @author Nils Funk
 * @date Juli, 2018
 */

#ifndef EXPLORATION_PathPlannerOmpl_H
#define EXPLORATION_PathPlannerOmpl_H

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/Time.h>

#include "se/utils/ompl_to_eigen.hpp"
#include "se/ompl/motion_validator_occupancy_skeleton.hpp"
#include "se/ompl/collision_checker_meter.hpp"
#include "se/ompl/collision_checker_voxel.hpp"
// #include "se/ompl/ompl_setup.hpp"

#include "se/octree.hpp"
#include "se/utils/helper_functions.hpp"

#include "se/boundary_extraction.hpp"
#include "se/utils/eigen_utils.h"
#include "se/path_planning/exploration_utils.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace se {
namespace exploration {

template<typename FieldType>
class PathPlannerOmpl {
 public:
  typedef std::shared_ptr<PathPlannerOmpl> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @param [in] ow Map
   * @param ss_ SimplePlanner: Create the set of classes typically needed to solve a geometric problem
   *              StateSpacePtr:  Representation of a space in which planning can be performed. Topology specific sampling, interpolation and distance are defined.
   *              RealVectorStateSpace: A state space representing R^n. The distance function is the L2 norm.
   */
  PathPlannerOmpl(const std::shared_ptr<Octree<FieldType> > octree_ptr,
                  const std::shared_ptr<CollisionCheckerV<FieldType> > pcc,
                 const Planning_Configuration &planning_config);

  PathPlannerOmpl(const std::shared_ptr<Octree<FieldType> > octree_ptr,
                  const std::shared_ptr<CollisionCheckerM<FieldType> > pcc,
                  const Planning_Configuration &planning_config);
  ~PathPlannerOmpl() {};

  /**
   * Set up the planner.
   * @param [in] start Start position for path planning. [m]
   * @param [in] goal Goal position for path planning. [m]
   */
  bool setupPlanner(const Eigen::Vector3i &start_v,
                    const Eigen::Vector3i &goal_v,
                    const map3i &free_blocks);
  bool setStartGoal(const Eigen::Vector3i &start_v, const Eigen::Vector3i &goal_v);
  bool setStartGoal(const Eigen::Vector3f &start_m, const Eigen::Vector3f &goal_m);

  Path_v::Ptr getPathNotSimplified_v() { return path_not_simplified_v_; }
  /**
 * Plan the global path.
 * @param [in] start Start position for path planning. [m]
 * @param [in] goal Goal position for path planning. [m]
 * @return True if straight line planning was successful.
 *TODO: Instead of Eigen::Vector3d use a trajectory point type/message
 */
  bool planPath();

  bool start_end_occupied() { return start_end_occupied_; };
  bool ompl_failed() { return ompl_failed_; };

  void prunePath(og::PathGeometric &path);
  void simplifyPath(ompl::geometric::PathGeometric &path);

  Path_v::Ptr getPath() { return path_v_; };
  float getPathLength();
  VecPose getPathSegments();
  VecPose getPathSegments_m();

 private:
  /**
   * Set the space boundaries of the ompl planner from the map boundaries.
   * @param [in] min_boundary Lower boundaries in x, y and z of the map.
   * @param [in] max_boundary Upper boundaries in x, y and z of the map.
   * @param [out] space The ompl space to set the boundaries.
   */

  void setSpaceBoundaries();
  void setSpaceBoundaries_m();
  void reduceToControlPointCorridorRadius(Path_v::Ptr path_v);

  std::shared_ptr<og::RRTstar> planner_;  /// Shared pointer wrapper for base class for a planner
  og::SimpleSetupPtr ss_; /// Create the set of classes typically needed to solve a


  Path<kDim>::Ptr path_ = nullptr;
  Path<kDim>::Ptr path_not_simplified_ = nullptr;

  Path_v::Ptr path_v_ = nullptr;
  Path_v::Ptr path_not_simplified_v_ = nullptr;

  // std::shared_ptr<CollisionCheckerV<FieldType> > pcc_ = nullptr;

 std::shared_ptr<CollisionCheckerM<FieldType> > pcc_ = nullptr;

  std::shared_ptr<Octree<FieldType> > octree_ptr_ = nullptr;
  const Planning_Configuration planning_params_;

  // OmplSetup problem_setup_;
  // geometric
  // problem
 float  safety_radius_;

  int min_flight_corridor_radius_v_;

  float solving_time_;
  bool start_end_occupied_ = false;
  bool ompl_failed_ = false;

  // planing bounds, if set
  Eigen::Vector3i lower_bound_v_;
  Eigen::Vector3i upper_bound_v_;

  Eigen::Vector3f lower_bound_;
  Eigen::Vector3f upper_bound_;
};

// template<typename FieldType>
// PathPlannerOmpl<FieldType>::PathPlannerOmpl(const std::shared_ptr<Octree<FieldType> > octree_ptr,
//                                             const std::shared_ptr<CollisionCheckerV<FieldType> > pcc,
//                                             const Planning_Configuration &planning_config)
//     :
//     octree_ptr_(octree_ptr),
//     pcc_(pcc),
//     planning_params_(planning_config),
//     solving_time_(ompl_params.solving_time_),
//     lower_bound_v_(Eigen::Vector3i::Zero()),
//     upper_bound_v_(Eigen::Vector3i::Zero()),
//     ss_(aligned_shared<og::SimpleSetup>(ob::StateSpacePtr(new ob::RealVectorStateSpace(3))))
// //    ss_(ob::StateSpacePtr(std::make_shared<ob::RealVectorStateSpace>(kDim))),
// {
//   min_flight_corridor_radius_v_ = (ompl_params.robot_radius_ + ompl_params.safety_radius_
//       + ompl_params.min_control_point_radius_) / octree_ptr->voxelDim();
//   DLOG(INFO) << "min flight corridor radius: " << min_flight_corridor_radius_v_;
//   robot_radius_ = ompl_params.robot_radius_;
//   flight_corridor_radius_reduction_ = ompl_params.robot_radius_ + ompl_params.safety_radius_;
//   // Contstruct optimizing planner using RRT algorithm
//   // set Planner
// //    planner_ = std::shared_ptr<og::RRT>(new og::RRT(ss_.getSpaceInformation()));
//   planner_ = aligned_shared<og::RRTstar>(ss_->getSpaceInformation());
//   planner_->setGoalBias(0.1);
//   // planner_->setRange(15);
//   planner_->setNumSamplingAttempts(300);
//   path_v_ = aligned_shared<Path_v>();
//   path_not_simplified_v_ = aligned_shared<Path_v>();
// }

template<typename FieldType>
PathPlannerOmpl<FieldType>::PathPlannerOmpl(const std::shared_ptr<Octree<FieldType> > octree_ptr,
                                            const std::shared_ptr<CollisionCheckerM<FieldType> > pcc,
                                            const Planning_Configuration &planning_config)
    :
    octree_ptr_(octree_ptr),
    pcc_(pcc),
    planning_params_(planning_config),
    solving_time_(planning_config.ompl_solving_time),
    lower_bound_(Eigen::Vector3f::Zero()),
    upper_bound_(Eigen::Vector3f::Zero()),
    ss_(aligned_shared<og::SimpleSetup>(ob::StateSpacePtr(new ob::RealVectorStateSpace(3))))
//    ss_(ob::StateSpacePtr(std::make_shared<ob::RealVectorStateSpace>(kDim))),
{

    // min_flight_corridor_radius_v_ = static_cast<float>(ompl_params.robot_radius_ + ompl_params.safety_radius_
      // + ompl_params.min_control_point_radius_) / octree_ptr->voxelDim();

  safety_radius_ = planning_config.robot_safety_radius;
  // Contstruct optimizing planner using RRT algorithm
  // set Planner
//    planner_ = std::shared_ptr<og::RRT>(new og::RRT(ss_.getSpaceInformation()));
  planner_ = aligned_shared<og::RRTstar>(ss_->getSpaceInformation());
  planner_->setGoalBias(0.1);
  // planner_->setRange(15);
  planner_->setNumSamplingAttempts(300);
  path_ = aligned_shared<Path<kDim> >();
  path_not_simplified_ = aligned_shared<Path<kDim> >();
}

/**
 * IMPLEMENTATION
 */




template<typename FieldType>
bool PathPlannerOmpl<FieldType>::setStartGoal(const Eigen::Vector3i &start_v,
                                              const Eigen::Vector3i &goal_v) {

  std::chrono::time_point<std::chrono::steady_clock> timings[3];
  // double normal_time = std::chrono::duration_cast<std::chrono::duration<double> >(timings[2]
  // -timings[1]).count();

  if (!pcc_->isSphereSkeletonFree(start_v, safety_radius_)) {
    std::cout << "\033[1;31mStart at " << start_v.format(InLine) << " is occupied "
              << octree_ptr_->get(start_v).x << "\033[0m\n";
    //LOG(ERROR) << "Start is occupied";
    // start_end_occupied_ = true;
    // return false;
  }

  if (!pcc_->isSphereSkeletonFree(goal_v, safety_radius_)) {
    std::cout << "\033[1;31mGoal at " << goal_v.format(InLine) << " is occupied "
              << octree_ptr_->get(goal_v).x << "\033[0m\n";
    //LOG(ERROR) << "Goal is occupied";
    // start_end_occupied_ = true;
    // return false;
  }
  // Set the start and goal states
  ob::ScopedState<> start_ompl(ss_->getSpaceInformation());
  ob::ScopedState<> goal_ompl(ss_->getSpaceInformation());

  OmplToEigen::convertState(start_v, &start_ompl);
  OmplToEigen::convertState(goal_v, &goal_ompl);
  ss_->setStartAndGoalStates(start_ompl, goal_ompl, 0.05);

  return true;
}

template<typename FieldType>
bool PathPlannerOmpl<FieldType>::setStartGoal(const Eigen::Vector3f &start_m,
                                              const Eigen::Vector3f &goal_m) {

  std::chrono::time_point<std::chrono::steady_clock> timings[3];
  // double normal_time = std::chrono::duration_cast<std::chrono::duration<double> >(timings[2]
  // -timings[1]).count();

  if (!pcc_->isSphereSkeletonFree(start_m, safety_radius_)) {
    std::cout << "\033[1;31mStart at " << start_m.format(InLine) << " is occupied "
              << octree_ptr_->get(toVoxelCoord(start_m, octree_ptr_->voxelDim())).x << "\033[0m\n";
    //LOG(ERROR) << "Start is occupied";
    // start_end_occupied_ = true;
    // return false;
  }

  if (!pcc_->isSphereSkeletonFree(goal_m, safety_radius_)) {
    std::cout << "\033[1;31mGoal at " << goal_m.format(InLine) << " is occupied "
              << octree_ptr_->get(toVoxelCoord(goal_m, octree_ptr_->voxelDim())).x << "\033[0m\n";
    //LOG(ERROR) << "Goal is occupied";
    // start_end_occupied_ = true;
    // return false;
  }
  // Set the start and goal states
  ob::ScopedState<> start_ompl(ss_->getSpaceInformation());
  ob::ScopedState<> goal_ompl(ss_->getSpaceInformation());
  // start_ompl[0] = start_m.x();
  // start_ompl[1] = start_m.y();
  // start_ompl[2] = start_m.z();

  // goal_ompl[0] = goal_m.x();
  // goal_ompl[1] = goal_m.y();
  // goal_ompl[2] = goal_m.z();
  OmplToEigen::convertState(start_m, &start_ompl);
  OmplToEigen::convertState(goal_m, &goal_ompl);
  ss_->setStartAndGoalStates(start_ompl, goal_ompl, 0.05);

  return true;
}
/**
 * [voxel coord]
 * @tparam FieldType
 * @param start_v
 * @param goal_v
 * @return
 */
template<typename FieldType>
bool PathPlannerOmpl<FieldType>::setupPlanner(const Eigen::Vector3i &start_v,
                                              const Eigen::Vector3i &goal_v,
                                              const map3i &free_blocks) {
  LOG(INFO) << "start setting up planner voxel based";
  ss_->clear();
  // TODO to be replaced
  // Get map boundaries and set space boundaries
  getFreeMapBounds(octree_ptr_, free_blocks, lower_bound_v_, upper_bound_v_);
  // setSpaceBoundaries();
  setSpaceBoundaries_m();


  // Set motion validity checking for this space (collision checking)
  // auto motion_validator =
  //     aligned_shared<MotionValidatorOccupancySkeleton<FieldType> >(ss_->getSpaceInformation(),
  //                                                                  pcc_,
  //                                                                  min_flight_corridor_radius_v_);

    auto motion_validator =
      aligned_shared<MotionValidatorOccupancySkeleton<FieldType> >(ss_->getSpaceInformation(),
                                                                   pcc_,
                                                                   safety_radius_);
  ss_->getSpaceInformation()->setMotionValidator(motion_validator);

  // Set objective
//  ob::OptimizationObjectivePtr information_gain_obj
//      (std::shared_ptr<InformationGainObjective>(new InformationGainObjective(ss_.getSpaceInformation())));
  // TODO ??
  ob::OptimizationObjectivePtr
      objective(aligned_shared<ob::PathLengthOptimizationObjective>(ss_->getSpaceInformation()));
  ss_->getProblemDefinition()->setOptimizationObjective(objective);

  // Set planner // TODO: Currently fixed to Informend RRT*
  ss_->setPlanner(planner_);
  // ss_->getSpaceInformation()->getStateSpace()->setLongestValidSegmentFraction(0.1);

  // TODO voxelblock / node check
//  ss_->clear();
  Eigen::Vector3f start_m = start_v.cast<float>() * octree_ptr_->voxelDim();
  Eigen::Vector3f goal_m = goal_v.cast<float>()*octree_ptr_->voxelDim();
  if (!setStartGoal(start_m, goal_m)) {
    LOG(INFO) << "No start and goal set";
    return false;
  }
//  ss_->print(std::cout);


  return true;
}

template<typename FieldType>
bool PathPlannerOmpl<FieldType>::planPath() {
  DLOG(INFO) << "start path planner voxelblock ";
  // Setup the ompl planner

  ss_->print(std::cout);

  path_->states.clear();
  path_not_simplified_->states.clear();
  // path_v_->states.clear();
  // path_not_simplified_v_->states.clear();
  // Attempt to solve the problem within x seconds of planning time
  ob::PlannerStatus solved = ss_->solve(solving_time_);
  LOG(INFO) << ss_->getPlanner()->getName() << " found path : " << solved;
//  std::string filename = getTimeStampedFilename();
//  std::ofstream myfile(filename);

  if (solved) {
    if (ss_->haveExactSolutionPath()) {

      // Get non-simplified path and convert to Eigen
      og::PathGeometric path = ss_->getSolutionPath();

      // TODO: UNCOMMENTED FOR EVALUATION
      /*
      OmplToEigen::convertPath(path, path_not_simplified_, min_flight_corridor_radius_);
      pcc_->expandFlightCorridorDistance(path_not_simplified_);
      reduceToControlPointCorridorRadius(path_not_simplified_);
      */

      // Simplify path
      // prunePath(path);
      // Convert final path to Eigen
      // OmplToEigen::convertPath(path, path_v_, min_flight_corridor_radius_v_);
      OmplToEigen::convertPath(path, path_, safety_radius_);
      // std::cout << "FINAL PATH: ";
      // path.printAsMatrix(std::cout);

      if (true) {
        std::cout << "Found solution" << std::endl;
        std::cout << "FINAL PATH: ";
//        path.printAsMatrix(myfile);
        path.printAsMatrix(std::cout);
//        myfile.close();

      }
    } else {
      LOG(WARNING) << "\033[1;31mONLY APPROXIMATE SOLUTION FOUND. OMPL FAILED"
                      ".\033[0m\n";
      og::PathGeometric path = ss_->getSolutionPath();
      path.printAsMatrix(std::cout);
      const double dist_to_goal = ss_->getProblemDefinition()->getSolutionDifference();
      LOG(INFO) << "solution difference " << dist_to_goal;
      if(dist_to_goal>= 0.5){
        return false;
      }
//      path.printAsMatrix(myfile);
//
//      myfile.close();
      LOG(INFO) << "with an optimization objective value of "
                << ss_->getProblemDefinition()->getSolutionPath()->cost(ss_->getProblemDefinition()->getOptimizationObjective());
      // evaluate the other possibilities
      // ss_->getProblemDefinition()->print(std::cout);

      ob::PlannerData planner_data(ss_->getSpaceInformation());
      ss_->getPlannerData(planner_data);
      // OmplToEigen::convertPath(path, path_v_, min_flight_corridor_radius_v_);
      OmplToEigen::convertPath(path, path_, safety_radius_);

      // Start traversing the graph and find the node that gets the closest to the
      // actual goal point.
      if (planner_data.numStartVertices() < 1) {
        LOG(ERROR) << "No start vertices in RRT!";
        return false;
      } else {
        LOG(INFO) << "planner data received";
      }
      ompl_failed_ = true;
      return true;
    }
  } else {
    std::cout << "\033[1;31mNO STRAIGHT-LINE SOLUTION FOUND. OMPL FAILED.\033[0m\n";
    ompl_failed_ = true;

    return false;
  }

  return true;
}

template<typename FieldType>
void PathPlannerOmpl<FieldType>::reduceToControlPointCorridorRadius(Path_v::Ptr path_v) {

  for (auto it_i = path_v->states.begin(); it_i != path_v->states.end(); ++it_i) {
    (*it_i).segment_radius = (*it_i).segment_radius - safety_radius_;
  }
}

template<typename FieldType>
void PathPlannerOmpl<FieldType>::prunePath(og::PathGeometric &path) {

  ob::ProblemDefinitionPtr pdef = ss_->getProblemDefinition();
  if (pdef) {
    const ob::PathPtr &p = pdef->getSolutionPath();
    if (p) {
      ompl::time::point start = ompl::time::now();
      std::size_t numStates = path.getStateCount();

      // Simplify
      simplifyPath(path);

      double simplifyTime = ompl::time::seconds(ompl::time::now() - start);
      OMPL_INFORM("SimpleSetup: Path simplification took %f seconds and "
                  "changed from %d to %d states", simplifyTime, numStates, path.getStateCount());
      return;
    }
  }
  OMPL_WARN("No solution to simplify");
}

template<typename FieldType>
void PathPlannerOmpl<FieldType>::simplifyPath(ompl::geometric::PathGeometric &path) {

  og::PathSimplifier simplifier(ss_->getSpaceInformation());

  // Termination condition
  const double max_time = 0.5; // TODO: parameterize
  ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(max_time);

  if (path.getStateCount() < 5) {
    return;
  }

  // Try a randomized step of connecting vertices
  bool tryMore = false;
  if (ptc == false) {
    tryMore = simplifier.reduceVertices(path);
  }

  // Try to collapse close-by vertices
  if (ptc == false) {
    simplifier.collapseCloseVertices(path);
  }

  // Try to reduce vertices some more, if there is any point in doing so
  int times = 0;
  while (tryMore && ptc == false && ++times <= 5) {
    tryMore = simplifier.reduceVertices(path);
  }
}
/**
* set the state space boundaries in voxel coord
*/

template<typename FieldType>
void PathPlannerOmpl<FieldType>::setSpaceBoundaries() {
  ob::RealVectorBounds bounds(3);

  bounds.setLow(0, lower_bound_v_.x());
  bounds.setLow(1, lower_bound_v_.y());
  bounds.setLow(2, lower_bound_v_.z());
  bounds.setHigh(0, upper_bound_v_.x());
  bounds.setHigh(1, upper_bound_v_.y());
  bounds.setHigh(2, upper_bound_v_.z());
  ss_->getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(bounds);
}
/**
* set the state space boundaries in map units
*/
template<typename FieldType>
void PathPlannerOmpl<FieldType>::setSpaceBoundaries_m() {
  ob::RealVectorBounds bounds(3);
  const float dim = octree_ptr_->voxelDim();

  lower_bound_ = lower_bound_v_.cast<float>() * dim;
  upper_bound_ = upper_bound_v_.cast<float>() * dim;
  bounds.setLow(0, lower_bound_v_.x()*dim);
  bounds.setLow(1, lower_bound_v_.y()*dim);
  bounds.setLow(2, planning_params_.height_min);
  bounds.setHigh(0, upper_bound_v_.x()*dim);
  bounds.setHigh(1, upper_bound_v_.y()*dim);
  bounds.setHigh(2, planning_params_.height_max);
  ss_->getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(bounds);
}

template<typename FieldType>
float PathPlannerOmpl<FieldType>::getPathLength() {
  return ss_->getProblemDefinition()->getSolutionPath()->cost(ss_->getProblemDefinition()->getOptimizationObjective()).value();
}

template<typename FieldType>
VecPose PathPlannerOmpl<FieldType>::getPathSegments() {
  VecPose path;
  for (auto it_i = path_v_->states.begin(); it_i != path_v_->states.end(); ++it_i) {
    pose3D tmp({0.f, 0.f, 0.f}, {1.0f, 0.f, 0.f, 0.f});
    tmp.p = (*it_i).segment.cast<float>();
    path.push_back(tmp);
  }
  return path;
}

/*
* getting the planned path [m]
* return path in voxel coord
*/

template<typename FieldType>
VecPose PathPlannerOmpl<FieldType>::getPathSegments_m() {
  VecPose path;
  for (auto it_i = path_->states.begin(); it_i != path_->states.end(); ++it_i) {
    pose3D tmp({0.f, 0.f, 0.f}, {1.0f, 0.f, 0.f, 0.f});

    tmp.p = (*it_i).segment_end/octree_ptr_->voxelDim();
    path.push_back(tmp);
  }
  return path;
}


} // exploration
} // se

#endif //exploration_PathPlannerOmpl_H

