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
#include "se/ompl/motion_validator_occupancy_dense.hpp"
#include "se/ompl/motion_validator_occupancy_skeleton.hpp"
#include "se/ompl/state_validity_checker.hpp"
#include "se/ompl/prob_collision_checker.hpp"
#include "se/octree.hpp"
#include "se/utils/helper_functions.hpp"

#include "se/boundary_extraction.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace se {
namespace exploration {

template<typename FieldType>
class PathPlannerOmpl {
 public:
  typedef std::shared_ptr<PathPlannerOmpl> Ptr;

  /**
   * @param [in] ow Map
   * @param ss_ SimplePlanner: Create the set of classes typically needed to solve a geometric problem
   *              StateSpacePtr:  Representation of a space in which planning can be performed. Topology specific sampling, interpolation and distance are defined.
   *              RealVectorStateSpace: A state space representing R^n. The distance function is the L2 norm.
   */
  PathPlannerOmpl(const std::shared_ptr<Octree<FieldType> > &octree_ptr,
                  const std::shared_ptr<ProbCollisionChecker<FieldType> > &pcc,
                  const PlanningParameter &ompl_params,
                  map3i &free_map);

  ~PathPlannerOmpl() {};

  /**
   * Set up the planner.
   * @param [in] start Start position for path planning. [m]
   * @param [in] goal Goal position for path planning. [m]
   */
  bool setupPlanner(const Eigen::Vector3d &start_m, const Eigen::Vector3d &goal_m);
  bool setupPlanner(const Eigen::Vector3i &start_v, const Eigen::Vector3i &goal_v);

  Path<kDim>::Ptr getPathNotSimplified() { return path_not_simplified_; }
  Path_v::Ptr getPathNotSimplified_v() { return path_not_simplified_v_; }
  /**
 * Plan the global path.
 * @param [in] start Start position for path planning. [m]
 * @param [in] goal Goal position for path planning. [m]
 * @return True if straight line planning was successful.
 *TODO: Instead of Eigen::Vector3d use a trajectory point type/message
 */
  bool planPath(const Eigen::Vector3d &start_m, const Eigen::Vector3d &goal_m);
  bool planPath(const Eigen::Vector3i &start_v, const Eigen::Vector3i &goal_v);

  bool start_end_occupied() { return start_end_occupied_; };
  bool ompl_failed() { return ompl_failed_; };

  void prunePath(og::PathGeometric &path);
  void simplifyPath(ompl::geometric::PathGeometric &path);

  Path<kDim>::Ptr getPath() { return path_; };

 private:
  /**
   * Set the space boundaries of the ompl planner from the map boundaries.
   * @param [in] min_boundary Lower boundaries in x, y and z of the map.
   * @param [in] max_boundary Upper boundaries in x, y and z of the map.
   * @param [out] space The ompl space to set the boundaries.
   */

  void setSpaceBoundaries(const Eigen::Vector3i &min_boundary,
                          const Eigen::Vector3i &max_boundary,
                          ompl::base::StateSpacePtr space);

  void reduceToControlPointCorridorRadius(Path<kDim>::Ptr path_m);
  void reduceToControlPointCorridorRadius(Path_v::Ptr path_v);
  /**
   * Prune the path to a minimal set of waypoints.
   * @param [in/out] path The straight-line plan.
   */
  /*
    void prunePath(ompl::geometric::PathGeometric& path);
  */
  /**
   * Reduces and Collapses vertices.
   * @param [in/out] path The straight-line plan.
   */
  /*
    void simplifyPath(ompl::geometric::PathGeometric& path);
  */
  /**
   * Simplifies the path by line-of-sight pruning
   * @param [in/out] path The straight-line plan.
   */
  /*
    void simplifyPathLineOfSight(ompl::geometric::PathGeometric& path);


  */
  ompl::base::PlannerPtr planner_;  /// Shared pointer wrapper for base class for a planner
  std::shared_ptr<ob::RealVectorStateSpace> rvss_ =nullptr;
  og::SimpleSetupPtr ss_; /// Create the set of classes typically needed to solve a
  Path<kDim>::Ptr path_ = nullptr;
  Path<kDim>::Ptr path_not_simplified_ = nullptr;

  Path_v::Ptr path_v_ = nullptr;
  Path_v::Ptr path_not_simplified_v_ = nullptr;

  std::shared_ptr<ProbCollisionChecker<FieldType> > pcc_ = nullptr;
  std::shared_ptr<Octree<FieldType> > octree_ptr_ = nullptr;
  PlanningParameter planning_params_;

  map3i free_map_;

  // geometric
  // problem
  double min_flight_corridor_radius_;
  double robot_radius_;
  double flight_corridor_radius_reduction_;
  double solving_time_;
  bool start_end_occupied_ = false;
  bool ompl_failed_ = false;
};

template<typename FieldType>
PathPlannerOmpl<FieldType>::PathPlannerOmpl(const std::shared_ptr<Octree<FieldType> > &octree_ptr,
                                            const std::shared_ptr<ProbCollisionChecker<FieldType> > &pcc,
                                            const PlanningParameter &ompl_params,
                                            map3i &free_map)
    :
    octree_ptr_(octree_ptr), pcc_(pcc),
//    ss_(ob::StateSpacePtr(std::make_shared<ob::RealVectorStateSpace>(kDim))),
    free_map_(free_map) {
  planning_params_ = ompl_params;
  solving_time_ = ompl_params.solving_time_;
  min_flight_corridor_radius_ = ompl_params.robot_radius_ + ompl_params.safety_radius_
      + ompl_params.min_control_point_radius_;
  DLOG(INFO) << "min flight corridor radius: " << min_flight_corridor_radius_;
  robot_radius_ = ompl_params.robot_radius_;
  flight_corridor_radius_reduction_ = ompl_params.robot_radius_ + ompl_params.safety_radius_;
  ss_ = std::make_shared<og::SimpleSetup>(ob::StateSpacePtr(new ob::RealVectorStateSpace(3)));
  // Contstruct optimizing planner using RRT algorithm
  // set Planner
//    planner_ = std::shared_ptr<og::RRT>(new og::RRT(ss_.getSpaceInformation()));
  planner_ = std::make_shared<og::RRTstar>(ss_->getSpaceInformation());

  path_ = std::make_shared<Path<kDim>>();
  path_not_simplified_ = std::make_shared<Path<kDim>>();

  path_v_ = std::make_shared<Path_v>();
  path_not_simplified_v_ = std::make_shared<Path_v>();
}

/**
 * IMPLEMENTATION
 */


/*
template<typename FieldType>
bool PathPlannerOmpl<FieldType>::setupPlanner(const Eigen::Vector3d &start_m,
                                              const Eigen::Vector3d &goal_m) {
  // Check if octree is valid
  // TODO: Check map sanity
  // ow_->checkSanity();
  DLOG(INFO) << "start setting up planner";
  // Get map boundaries and set space boundaries TODO: Changed this to int [voxel] rather than double [m]
  Eigen::Vector3i min_boundary(0, 0, 0), max_boundary(0, 0, 0);
  // TODO fix this function first
//  ow_->GetMapBoundsMeter(min_boundary, max_boundary);
  getFreeMapBounds(free_map_, min_boundary, max_boundary);
  DLOG(INFO) << "map boundaries: " << min_boundary << ", " << max_boundary;
  setSpaceBoundaries(min_boundary, max_boundary, ss_->getStateSpace());

  // set the object used to check which states in the space are valid
  std::shared_ptr<StateValidityChecker<FieldType> > state_validity_checker;
  state_validity_checker =
      std::shared_ptr<StateValidityChecker<FieldType> >(new StateValidityChecker<FieldType>
          (ss_->getSpaceInformation(),
                                                                                            pcc_,
                                                                                            min_flight_corridor_radius_));
  ss_->setStateValidityChecker(ob::StateValidityCheckerPtr(state_validity_checker));
  ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.001);

  DLOG(INFO) << "use skeletoncheck: " << planning_params_.use_skeleton_check_;
  if (planning_params_.use_skeleton_check_) {
    // Set motion validity checking for this space (collision checking)
    auto motion_validator =
        std::shared_ptr<MotionValidatorOccupancySkeleton<FieldType>>(new MotionValidatorOccupancySkeleton<
            FieldType>(ss_->getSpaceInformation(), pcc_, min_flight_corridor_radius_));
    ss_->getSpaceInformation()->setMotionValidator(motion_validator);
  } else {
    // Set motion validity checking for this space (collision checking)
    auto motion_validator =
        std::shared_ptr<MotionValidatorOccupancyDense<FieldType> >(new MotionValidatorOccupancyDense<
            FieldType>(ss_->getSpaceInformation(), pcc_, min_flight_corridor_radius_));
    ss_->getSpaceInformation()->setMotionValidator(motion_validator);
  }

  // TODO check if it's necessary
//
//  if (!ow_->isVoxelBlockAllocatedMeter(start_m)) {
//    std::cout << "\033[1;31mStart position is not allocated\033[0m\n";
//    LOG(ERROR) << "Start position is not allocated";
//    return false;
//  }
//
//  if (!ow_->isVoxelBlockAllocatedMeter(goal_m)) {
//    std::cout << "\033[1;31mEnd position is not allocated\033[0m\n";
//    LOG(ERROR) << "End position is not allocated";
//    return false;
//  }

  if (!pcc_->isSphereCollisionFree(start_m, min_flight_corridor_radius_)) {
    std::cout << "\033[1;31mStart is occupied\033[0m\n";
    //LOG(ERROR) << "Start is occupied";
    start_end_occupied_ = true;
    return false;
  }

  if (!pcc_->isSphereCollisionFree(goal_m, min_flight_corridor_radius_)) {
    std::cout << "\033[1;31mGoal is occupied\033[0m\n";
    //LOG(ERROR) << "Goal is occupied";
    start_end_occupied_ = true;
    return false;
  }

  // Set the start and goal states
  ob::ScopedState<ob::RealVectorStateSpace> ompl_start(ss_->getSpaceInformation()),
      ompl_goal(ss_->getSpaceInformation());

  OmplToEigen::convertState(start_m, &ompl_start);
  OmplToEigen::convertState(goal_m, &ompl_goal);

  ss_->setStartAndGoalStates(ompl_start, ompl_goal);
  DLOG(INFO) << "start and goal positions set";
  // Set objective
  // TODO ENTROPY stuff
//  ob::OptimizationObjectivePtr information_gain_obj
//      (std::shared_ptr<InformationGainObjective>(new InformationGainObjective(ss_.getSpaceInformation())));
  ob::OptimizationObjectivePtr
      objective(std::make_shared<ob::PathLengthOptimizationObjective>(ss_->getSpaceInformation()));
  ss_->getProblemDefinition()->setOptimizationObjective(objective);
  ss_->getOptimizationObjective()->print(std::cout);
  // Set planner // TODO: Currently fixed to Informend RRT*

  ss_->setPlanner(planner_);
//    ss_.getPlanner()->getProblemDefinition()->getOptimizationObjective()
//    ->print(std::cout);
  // Get more output information
  ss_->setup();

//    ss_.print();

  return true;
}
*/


/**
 * [voxel coord]
 * @tparam FieldType
 * @param start_v
 * @param goal_v
 * @return
 */
template<typename FieldType>
bool PathPlannerOmpl<FieldType>::setupPlanner(const Eigen::Vector3i &start_v,
                                              const Eigen::Vector3i &goal_v) {
  DLOG(INFO) << "start setting up planner voxel based";
  // Get map boundaries and set space boundaries
  Eigen::Vector3i min_boundary(0, 0, 0), max_boundary(0, 0, 0);
  getFreeMapBounds(free_map_, min_boundary, max_boundary);
  DLOG(INFO) << "map boundaries: " << min_boundary.format(InLine) << ", "
             << max_boundary.format(InLine);
  setSpaceBoundaries(min_boundary, max_boundary, ss_->getStateSpace());

  const int min_flight_corridor_radius_v =
      static_cast<int>(min_flight_corridor_radius_ / octree_ptr_->voxelDim());

  // set the object used to check which states in the space are valid only for SDF
/*  std::shared_ptr<StateValidityChecker<FieldType> > state_validity_checker
      (std::make_shared<StateValidityChecker<FieldType> >(ss_.getSpaceInformation(),
                                                          pcc_,
                                                          min_flight_corridor_radius_v));
  ss_.setStateValidityChecker(state_validity_checker);*/


  // This is a fraction of the space extent! Not actual metric units. For
  // mysterious reasons. Thanks OMPL!
//  float validity_checking_resolution = 0.01;
//  if ((max_boundary - min_boundary).norm() > 1e-3) {
//    // If bounds are set, set this to approximately one voxel.
//    validity_checking_resolution =
//        octree_ptr_->voxelDim() / (max_boundary - min_boundary).norm() / 2.0;
//  }
//  ss_.getSpaceInformation()->setStateValidityCheckingResolution(validity_checking_resolution);



/*  DLOG(INFO) << "use skeletoncheck: " << planning_params_.use_skeleton_check_;
  if (planning_params_.use_skeleton_check_) {
    // Set motion validity checking for this space (collision checking)
    auto motion_validator
        (std::make_shared<MotionValidatorOccupancySkeleton<FieldType> >(ss_.getSpaceInformation(),
                                                                        pcc_,
                                                                        min_flight_corridor_radius_v));
    ss_.getSpaceInformation()->setMotionValidator(motion_validator);
  } else {
    // Set motion validity checking for this space (collision checking)
    auto motion_validator
        (std::make_shared<MotionValidatorOccupancyDense<FieldType> >(ss_.getSpaceInformation(),
                                                                     pcc_,
                                                                     min_flight_corridor_radius_v));
    ss_.getSpaceInformation()->setMotionValidator(motion_validator);
  }*/

  // Set objective
//  ob::OptimizationObjectivePtr information_gain_obj
//      (std::shared_ptr<InformationGainObjective>(new InformationGainObjective(ss_.getSpaceInformation())));
  // TODO ??
  ob::OptimizationObjectivePtr
      objective(std::make_shared<ob::PathLengthOptimizationObjective>(ss_->getSpaceInformation()));
  ss_->getProblemDefinition()->setOptimizationObjective(objective);
  ss_->getProblemDefinition()->getOptimizationObjective()->print(std::cout);

  // Set planner // TODO: Currently fixed to Informend RRT*
  ss_->setPlanner(planner_);
  ss_->getPlanner()->printSettings(std::cout);
  ss_->getPlanner()->printProperties(std::cout);

  // TODO voxelblock / node check
  ss_->clear();
  if (!pcc_->isSphereCollisionFree(start_v, min_flight_corridor_radius_v)) {
    std::cout << "\033[1;31mStart is occupied\033[0m\n";
    //LOG(ERROR) << "Start is occupied";
    start_end_occupied_ = true;
    return false;
  }

  if (!pcc_->isSphereCollisionFree(goal_v, min_flight_corridor_radius_v)) {
    std::cout << "\033[1;31mGoal is occupied\033[0m\n";
    //LOG(ERROR) << "Goal is occupied";
    start_end_occupied_ = true;
    return false;
  }

  // Set the start and goal states
  ob::ScopedState<ob::RealVectorStateSpace> ompl_start(ss_->getSpaceInformation());
  ob::ScopedState<ob::RealVectorStateSpace> ompl_goal(ss_->getSpaceInformation());

  OmplToEigen::convertState(start_v, &ompl_start);
  OmplToEigen::convertState(goal_v, &ompl_goal);
  ss_->setStartAndGoalStates(ompl_start, ompl_goal);
  DLOG(INFO) << "start and goal positions set";


  // Get more output information
  ss_->getSpaceInformation()->setup();
  DLOG(INFO) << "ss_.print";
  ss_->print();

  return true;
}

/*template<typename FieldType>
bool PathPlannerOmpl<FieldType>::planPath(const Eigen::Vector3d &start_m,
                                          const Eigen::Vector3d &goal_m) {
  DLOG(INFO) << "start path planner ";
  // Setup the ompl planner
  if (!setupPlanner(start_m, goal_m)) {
    LOG(ERROR) << "Could not set up straight-line planner";
    return false;
  }
  path_->states.clear();
  path_not_simplified_->states.clear();

  // Attempt to solve the problem within x seconds of planning time
  ob::PlannerStatus solved = ss_.solve(solving_time_);
  DLOG(INFO) << ss_.getPlanner()->getName() << " found path : " << solved;
  std::string filename = getTimeStampedFilename();
  std::ofstream myfile(filename);

  if (solved) {
    if (ss_.haveExactSolutionPath()) {

      // Get non-simplified path and convert to Eigen
      og::PathGeometric path = ss_.getSolutionPath();

      // TODO: UNCOMMENTED FOR EVALUATION
      *//*
      OmplToEigen::convertPath(path, path_not_simplified_, min_flight_corridor_radius_);
      pcc_->expandFlightCorridorDistance(path_not_simplified_);
      reduceToControlPointCorridorRadius(path_not_simplified_);
      *//*


      // Simplify path
      prunePath(path);
      // Convert final path to Eigen
      OmplToEigen::convertPath(path, path_, min_flight_corridor_radius_);

      std::cout << "FINAL PATH: ";
      path.printAsMatrix(std::cout);

      if (planning_params_.print_trajectory_info_) {
        std::cout << "Found solution" << std::endl;
        std::cout << "FINAL PATH: ";
        path.printAsMatrix(myfile);
        path.printAsMatrix(std::cout);
        myfile.close();

      }
    } else {
      DLOG(WARNING) << "\033[1;31mONLY APPROXIMATE SOLUTION FOUND. OMPL FAILED"
                       ".\033[0m\n";
      og::PathGeometric path = ss_.getSolutionPath();
      path.printAsMatrix(std::cout);

      path.printAsMatrix(myfile);

      myfile.close();
      DLOG(INFO) << "with an optimization objective value of "
                 << ss_.getProblemDefinition()->getSolutionPath()->cost(ss_.getProblemDefinition()->getOptimizationObjective());
      // evaluate the other possibilities
      ob::PlannerData planner_data(ss_.getSpaceInformation());
      ss_.getPlannerData(planner_data);
      // Start traversing the graph and find the node that gets the closest to the
      // actual goal point.
      if (planner_data.numStartVertices() < 1) {
        DLOG(ERROR) << "No start vertices in RRT!";
        return false;
      } else {
        DLOG(INFO) << "planner data received";
      }
      ompl_failed_ = true;
      return false;
    }
  } else {
    std::cout << "\033[1;31mNO STRAIGHT-LINE SOLUTION FOUND. OMPL FAILED.\033[0m\n";
    ompl_failed_ = true;

    return false;
  }

  return true;
}*/

template<typename FieldType>
bool PathPlannerOmpl<FieldType>::planPath(const Eigen::Vector3i &start_v,
                                          const Eigen::Vector3i &goal_v) {
  DLOG(INFO) << "start path planner voxelblock ";
  // Setup the ompl planner
  if (!setupPlanner(start_v, goal_v)) {
    LOG(ERROR) << "Could not set up straight-line planner";
    return false;
  }
  path_v_->states.clear();
  path_not_simplified_v_->states.clear();

  // Attempt to solve the problem within x seconds of planning time
  ob::PlannerStatus solved = ss_->solve(solving_time_);
  DLOG(INFO) << ss_->getPlanner()->getName() << " found path : " << solved;
  std::string filename = getTimeStampedFilename();
  std::ofstream myfile(filename);

  const int min_flight_corridor_radius_v =
      static_cast<int>(min_flight_corridor_radius_ / octree_ptr_->voxelDim());
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
      prunePath(path);
      // Convert final path to Eigen
      OmplToEigen::convertPath(path, path_v_, min_flight_corridor_radius_v);

      std::cout << "FINAL PATH: ";
      path.printAsMatrix(std::cout);

      if (planning_params_.print_trajectory_info_) {
        std::cout << "Found solution" << std::endl;
        std::cout << "FINAL PATH: ";
        path.printAsMatrix(myfile);
        path.printAsMatrix(std::cout);
        myfile.close();

      }
    } else {
      DLOG(WARNING) << "\033[1;31mONLY APPROXIMATE SOLUTION FOUND. OMPL FAILED"
                       ".\033[0m\n";
      og::PathGeometric path = ss_->getSolutionPath();
      path.printAsMatrix(std::cout);

      path.printAsMatrix(myfile);

      myfile.close();
      DLOG(INFO) << "with an optimization objective value of "
                 << ss_->getProblemDefinition()->getSolutionPath()->cost(ss_->getProblemDefinition
                 ()->getOptimizationObjective());
      // evaluate the other possibilities
      ob::PlannerData planner_data(ss_->getSpaceInformation());
      ss_->getPlannerData(planner_data);
      // Start traversing the graph and find the node that gets the closest to the
      // actual goal point.
      if (planner_data.numStartVertices() < 1) {
        DLOG(ERROR) << "No start vertices in RRT!";
        return false;
      } else {
        DLOG(INFO) << "planner data received";
      }
      ompl_failed_ = true;
      return false;
    }
  } else {
    std::cout << "\033[1;31mNO STRAIGHT-LINE SOLUTION FOUND. OMPL FAILED.\033[0m\n";
    ompl_failed_ = true;

    return false;
  }

  return true;
}

template<typename FieldType>
void PathPlannerOmpl<FieldType>::reduceToControlPointCorridorRadius(Path<kDim>::Ptr path_m) {

  for (auto it_i = path_m->states.begin(); it_i != path_m->states.end(); ++it_i) {
    (*it_i).segment_radius = (*it_i).segment_radius - flight_corridor_radius_reduction_;
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

  if (path.getStateCount() < 3) {
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

template<typename FieldType>
void PathPlannerOmpl<FieldType>::setSpaceBoundaries(const Eigen::Vector3i &min_boundary,
                                                    const Eigen::Vector3i &max_boundary,
                                                    ob::StateSpacePtr space) {
  ob::RealVectorBounds bounds(3);

  bounds.setLow(0, min_boundary.x());
  bounds.setLow(1, min_boundary.y());
  bounds.setLow(2, min_boundary.z());
  bounds.setHigh(0, max_boundary.x());
  bounds.setHigh(1, max_boundary.y());
  bounds.setHigh(2, max_boundary.z());
  ss_->getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(bounds);
}
}
}

#endif //exploration_PathPlannerOmpl_H

