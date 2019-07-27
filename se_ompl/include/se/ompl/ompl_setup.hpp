//
// Created by anna on 18/04/19.
//

#ifndef EXPLORATION_WS_MAV_SETUP_HPP
#define EXPLORATION_WS_MAV_SETUP_HPP

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include "se/utils/support_structs.hpp"

namespace se {
namespace exploration {
using namespace ompl;

/**
 * Convert Ompl path to Eigen path.
 * @param [in] ompl_path Path of type ompl::geometric::PathGeometric.
 * @param [out] eigen_path Path in Eigen format.
 *
 * @note Header of Eigen path message not set in this method.
 */
//static void convertPath(ompl::geometric::PathGeometric &ompl_path,
//                        Path<3>::Ptr eigen_path, double radius_m) {
//  std::vector<ompl::base::State *> &states = ompl_path.getStates();
//
//  for (ompl::base::State *state : states) {
//    State<kDim> state_m;
//    Eigen::Vector3d position_m(
//        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
//        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
//        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);
//
//    state_m.segment_end = position_m;
//    state_m.segment_radius = radius_m;
//
//    // Save the 3D path in output Eigen format
//    eigen_path->states.push_back(state_m);
//  }
//};
//
///**
// * Convert Ompl state to Eigen Vector.
// * @param [in] state Position (state) in ompl::base::State type.
// * @return The converted state as Eigen::Vector3d type.
// *
// * @note Header of Eigen path message not set in this method.
// */
//static Eigen::Vector3d convertState(const ompl::base::State &state) {
//  Eigen::Vector3d eigen_point(
//      state.as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
//      state.as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
//      state.as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);
//
//  return eigen_point;
//};
//
///**
// * Convert Eigen Vector to ompl scoped state.
// * @param [in] state Position (state) as Eigen::Vector type.
// * @param [out] scoped_state Converted state as ompl::base::ScopedState type.
// */
//static void convertState(
//    const Eigen::Vector3d &state,
//    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> *scoped_state) {
//  (*scoped_state)->values[0] = state.x();
//  (*scoped_state)->values[1] = state.y();
//  (*scoped_state)->values[2] = state.z();
//};
//
class MavSetup : public geometric::SimpleSetup {
 public:
  MavSetup()
      :
      geometric::SimpleSetup(base::StateSpacePtr(std::make_shared<base::RealVectorStateSpace>(kDim))) {};

//  void setMapCollisionChecking(float robot_radius, map) {
//    std::shared_ptr<MapValidityChecker> validity_checker(
//        new MapValidityChecker(getSpaceInformation(), validity_checker));
//
//    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
//    si_->setMotionValidator(
//        base::MotionValidatorPtr(new MapMotionValidator<se::OFusion>(
//            getSpaceInformation(), validity_checker)));
//  }
};

}  // namespace exploration
}
#endif  // EXPLORATION_WS_MAV_SETUP_HPP
