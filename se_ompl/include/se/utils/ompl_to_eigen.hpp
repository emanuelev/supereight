/**
 * Motion Planning, Convert Ompl path to Eigen path.
 *
 * Copyright (C) 2017 Imperial College London.
 * Copyright (C) 2017 ETH Zürich.
 *
 * @file OmplToEigen.hpp
 *
 * @ingroup common
 *
 * @author Marius Grimm (marius.grimm93@web.de)
 * @date May 22, 2017
 */

#ifndef EXPLORATION_CONVERTOMPLTOEIGEN_HPP
#define EXPLORATION_CONVERTOMPLTOEIGEN_HPP

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include "support_structs.hpp"

namespace se {
namespace exploration {

/**
 * Class for Ompl to Eigen conversions.
 */
class OmplToEigen {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Constructor. */
  OmplToEigen() = default;

  /** Delete copy constructor. */
  OmplToEigen(const OmplToEigen &) = delete;

  /** Use default virtual destructor. */
  virtual ~OmplToEigen() = default;

  /**
   * Convert Ompl path to Eigen path.
   * @param [in] ompl_path Path of type ompl::geometric::PathGeometric.
   * @param [out] eigen_path Path in Eigen format.
   *
   * @note Header of Eigen path message not set in this method.
   */
  static void convertPath(ompl::geometric::PathGeometric &ompl_path,
                          Path<3>::Ptr eigen_path,
                          float radius_m) {
    std::vector<ompl::base::State *> &states = ompl_path.getStates();

    for (ompl::base::State *state : states) {
      State<kDim> state_m;
      Eigen::Vector3f position_m
          (state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
           state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
           state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);

      state_m.segment_end = position_m;
      state_m.segment_radius = radius_m;

      // Save the 3D path in output Eigen format
      eigen_path->states.push_back(state_m);
    }
  };

  static void convertPath(ompl::geometric::PathGeometric &ompl_path,
                          Path_v::Ptr eigen_path,
                          int radius_v) {
    std::vector<ompl::base::State *> &states = ompl_path.getStates();

    for (ompl::base::State *state : states) {
      State_v state_v;
      Eigen::Vector3i position_v
          (static_cast<int>(round(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0])),
           static_cast<int>(round(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1])),
           static_cast<int>(round(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2])));

      state_v.segment = position_v;
      state_v.segment_radius = radius_v;

      // Save the 3D path in output Eigen format
      eigen_path->states.push_back(state_v);
    }
  };
  /**
   * Convert Ompl state to Eigen Vector.
   * @param [in] state Position (state) in ompl::base::State type.
   * @return The converted state as Eigen::Vector3d type.
   *
   * @note Header of Eigen path message not set in this method.
   */
  static Eigen::Vector3f convertState(const ompl::base::State &state) {

    Eigen::Vector3f eigen_point(state.as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                state.as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                state.as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);

    return eigen_point;
  };

  // state [meter] to voxel
  static Eigen::Vector3i convertState_v(const ompl::base::State &state, const float dim) {

    Eigen::Vector3i eigen_point(static_cast<int>(state.as<ompl::base::RealVectorStateSpace::StateType>()->values[0]/dim),
                                static_cast<int>(state.as<ompl::base::RealVectorStateSpace::StateType>()->values[1]/dim),
                                static_cast<int>(state.as<ompl::base::RealVectorStateSpace::StateType>()->values[2]/dim));

    return eigen_point;
  };
  /**
   * Convert Eigen Vector to ompl scoped state.
   * @param [in] state Position (state) as Eigen::Vector type.
   * @param [out] scoped_state Converted state as ompl::base::ScopedState type.
   */
  static void convertState(const Eigen::Vector3f &state,
                           ompl::base::ScopedState<> *scoped_state) {

    (*scoped_state)[0] = state.x();
    (*scoped_state)[1] = state.y();
    (*scoped_state)[2] = state.z();
  };
  static void convertState(const Eigen::Vector3i &state_v,
                           ompl::base::ScopedState<> *scoped_state) {

    (*scoped_state)[0] = state_v.x();
    (*scoped_state)[1] = state_v.y();
    (*scoped_state)[2] = state_v.z();
  };
};

} // namespace exploration
}
#endif //EXPLORATION_CONVERTOMPLTOEIGEN_HPP
