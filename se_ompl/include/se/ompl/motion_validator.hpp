/**
 * Probabilistic Trajectory Planning, OMPL Motion Validator.
 *
 * Copyright (C) 2018 Imperial College London.
 * Copyright (C) 2018 ETH Zürich.
 *
 * @file MotionValidator.hpp
 * @author Nils Funk
 * @date June 03, 2017
 */

#ifndef EXPLORATION_MOTIONVALIDATOR_HPP
#define EXPLORATION_MOTIONVALIDATOR_HPP

#include <iostream>
#include <stdexcept>
#include <ctime>
#include <glog/logging.h>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>

#include <exploration/ompl/prob_collision_checker.hpp>
#include <exploration/occupancy_world.hpp>
#include <exploration/ompl/OmplToEigen.hpp>


namespace ob = ompl::base;

namespace se {
namespace exploration {

/**
 * Class for interfacing the collsion checking with OMPL.
 *
 * @note Derived from class ompl::base::MotionValidator.
 * @warning
 */

template<typename FieldType>
class MotionValidator : public ompl::base::MotionValidator {
 public:
  MotionValidator(const ompl::base::SpaceInformationPtr &si,
                 ProbCollisionChecker<FieldType> pcc_,
                  const double min_flight_corridor_radius)
      :
      ompl::base::MotionValidator(si),
      pcc_(pcc),
      min_flight_corridor_radius_(min_flight_corridor_radius) {

    stateSpace_ = si_->getStateSpace().get();
    if (stateSpace_ == nullptr)
      throw std::runtime_error("No state space for motion validator");
  }

  /**
   * Checks if the current robot state is valid.
   * @param [in] state The current robot state.
   * @return True if valid, false otherwise.
   */
  virtual bool checkMotion(const ompl::base::State *s1,
                           const ompl::base::State *s2) const override {
    if (!si_->satisfiesBounds(s2)) {
      invalid_++;
      return false;
    }

    Eigen::Vector3d start = OmplToEigen::convertState(*s1);
    Eigen::Vector3d ending = OmplToEigen::convertState(*s2);

    //double sqrt3 = 1.75;
    //double segment_length = (ending - start).norm();
    double corridor_factor;
    //if (segment_length < 0.4) {
    //  corridor_factor = 0.5;
    //} else {
    // corridor_factor = 1;
    //}

    corridor_factor = 1;

    if (!pcc_.checkVoxelDistance(start,
                                  corridor_factor * (0.75 * 0.1 + min_flight_corridor_radius_)))
      return false;

    if (!pcc_.checkVoxelDistance(ending,
                                  corridor_factor * (0.75 * 0.1 + min_flight_corridor_radius_)))
      return false;

    if (pcc_.checkLineDistance(start, ending, corridor_factor * min_flight_corridor_radius_)) {
      return true;
    }
/*
 *
//      LOG(INFO) << "Simplified check of the segment from: x = " << start.x() << " ; y = " << start.y() << " ; z = " << start.z();
//      LOG(INFO) << "                                  to: x = " << ending.x() << " ; y = " << ending.y() << " ; z = " << ending.z();
      clock_t time_req;
      time_req = clock();


      if(pcc_->checkSegmentFlightCorridorSkeleton(start, ending, 0, min_flight_corridor_radius_)) {
//        LOG(INFO) << "Points can be connected";
        time_req = clock() - time_req;
        //std::cout << "Using checkSegmentFlightCorridorSkeleton, it took " << (float)time_req/CLOCKS_PER_SEC << " seconds" << std::endl;
        return true;
        if(pcc_->checkSegmentFlightCorridor(start, ending, 0, min_flight_corridor_radius_)){
          //LOG(INFO) << "Minimum flight corridor can be build";
          valid_++;
          return true;
        }
      }
*/
    invalid_++;
    return false;

  }

  bool checkMotion(const ompl::base::State *s1,
                   const ompl::base::State *s2,
                   std::pair<ompl::base::State *, double> &lastValid) const override {
    std::cout << "Motion Validator 2" << std::endl;
    if (!si_->satisfiesBounds(s2)) {
      invalid_++;
      return false;
    }

    /* assume motion starts in a valid configuration so s1 is valid */
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* temporary storage for the checked state */
    ob::State *test = si_->allocState();
    ob::State *test_prev = si_->allocState();

    for (int j = 1; j <= nd; ++j) {
      stateSpace_->interpolate(s1, s2, (double) j / (double) nd, test);
      stateSpace_->interpolate(s1, s2, (double) (j - 1) / (double) nd, test_prev);

      Eigen::Vector3d start = OmplToEigen::convertState(*test_prev);
      Eigen::Vector3d ending = OmplToEigen::convertState(*test);

      //LOG(INFO) << "Extended check of the segment from: x = " << start.x() << " ; y = " << start.y() << " ; z = " << start.z();
      //LOG(INFO) << "to x = " << ending.x() << " ; y = " << ending.y() << " ; z = " << ending.z();

      if (!pcc_.checkLineDistance(start, ending, 0)) {
        lastValid.second = (double) (j - 1) / (double) nd;
        if (lastValid.first != nullptr)
          stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
        invalid_++;
        si_->freeState(test);
        si_->freeState(test_prev);
        return false;
      }

      if (!pcc_.checkLineDistance(start, ending, min_flight_corridor_radius_)) {
        lastValid.second = (double) (j - 1) / (double) nd;
        if (lastValid.first != nullptr)
          stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
        invalid_++;
        si_->freeState(test);
        si_->freeState(test_prev);
        return false;
      }
      si_->freeState(test);
      si_->freeState(test_prev);
    }

    valid_++;
    return true;
  }

 private:
  ProbCollisionChecker<FieldType> pcc_ ;
  double min_flight_corridor_radius_;
  ob::StateSpace *stateSpace_;
};

} // namespace exploration
}
#endif //exploration_MOTIONVALIDATOR_HPP
