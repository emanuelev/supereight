#ifndef EXPLORATION_STATEVALIDITYCHECKER_HPP
#define EXPLORATION_STATEVALIDITYCHECKER_HPP

#include <glog/logging.h>
#include <ctime>
#include <iostream>
#include <stdexcept>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>

#include "se/utils/ompl_to_eigen.hpp"
#include "prob_collision_checker.hpp"
#include "se/occupancy_world.hpp"

namespace ob = ompl::base;

namespace se {
namespace exploration {

template<typename FieldType>
class StateValidityChecker : public ompl::base::StateValidityChecker {
 public:
  StateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                       const ProbCollisionChecker<FieldType> &pcc,
                       const double min_flight_corridor_radius)
      :
      ompl::base::StateValidityChecker(si),
      pcc_(pcc),
      min_flight_corridor_radius_(min_flight_corridor_radius) {}

  virtual bool isValid(const ompl::base::State *state) const {
    if (!si_->satisfiesBounds(state)) {
      return false;
    }
    //if (this->clearance(state) - 1.73 * 0.05 >= min_flight_corridor_radius_)
    if (this->clearance(state) >= min_flight_corridor_radius_)
      return true;
    return false;
  }
  // Returns the distance from the given state's position to the
  // boundary of the circular obstacle.
  double clearance(const ob::State *state) const {
    Eigen::Vector3d position_m = OmplToEigen::convertState(*state);
    return pcc_.getVoxelDistance(position_m);
  }

 private:
  ProbCollisionChecker<FieldType> pcc_;
  double min_flight_corridor_radius_;
  ob::StateSpace *stateSpace_;
};

} // namespace exploration
}
#endif //exploration_STATEVALIDITYCHECKER_HPP

