//
// Created by anna on 18/04/19.
//

#ifndef EXPLORATION_WS_MAV_SETUP_HPP
#define EXPLORATION_WS_MAV_SETUP_HPP

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/Planner.h>

#include "se/continuous/volume_template.hpp"
#include "se/octree.hpp"
#include "se/utils/eigen_utils.h"

#include "se/utils/support_structs.hpp"
#include "se/ompl/motion_validator_occupancy_skeleton.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace se {
namespace exploration {

template<typename FieldType>
class OmplSetup : public og::SimpleSetup {
 public:
  OmplSetup() : og::SimpleSetup(ob::StateSpacePtr(new ob::RealVectorStateSpace(3))) {};

  void setDefaultObjective() {
    std::cout << "omple setup" << std::endl;
    ob::OptimizationObjectivePtr
        objective(aligned_shared<ob::PathLengthOptimizationObjective>(getSpaceInformation()));
    getProblemDefinition()->setOptimizationObjective(ob::OptimizationObjectivePtr(objective));
  }

  void setDefaultPlanner() {
    setRRTStar();
  }

  void setRRTStar() {
    std::shared_ptr<og::RRTstar> planner = aligned_shared<og::RRTstar>(getSpaceInformation());
    planner->setGoalBias(0.1);
    planner->setNumSamplingAttempts(300);
    // planner->setRange(15);
    setPlanner(planner);
  }

  void setCollisionChecking(const std::shared_ptr<CollisionCheckerV<FieldType> > pcc,
                            const int radius_v) {
    auto motion_validator = aligned_shared<MotionValidatorOccupancySkeleton<FieldType> >(
        getSpaceInformation(),
        pcc,
        radius_v);
    getSpaceInformation()->setMotionValidator(motion_validator);
  }
  const ob::StateSpacePtr &getGStateSpace() const {
    return getStateSpace();
  }
  void prunePath() {

    ob::ProblemDefinitionPtr pdef = getProblemDefinition();
    if (pdef) {
      const ob::PathPtr &p = pdef->getSolutionPath();
      if (p) {
        ompl::time::point start = ompl::time::now();
        og::PathGeometric &path = static_cast<og::PathGeometric &>(*p);
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

  void simplifyPath(og::PathGeometric &path) {

    og::PathSimplifier simplifier(getSpaceInformation());

    // Termination condition
    const double max_time = 0.2; // TODO: parameterize
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(max_time);

    if (path.getStateCount() < 3) {
      return;
    }

    // Try a randomized step of connecting vertices
    bool tryMore = false;
    if (ptc == false) {
      tryMore = simplifier.reduceVertices(path);
    }

    // // Try to collapse close-by vertices
    if (ptc == false) {
      simplifier.collapseCloseVertices(path);
    }

    // Try to reduce vertices some more, if there is any point in doing so
    int times = 0;
    while (tryMore && ptc == false && ++times <= 5) {
      tryMore = simplifier.reduceVertices(path);
    }
  }

  float getPathLength() {
    ob::ProblemDefinitionPtr pdef = getProblemDefinition();
    return pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
  }
  void setSpaceBounds(const Eigen::Vector3i &lower_bound,
                      const Eigen::Vector3i &upper_bound,
                      const float dim) {
    ob::RealVectorBounds bounds(3);

    bounds.setLow(0, lower_bound.x() * dim);
    bounds.setLow(1, lower_bound.y() * dim);
    bounds.setLow(2, 31.2);
    bounds.setHigh(0, upper_bound.x() * dim);
    bounds.setHigh(1, upper_bound.y() * dim);
    bounds.setHigh(2, 32.8);
    si_->getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  }
};

}  // namespace exploration
}
#endif  // EXPLORATION_WS_MAV_SETUP_HPP
