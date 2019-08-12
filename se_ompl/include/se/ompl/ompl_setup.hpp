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
#include "se/utils/planning_parameter.hpp"
#include "se/ompl/motion_validator_occupancy_skeleton.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace se {
namespace exploration {

template<typename FieldType>
class OmplSetup : public og::SimpleSetup {
 public:
  OmplSetup() : og::SimpleSetup(ob::StateSpacePtr(new <base::RealVectorStateSpace>(3))) {};

  void setDefaultObjective() {
    ob::OptimizationObjectivePtr
        objective(aligned_shared<ob::PathLengthOptimizationObjective>(getSpaceInformation()));
    getProblemDefinition()->setOptimizationObjective(ob::setOptimizationObjectivePtr(objective));
  }

  void setDefaultPlanner() {
    setRRTStar();
  }

  void setRRTStar() {
    std::shared_ptr<og::RRTstar> planner = aligned_shared<og::RRTstar>(getSpaceInformation());
    // planner->setGoalBias(0.2);
    planner->setRange(15);
    setPlanner(planner);
  }

  void setCollisionChecking(const std::shared_ptr<Octree<FieldType> > octree_ptr,
                            const std::shared_ptr<CollisionCheckerV<FieldType> > pcc,
                            const int radius_v) {
    auto motion_validator = aligned_shared<MotionValidatorOccupancySkeleton<FieldType> >(
        getSpaceInformation(),
        pcc,
        radius_v);
    getSpaceInformation()->setMotionValidator(motion_validator);
  }
};

}  // namespace exploration
}
#endif  // EXPLORATION_WS_MAV_SETUP_HPP
