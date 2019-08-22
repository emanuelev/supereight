#ifndef PTP_CLEARANCEOBJECTIVE_H
#define PTP_CLEARANCEOBJECTIVE_H

#include <ompl/base/objectives/StateCostIntegralObjective.h>

namespace se {
namespace exploration {
class ClearanceObjective : public ob::StateCostIntegralObjective {
 public:
  explicit ClearanceObjective(const ob::SpaceInformationPtr &si)
      :
      ob::StateCostIntegralObjective(si, true) // true when
  // interpolation active
  {
  }
  ob::Cost stateCost(const ob::State *s) const {
      // cost gets minimized => reciprocal
      return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
  }
};

//    ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
//    {
//      ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
//      ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));
//      ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
//      opt->addObjective(lengthObj, 1.0); // weight
//      opt->addObjective(clearObj, 10.0);
//      return ob::OptimizationObjectivePtr(opt);
//    }
} // namespace exploration
}
#endif //PTP_CLEARANCEOBJECTIVE_H
