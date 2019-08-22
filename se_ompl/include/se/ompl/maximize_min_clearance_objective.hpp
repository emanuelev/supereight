#ifndef PTP_MAXIMIZEMINCLEARANCEOBJECTIVE_HPP
#define PTP_MAXIMIZEMINCLEARANCEOBJECTIVE_HPP

#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/tools/config/MagicConstants.h>

namespace se {
namespace exploration {
class MaximizeMinClearanceObjective : public ob::MaximizeMinClearanceObjective {
 public:
  MaximizeMinClearanceObjective(const ob::SpaceInformationPtr &si)
      :
      ob::MaximizeMinClearanceObjective(si) {}

  ob::Cost stateCost(const ob::State *s) const {
    return ob::Cost(this->si_->getStateValidityChecker()->clearance(s));
  }

  bool isCostBetterThan(ob::Cost c1, ob::Cost c2) const {
    return c1.value() > c2.value() + std::numeric_limits<double>::epsilon() * 1e3;
  }

  ob::Cost combineCosts(ob::Cost c1, ob::Cost c2) const {
    if (c1.value() < c2.value())
      return c1;
    else
      return c2;
  }

  ob::Cost motionCost(ob::State *s1, ob::State *s2) const {
    return this->combineCosts(this->stateCost(s1), this->stateCost(s2));
  }

  ob::Cost identityCost() const {
    return ob::Cost(std::numeric_limits<double>::infinity());
  }

  ob::Cost infiniteCost() const {
    return ob::Cost(-std::numeric_limits<double>::infinity());
  }
};

} // namespace exploration
}
#endif //PTP_MAXIMIZEMINCLEARANCEOBJECTIVE_HPP
