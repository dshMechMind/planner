#pragma once

#include <vector>
#include "state_validity_checker.hpp"
#include "dynamic_limit_checker.hpp"
#include "bounds_checker.hpp"
#include "../../utility/class_forward.hpp"
#include "../../utility/export.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(AllStateValidityChecker)

class EXPORT AllStateValidityChecker : public StateValidityChecker
{
public:
    AllStateValidityChecker(const Bounds& b, const StateValidityCheckerPtr& CollisionChecker)
    {
        checkers.push_back(CollisionChecker);
        checkers.push_back(std::make_shared<DynamicLimitChecker>());
        checkers.push_back(std::make_shared<BoundsChecker>(b));
    }

    bool isValid(const State& q) override 
    { 
        for (auto& checker : checkers)
            if (!checker->isValid(q))
                return false;
        return true; 
    }

private:
    std::vector<StateValidityCheckerPtr> checkers;
};
}
