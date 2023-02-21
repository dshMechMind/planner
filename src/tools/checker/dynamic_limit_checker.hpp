#pragma once

#include "state_validity_checker.hpp"
#include "../../utility/class_forward.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(DynamicLimitChecker)

class DynamicLimitChecker : public StateValidityChecker
{
public:
    DynamicLimitChecker() : StateValidityChecker() {}
    bool isValid(const State& q) override { return (bool)q.size(); }; // TODO:
};
}
