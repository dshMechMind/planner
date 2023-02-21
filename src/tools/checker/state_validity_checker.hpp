#pragma once

#include "../../base/state.hpp"
#include "../../utility/class_forward.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(StateValidityChecker)

class StateValidityChecker
{
public:
    StateValidityChecker() {}
    virtual ~StateValidityChecker() = default;
    virtual bool isValid(const State& q) = 0; // TODO: const
};
}
