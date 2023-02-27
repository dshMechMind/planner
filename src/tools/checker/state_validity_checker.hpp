#pragma once

#include "../../base/state.hpp"
#include "../../utility/class_forward.hpp"
#include "../../utility/export.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(StateValidityChecker)

class EXPORT StateValidityChecker
{
public:
    virtual ~StateValidityChecker() = default;
    virtual bool isValid(const State& q) { return true; }; // TODO: const
};
}
