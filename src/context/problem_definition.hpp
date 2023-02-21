#pragma once

#include "../goal/goal.hpp"
#include "../base/state.hpp"
#include "../utility/class_forward.hpp"
#include "optimization_objective.hpp"

namespace planner {
struct ProblemDefinition {
    State start;
    GoalPtr goal; 
    OptimizationObjectivePtr optObj;
};
}
