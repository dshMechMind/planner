#pragma once

#include "planner_param.hpp"
#include "../tools/checker/state_validity_checker.hpp"

namespace planner {
enum class PlannerType { RRTSimple, RRTConnect };

struct PlannerContext
{
    PlannerType plannerType;
    SamplerParamBasePtr samplerParam;
    PlannerParamBasePtr plannerParam;
    PlannerGeneralParamters generalParam;
    StateValidityCheckerPtr collisionChecker;
};
}
