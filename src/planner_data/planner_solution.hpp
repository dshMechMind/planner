#pragma once

#include <vector>
#include "../base/state.hpp"
#include "../base/path.hpp"
#include "../utility/log_utility.hpp"
#include "../utility/export.hpp"
#include "planner_param.hpp"

namespace planner {
enum class PlannerStatus 
{ 
    INVALID_START,
    INVALID_GOAL,
    TIMEOUT,
    APPROXIMATE_SOLUTION,
    EXACT_SOLUTION,
    CRASH,
    ABORT,
    INITIALIZED
 };

struct EXPORT PlannerSolution
{
    PlannerSolution() { init(); }
    
    void init()
    {
        status = PlannerStatus::INITIALIZED;
        path.clear();
        cost = 0;
        approximate = false;
    }
    void printPath() const
    {
        for (size_t i = 0; i < path.size(); i++)
        {
            LOGD("path[%ld]", i);
            path[i].printState();
        }
    }

    PlannerStatus status;
    Path path;
    double cost;
    bool approximate;
};
}
