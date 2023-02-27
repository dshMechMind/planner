#pragma once

#include <algorithm>
#include "../../utility/log_utility.hpp"
#include "../../base/math_utility.hpp"
#include "../../planner_data/planner_solution.hpp"
#include "../local_planner/localPlanner.hpp"

namespace planner
{
class PathSimplifier
{
public:
    struct Solution
    {
        Path pathSimplified;
        Path pathSmoothed;
    };

    PathSimplifier(const LocalPlannerBasePtr& localPlanner, const DistanceBasePtr& distance) :
        localPlanner_(localPlanner), distance_(distance) {}

    // rangeRatio the maximum distance between states a connection is attempted,
    // as a fraction relative to the total number of state (between 0 ~ 1)
    Path reduceVertices(const Path& rawPath, double stepSize, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 10, double rangeRatio = 0.33);
    Path collapseCloseVertices(const Path& rawPath, double stepSize, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 10) const;
    Path smoothBSpline(const Path& rawPath, unsigned int maxSteps, double minChange, double stepSize) const;
    // Add a state at the middle of each segment
    Path subdividePath(const Path& path) const;

private:
    RNG rng_;
    LocalPlannerBasePtr localPlanner_;
    DistanceBasePtr distance_;
};
}
