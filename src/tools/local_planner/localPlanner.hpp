#pragma once

#include <optional>
#include "../../utility/class_forward.hpp"
#include "../../base/path.hpp"
#include "../distance/distance.hpp"
#include "../checker/all_state_validity_checker.hpp"

namespace planner{
PLANNER_CLASS_FORWARD(LocalPlannerBase)
PLANNER_CLASS_FORWARD(LocalPlanner)

class LocalPlannerBase
{
public:
    LocalPlannerBase(const DistanceBasePtr& distance, const AllStateValidityCheckerPtr& checker) : 
        dimension_(distance->dimension_), distance_(distance), checker_(checker) {}
    
    // interpolate without validity check
    virtual std::optional<State> interpolateState(const State& q1, const State& q2, double time) const = 0;
    virtual std::optional<State> validInterpolateState(const State& q1, const State& q2, double time) const = 0;
    // time[0.0, 1.0], return true, if interpolated path is valid
    virtual std::optional<Path> validInterpolatePath(const State& q1, const State& q2, double stepSize) const = 0;      

    const std::size_t dimension_;

protected:
    DistanceBasePtr distance_;
    AllStateValidityCheckerPtr checker_;
};

class LocalPlanner : public LocalPlannerBase
{
public:
    LocalPlanner(const DistanceBasePtr& distance, const AllStateValidityCheckerPtr& checker) : 
        LocalPlannerBase(distance, checker) {}

    std::optional<State> interpolateState(const State& q1, const State& q2, double time) const override;
    std::optional<State> validInterpolateState(const State& q1, const State& q2, double time) const override;
    std::optional<Path> validInterpolatePath(const State& q1, const State& q2, double stepSize) const override;
};
}
