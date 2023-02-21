#pragma once

#include "assert.h"
#include "../../base/math_utility.hpp"
#include "../../utility/class_forward.hpp"
#include "../../base/bounds.hpp"
#include "../../base/state.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(DistanceBase)
PLANNER_CLASS_FORWARD(Distance)

class DistanceBase
{
public:
    DistanceBase(const Bounds& b) : dimension_(b.size()), bounds_(b) {}

    virtual bool isEquivalent(const State& q1, const State& q2) const = 0;
    virtual double distance(const State& q1, const State& q2) const = 0;

    const std::size_t dimension_;
    const Bounds bounds_;
};

class Distance : public DistanceBase
{
public:
    Distance(const Bounds& b) : DistanceBase(b) {}

    double distance(const State& q1, const State& q2) const override;
    bool isEquivalent(const State& q1, const State& q2) const override;
};
}
