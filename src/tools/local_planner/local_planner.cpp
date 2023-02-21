#include "localPlanner.hpp"

namespace planner {
std::optional<State> LocalPlanner::interpolateState(const State& q1, const State& q2, double time) const
{
    assert(q1.size() == q2.size());
    assert((time >= 0 ) && (time <= 1));

    State q;
    for(std::size_t i = 0; i < q1.size(); i++)
        q.push_back((q2[i] - q1[i]) * time + q1[i]);
    return q;
}
std::optional<State> LocalPlanner::validInterpolateState(const State& q1, const State& q2, double time) const
{
    if (auto q = interpolateState(q1, q2, time))
    {
        if (checker_->isValid(q.value()))
        return q;
    }
    return {};
}

std::optional<Path> LocalPlanner::validInterpolatePath(const State& q1, const State& q2, double stepSize) const
{
    Path path;
    unsigned int steps = floor(abs(distance_->distance(q1, q2)) / stepSize);
    double deltaTime = 1.0 / (double)steps;
    path.push_back(q1);
    for (double i = 1; i < steps; i++)
    {
        auto validQ = validInterpolateState(q1, q2, i * deltaTime);
        if (!validQ)
            return {};
        path.push_back(validQ.value());            
    }
    path.push_back(q2);
    return path;
}
}
