#include "assert.h"
#include "../../base/math_utility.hpp"
#include "distance.hpp"
#include "../../utility/log_utility.hpp"

namespace planner {
double Distance::distance(const State& q1, const State& q2) const
{
    // Linf norm
    assert(q1.size() == dimension_);
    assert(q2.size() == dimension_);
    
    State qm1 = bounds_.mapping(q1);
    State qm2 = bounds_.mapping(q2);
    double distance = 0.0;
    for (size_t i = 0; i < qm1.size(); i++)
    {
        double residuum = abs(normalizeAngle(qm2[i] - qm1[i])); // TODO bound = 720, compare distance among dis(s1, s2), dis(s1, bound_l), dis(s1, bound_h)
        if (distance < residuum)
            distance = residuum;
    }
    return distance;
}
bool Distance::isEquivalent(const State& q1, const State& q2) const
{
    assert(q1.size() == dimension_);
    assert(q2.size() == dimension_);
    for (std::size_t i = 0; i < dimension_; i++)
    {
        if (q1[i] != q2[i])
            return false;
    }
    return true;
}
}