#include "goal.hpp"

namespace planner {
GoalSingleState::GoalSingleState(const Bounds& b, const State& g) : 
    Goal(b, GoalType::SingleState), distance_(b) 
{ 
    if (!setGoal(g))
        std::cout << "goal out of region" << std::endl;
}
bool GoalSingleState::setGoal(const State& g) 
{ 
    assert(g.size() == dimension_);
    auto q = bounds_.stateAffine2Bounds(g); 
    if (!q)
        return false;
    q_ = bounds_.mappingBack(q.value());
    return true;
}

GoalWithJointTolerance::GoalWithJointTolerance(const Bounds& b, double tolerance, const State& g) :
    Goal(b, GoalType::JointTolerance), distance_(b), tolerance_(tolerance)
{
    if (!setGoal(g))
        std::cout << "goal out of region" << std::endl;
}
bool GoalWithJointTolerance::setGoal(const State& g) 
{ 
    assert(g.size() == dimension_);
    auto q = bounds_.stateAffine2Bounds(g); 
    if (!q)
        return false;
    q_ = q.value();
    return true;
}
bool GoalWithJointTolerance::isSatisfied(const State& q) const
{
    assert(q.size() == dimension_);
    return distance_.distance(q, q_) < tolerance_;
}
}