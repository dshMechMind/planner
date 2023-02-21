#pragma once

#include "../utility/export.hpp"
#include "../base/state.hpp"
#include "../goal/goal.hpp"
#include "../utility/class_forward.hpp"
#include "../tools/distance/distance.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(OptimizationObjective)
PLANNER_CLASS_FORWARD(OptiPathLength)

enum class OptiType { PathLength };

class OptimizationObjective
{
public:
    OptimizationObjective(double costThreshold, OptiType optiType) : threshold(costThreshold), type(optiType) {}
    
    virtual double stateCost(const State& q) const = 0;
    virtual double costToGo(const State& q, const GoalPtr g) const = 0;

    const double threshold;
    const OptiType type;
};

class OptiPathLength : public OptimizationObjective
{
public:
    OptiPathLength(double threshold, const Bounds& b) :
        OptimizationObjective(threshold, OptiType::PathLength), distance_(b) {}
    
    double stateCost(const State& q) const override { return q[0]; } // TODO:
    double costToGo(const State& q, const GoalPtr g) const override
    {
        switch (g->type)
        {
        case GoalType::JointTolerance:
        {
            State goal = g->sample().value();
            double distance = distance_.distance(q, goal);
            return distance < threshold ? distance : std::numeric_limits<double>::infinity();
        }
        default:
            return std::numeric_limits<double>::infinity();
        }
    }

private:
    Distance distance_;
};
}
