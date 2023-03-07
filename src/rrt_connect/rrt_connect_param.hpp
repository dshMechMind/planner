#pragma once

#include <cstddef>
#include "../planner_data/planner_param.hpp"

namespace planner {
struct RRTConnectParam : public PlannerParamBase
{
public:
    RRTConnectParam(std::size_t dimension, unsigned int steps, double stepSize,
                   double localPlannerStepSize, double connectionRange) :
        dimension_(dimension), steps_(steps), stepSize_(stepSize), localPlannerStepSize_(localPlannerStepSize),
        connectionRange_(connectionRange) {}

    std::size_t dimension_;
    unsigned int steps_;
    double stepSize_;
    double localPlannerStepSize_;
    double connectionRange_;
};
}

#define RRT_MAX_STEPS (unsigned int)50000
#define RRT_STEP_SIZE (double)0.02
#define LOCAL_PLANNER_SETP_SIZE RRT_STEP_SIZE * 0.5
#define CONNECTION_RANGE (double)0.5
#define COST_THRESHOLD (double)50.0
#define GOAL_THRESHOLD (double)0.0
#define GOAL_BIAS (double)0.05

#define PATH_SIMPLIFIER_MAX_EMPTY_STEPS (unsigned int)10
#define PATH_SIMPLIFIER_REDUCE_VERTICES_MAX_STEPS (unsigned int)15
#define PATH_SIMPLIFIER_REDUCE_VERTICES_RANGE_RATIO (double)0.33
#define PATH_SIMPLIFIER_SMOOTH_BSPLINE_MAX_STEPS (unsigned int)10
#define PATH_SIMPLIFIER_SMOOTH_BSPLINE_MIN_CHANGE LOCAL_PLANNER_SETP_SIZE * 0.05
#define PATH_SIMPLIFIER_COLLAPSE_CLOSE_VERTICES_MAX_STEPS (unsigned int)0

#define SAMPLER_ATTEMPTS (unsigned int)10
