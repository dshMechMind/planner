#pragma once

#include "../utility/class_forward.hpp"
#include "../base/bounds.hpp"

namespace planner {
PLANNER_STRUCT_FORWARD(PlannerParamBase)
PLANNER_STRUCT_FORWARD(SamplerParamBase)

struct PathSimplifierParamters
{
    double stepSize;
    unsigned int maxEmptySteps;
    unsigned int reduceVerticesMaxSteps;
    double reduceVerticesRangeRatio;
    unsigned int smoothBSplineMaxSteps;
    double smoothBSplineMinChange;
    unsigned int collapseCloseVerticesMaxSteps;
};

struct PlannerGeneralParamters
{
public:
    std::size_t dimension;
    PathSimplifierParamters pathSimplifier;
    Bounds bounds;
};

struct SamplerParamBase 
{
    SamplerParamBase(unsigned int sampleAttempts) : attempts(sampleAttempts) {}
    unsigned int attempts; 
};
struct PlannerParamBase {};
}
