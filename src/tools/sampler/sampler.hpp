#pragma once

#include <optional>
#include "../../base/math_utility.hpp"
#include "../../base/state.hpp"
#include "../../base/bounds.hpp"
#include "../../utility/class_forward.hpp"
#include "../../planner_data/planner_param.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(SamplerBase)
PLANNER_CLASS_FORWARD(SampleUniform)
PLANNER_CLASS_FORWARD(SampleWithBias)
PLANNER_STRUCT_FORWARD(SampleWithBiasParam)


class SamplerBase
{
public:
    SamplerBase(const Bounds& b, unsigned int sampleAttempts) : 
        attempts(sampleAttempts), bounds(b) {}

    virtual State sample() = 0;
    virtual State sample(const State& goal) { return {}; }
    virtual State sampleNear(const State& near, double distance) = 0;

    const unsigned int attempts;
    const Bounds bounds;

protected:
    RNG rng_;
};

class SampleUniform : public SamplerBase
{
public:
    SampleUniform(const Bounds& bounds, unsigned int sampleAttempts) : 
        SamplerBase(bounds, sampleAttempts) {}

    State sample() override;
    State sampleNear(const State& near, double distance) override;
};

struct SampleWithBiasParam : public SamplerParamBase
{
    SampleWithBiasParam(unsigned int sampleAttempts, double b) : 
        SamplerParamBase(sampleAttempts), bias(b) {}

    double bias;
};

class SampleWithBias : public SampleUniform
{
public:
    SampleWithBias(const Bounds& bounds, const State& center, const SampleWithBiasParam& param) : 
        SampleUniform(bounds, param.attempts), bias(param.bias), qCenter_(center)  {}

    State sample() override;
    State sample(const State& goal) override;
    State sampleNear(const State& near, double distance) override;
    
    const double bias;

protected:
    State qCenter_;
};
}
