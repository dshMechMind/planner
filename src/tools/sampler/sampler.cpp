#include <math.h>
#include <assert.h>
#include "sampler.hpp"

namespace planner {
State SampleUniform::sample()
{
    State q;
    for (size_t i = 0; i < bounds.size(); i++)
    {
        double d = rng_.uniformReal(-bounds[i].b, bounds[i].b);
        q.push_back(d);
    }
    return bounds.mappingBack(q);
}

State SampleUniform::sampleNear(const State& near, double distance)
{
    assert(near.size() == bounds.size());
    State q;
    State nearMapping = bounds.mapping(near);
    for (size_t i = 0; i < bounds.size(); i++)
    {
        double l = std::max(nearMapping[i] - distance, -bounds[i].b);
        double h = std::min(nearMapping[i] + distance, bounds[i].b);
        double d = rng_.uniformReal(l, h);
        q.push_back(d);
    }
    return bounds.mappingBack(q);
}

State SampleWithBias::sample()
{
    if (rng_.uniformReal01() < bias)
             return qCenter_;
    return SampleUniform::sample();
}

State SampleWithBias::sampleNear(const State& near, double distance)
{
    if (rng_.uniformReal01() < bias)
        return qCenter_;
    return SampleUniform::sampleNear(near, distance);
}
}
