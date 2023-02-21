#pragma once

#include <random>
#include <assert.h>

namespace planner {
class RNG
{
public:
    /* Generate a random real within given bounds: [\e lower_bound, \e upper_bound) */
    double uniformReal(double lowerBound, double upperBound)
    {
        assert(lowerBound <= upperBound);
        return (upperBound - lowerBound) * uniDist_(generator_) + lowerBound;
    }
    double uniformReal01() { return uniDist_(generator_); }
    int uniformInt(int lowerBound, int upperBound)
    {
        int r = (int)floor(uniformReal((double)lowerBound, (double)upperBound + 1.0));
        return (r > upperBound) ? upperBound : r;
    }

private:
    std::mt19937 generator_;
    std::uniform_real_distribution<> uniDist_{0.0, std::nextafter(1.0, __DBL_MAX__)}; // [0, 1]
};
}
