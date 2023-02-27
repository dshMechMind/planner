#pragma once

#include <vector>
#include <memory>
#include <assert.h>
#include <iostream>
#include <optional>
#include "state.hpp"

namespace planner {
struct Bound
{
    Bound(const double l, const double h)
    {
        assert(l < h);
        b = (h - l) * 0.5;
        offset = (l + h) * 0.5;
    }

    double mappingBack(double data) const { return data + offset; }
    double mapping(double data) const { return data -= offset; }

    double b;
    double offset;
};

class Bounds
{
public:
    Bounds(const std::vector<Bound>& bounds) : bounds_(bounds) {}
    Bounds() {};

    void push_back(const Bound& bound) { bounds_.push_back(bound); }
    const Bound operator[](std::size_t i) const { return bounds_[i]; }
    std::size_t size() const { return bounds_.size(); }
    std::optional<State> stateAffine2Bounds(const State& q) const;
    // [-b, b] ---> [l, h]
    State mappingBack(const State& q) const;
    // [l, h] ---> [-b, b]
    State mapping(const State& q) const;
    void print() const;

private:
    // if theta can affine to [-b, b] return true, else return false
    std::optional<double> affine(double theta, double b) const;
    
    std::vector<Bound> bounds_;
};
}
