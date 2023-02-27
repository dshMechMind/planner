#include <assert.h>
#include "math_utility.hpp"
#include "bounds.hpp"

namespace planner {
State Bounds::mappingBack(const State& q) const
{
    assert(bounds_.size() == q.size());
    State out;
    for (std::size_t i = 0; i < q.size(); i++)
        out.push_back(bounds_[i].mappingBack(q[i]));
    return out;
}
State Bounds::mapping(const State& q) const
{
    assert(bounds_.size() == q.size());
    State out;
    for (std::size_t i = 0; i < q.size(); i++)
        out.push_back(bounds_[i].mapping(q[i]));
    return out;
}
std::optional<double> Bounds::affine(double theta, double b) const
{
    if (theta >= -b || theta <= b)
        return theta;
    auto out = normalizeAngle(theta);
    if (out >= -b || out <= b)
        return out;
    return {};
}
std::optional<State> Bounds::stateAffine2Bounds(const State& q) const 
{
    assert(q.size() == size());

    State in = mapping(q);
    State out;
    for (std::size_t i = 0; i < in.size(); i++)
    {
        auto temp = affine(in[i], bounds_[i].b);
        if (!temp)
            return {};
        out.push_back(temp.value());
    }
    out = mappingBack(out);
    return out;
}

void Bounds::print() const 
{
    for (const auto& bound : bounds_)
        std::cout << "bound.b " << bound.b << "bound.offset " << bound.offset << std::endl;
}
}
