#pragma once

#include "../../base/state.hpp"
#include "../../base/tree.hpp"
#include "../distance/distance.hpp"
#include "../../utility/class_forward.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(NearestNeighborBase)
PLANNER_CLASS_FORWARD(NearestNeighbor)

class NearestNeighborBase
{
public:
    virtual void build(const VertexPtr root) = 0;
    virtual void update(const VertexPtr v) = 0;
    virtual const VertexPtr nearest(const State& q) const = 0;
};

class NearestNeighbor : public NearestNeighborBase
{
public:
    NearestNeighbor(const Distance& distance) : NearestNeighborBase(), distance_(distance) {}

    void build(const VertexPtr root) override;
    void update(const VertexPtr v) override;
    const VertexPtr nearest(const State& q) const override;

private:
    Distance distance_;
    std::vector<VertexPtr> vertexes_;
    VertexPtr root_;
};
}
