#pragma once

#include <vector>
#include <memory>

#include "state.hpp"
#include "../utility/class_forward.hpp"

namespace planner {
PLANNER_STRUCT_FORWARD(Vertex)

struct Vertex
{
public:
    Vertex(Vertex* parent, const State& q, double cost = 1.0) : 
        state(q), stateCost(cost), parent_(parent) {}

    Vertex(const Vertex&) = delete;
    Vertex& operator=(const Vertex&) = delete;

    void addChild(const VertexPtr v, double edgeCost = 1.0)
    { 
        children_.push_back(v);
        edgeCosts_.push_back(edgeCost);
    }
    Vertex* parent() { return parent_; }
    const Vertex* parent() const { return parent_; }
    const std::vector<VertexPtr>& children() const { return children_; }

    const State state;
    const double stateCost;

private:
    Vertex* parent_;
    std::vector<VertexPtr> children_;
    std::vector<double> edgeCosts_;
};

struct Edge
{
    Edge(const Vertex* u, const Vertex* v) : out(u), in(v) {}

    // out---->in
    const Vertex* out;
    const Vertex* in;
};

class Tree
{
public:
    void addVertex(const VertexPtr v, double edgeCost = 1.0) 
    { v->parent()->addChild(v, edgeCost); }

    void setRoot(VertexPtr root) { root_ = std::move(root); }
    const VertexPtr root() const { return root_; }

private:
    VertexPtr root_;
};
}
