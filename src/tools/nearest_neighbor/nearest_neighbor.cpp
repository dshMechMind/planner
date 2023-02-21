#include "assert.h"
#include "../../utility/log_utility.hpp"
#include "nearest_neighbor.hpp"

namespace planner {
void NearestNeighbor::build(const VertexPtr root)
{
    vertexes_.clear();
    vertexes_.push_back(root);
    root_ = root;
}

void NearestNeighbor::update(const VertexPtr v)
{
    vertexes_.push_back(v);
}
const VertexPtr NearestNeighbor::nearest(const State& q) const
{
    assert(vertexes_.size() >= 1);
    double cost = __DBL_MAX__;
    unsigned minIndex = 0;
    for (size_t i = 0; i < vertexes_.size(); i++) 
    {
        double dis = distance_.distance(vertexes_[i]->state, q);
        if (cost > dis) 
        {
            cost = dis;
            minIndex = i;
        }
    }
    LOGD("nearestVertex cost: %f, index: %d", cost, minIndex);
    return vertexes_[minIndex];
}
}
