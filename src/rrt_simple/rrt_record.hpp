#pragma once

#include <functional>
#include "../base/state.hpp"
#include "../base/tree.hpp"
#include "../base/path.hpp"

namespace planner {
struct RRTRecord
{
    RRTRecord(const State& s, const State& g, const Tree& t, const Path& p) :
        start(s), goal(g), tree(t), path(p) {}

    void visualizeCallBack(std::function<void(const RRTRecord&)> func) const { func(*this); }

    State start;
    State goal;
    Tree tree;
    Path path;
};
}