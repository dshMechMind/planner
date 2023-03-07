#pragma once

#include <functional>
#include "../base/state.hpp"
#include "../base/tree.hpp"
#include "../base/path.hpp"

namespace planner {
struct RRTConnectRecord
{
    RRTConnectRecord(const State& s, const State& g, const Tree& startTree, const Tree& goalTree, const Path& p) :
        start(s), goal(g), sTree(startTree), gTree(goalTree), path(p) {}

    void visualizeCallBack(std::function<void(const RRTConnectRecord&)> func) const { func(*this); }

    State start;
    State goal;
    Tree sTree;
    Tree gTree;
    Path path;
};
}