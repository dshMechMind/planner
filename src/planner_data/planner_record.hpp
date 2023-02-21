#pragma once

#include <map>
#include <functional>
#include "../base/tree.hpp"
#include "../utility/export.hpp"
#include "../utility/class_forward.hpp"
#include "planner_solution.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(PlannerRecord)

class EXPORT PlannerRecord
{
public:
    State start;
    State goal;
    std::vector<std::pair<Tree, std::string>> trees;

    Tree path2Tree(const Path& path) const;
    void addPath(const Path& path, const std::string& desc) 
    { 
        trees.push_back({path2Tree(path), desc});
    }
    void addTree(const Tree& tree, const std::string& desc) { trees.push_back({tree, desc}); }
    void visualizeCallBack(std::function<void(const PlannerRecord&)> func) const { func(*this); }
};
}
