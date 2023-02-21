#include "../utility/log_utility.hpp"
#include "planner_record.hpp"

namespace planner {
Tree PlannerRecord::path2Tree(const Path& path) const
{
    Tree tree;
    if (path.size() < 1)
        return tree;
    auto vStart = std::make_shared<Vertex>(nullptr, path[0]);
    VertexPtr vLast =  vStart;
    tree.setRoot(vStart);
    
    for (std::size_t i = 1; i < path.size(); i++)
    {
        auto vNext = std::make_shared<Vertex>(vLast.get(), path[i]);
        tree.addVertex(vNext);
        vLast = vNext;
    }
    return tree;
}
}
