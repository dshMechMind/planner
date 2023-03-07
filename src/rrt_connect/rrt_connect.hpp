#pragma once

#include <optional>
#include "../context/planner.hpp"
#include "../tools/nearest_neighbor/nearest_neighbor.hpp"
#include "rrt_connect_record.hpp"
#include "rrt_connect_param.hpp"

namespace planner {
class RRTConnect : public Planner
{
public:
    enum class Status { Reached, Advanced, Trapped };

    RRTConnect(const SpaceInformationPtr& si, const ProblemDefinition& pd,
        const PlannerContext& context);
    ~RRTConnect() override = default;
    
    const PlannerSolution& solve() override;
    void updateRecord(PlannerRecord& record) override;
    void visualize(std::function<void(const RRTConnectRecord&)> func) const;

private:
    enum class TreeType { Start, Goal };

    struct TreePair
    {
        TreePair(Tree* t, NearestNeighbor* n, TreeType treeType) :
            tree(t), nearest(n), type(treeType) {}

        Tree* tree;
        NearestNeighbor* nearest;
        TreeType type;
    };

    std::optional<VertexPtr> extend(const State& qSampled, NearestNeighbor* nearest);
    void update(const VertexPtr vNew, TreePair& treePair);
    State newState(const State& qSampled, const State& qNear, const double stepSize) const;
    Path generatePath(const VertexPtr v) const;
    std::optional<VertexPtr> connectTree(const VertexPtr& vNew, TreePair& treePair);
    void swap(TreePair& t1, TreePair& t2) const;
    
    RRTConnectParam param_;
    NearestNeighbor sNearest_;
    NearestNeighbor gNearest_;
    Tree sTree_;
    Tree gTree_;
    State currentGoal_;
};
}
