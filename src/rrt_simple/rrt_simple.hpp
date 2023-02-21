#pragma once

#include <optional>
#include "../context/planner.hpp"
#include "../tools/nearest_neighbor/nearest_neighbor.hpp"
#include "rrt_record.hpp"
#include "rrt_simple_param.hpp"

namespace planner {
class RRTSimple : public Planner
{
public:
    enum Status { Reached, Advanced, Trapped };

    RRTSimple(const SpaceInformationPtr& si, const ProblemDefinition& pd,
        const PlannerContext& context);
    ~RRTSimple() override = default;
    
    const PlannerSolution& solve() override;
    void updateRecord(PlannerRecord& record) override;
    void visualize(std::function<void(const RRTRecord&)> func) const;

private:
    std::optional<VertexPtr> extend(const State& qSampled);
    void update(const VertexPtr edge);
    State newState(const State& qSampled, const State& qNear, const double stepSize) const;
    Path generatePath(const VertexPtr v) const;
    
    RRTSimpleParam param_;
    NearestNeighbor nearestNeighbor_;
    Tree tree_;
    State currentGoal_;
};
}
