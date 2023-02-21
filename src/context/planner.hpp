#pragma once

#include "../base/tree.hpp"
#include "../base/state.hpp"
#include "../goal/goal.hpp"
#include "../planner_data/planner_solution.hpp"
#include "../planner_data/planner_record.hpp"
#include "../planner_data/planner_param.hpp"
#include "../utility/class_forward.hpp"
#include "../tools/checker/state_validity_checker.hpp"
#include "space_information.hpp"
#include "problem_definition.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(Planner)

enum class PlannerType { RRTSimple };

struct PlannerContext
{
    PlannerType plannerType;
    SamplerParamBasePtr samplerParam;
    PlannerParamBasePtr plannerParam;
    PlannerGeneralParamters generalParam;
    StateValidityCheckerPtr collisionChecker;
};

class Planner
{
public:
    Planner(const SpaceInformationPtr& si, const ProblemDefinition& pd) :
        si_(si), start_(pd.start), goal_(pd.goal), opti_(pd.optObj) {}
    virtual ~Planner() = default;

    virtual const PlannerSolution& solve() = 0;
    virtual void updateRecord(PlannerRecord& record) = 0;
    // virtual void visualize() = 0;

protected:
    SpaceInformationPtr si_;
    State start_;
    GoalPtr goal_;
    OptimizationObjectivePtr opti_;
    PlannerSolution solution_;
};
}
