#pragma once

#include "planner_data/planner_solution.hpp"
#include "context/problem_definition.hpp"
#include "context/space_information.hpp"
#include "planner_data/planner_param.hpp"
#include "planner_data/planner_record.hpp"
#include "utility/export.hpp"
#include "context/planner.hpp"
// #include "visualizer/visual_tool.hpp"

namespace planner {
class EXPORT PlannerInterface
{
public:
    PlannerInterface(const PlannerContext& context, const ProblemDefinition& pd);

   bool plan(PlannerSolution& solution);

    bool solved(const PlannerStatus& status) const
    {
        return status == PlannerStatus::APPROXIMATE_SOLUTION ||
               status == PlannerStatus::EXACT_SOLUTION;
    }

private:
    PlannerPtr allocPlanner(const PlannerContext& context);

    PlannerGeneralParamters param_;
    ProblemDefinition pd_;
    PlannerPtr planner_;
    SpaceInformationPtr si_;
    PlannerRecord record_;
    // VisualTool visualTool_;
    // TerminationCondition tc_;
};
}
