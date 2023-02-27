#include <memory>
#include "assert.h"
#include "rrt_simple/rrt_simple.hpp"
#include "rrt_simple/rrt_simple_param.hpp"
#include "base/tree.hpp"
#include "planner_interface.hpp"

namespace planner {
PlannerInterface::PlannerInterface(const PlannerContext& context, const ProblemDefinition& pd) : 
    param_(context.generalParam), pd_(pd) 
{
    planner_ = allocPlanner(context);
}

PlannerPtr PlannerInterface::allocPlanner(const PlannerContext& context)
{
    assert(context.plannerParam != nullptr);
    assert(context.samplerParam != nullptr);

    switch (context.plannerType)
    {
    case PlannerType::RRTSimple:
    {
        auto samplerParam = *std::static_pointer_cast<const SampleWithBiasParam>(context.samplerParam).get();
        si_ = std::make_shared<SpaceInformation>(context.generalParam, samplerParam, pd_, context.collisionChecker);
        RRTSimple rrt(si_, pd_, context);
        return std::make_shared<RRTSimple>(rrt);
    }      
    }
    return nullptr;
}
bool PlannerInterface::plan(PlannerSolution& solution)
{
    LOGD();
    solution.init();

    solution = planner_->solve();
    planner_->updateRecord(record_);
    LOGD("pathRaw size:%d", solution.path.size());
    
    auto simplifierSolution = si_->simplifyPath(solution.path, param_.pathSimplifier);
    solution.path = simplifierSolution.pathSmoothed;

    record_.addPath(simplifierSolution.pathSimplified, "pathSimplified");
    record_.addPath(simplifierSolution.pathSmoothed, "pathSmoothed");

    return solved(solution.status);
}
}
