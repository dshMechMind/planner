#include <math.h>
#include "../utility/log_utility.hpp"
#include "../rrt_simple/rrt_simple.hpp"
#include "../base/math_utility.hpp"

namespace planner {
namespace {
RRTSimpleParam rrtSimpleParam(const PlannerContext& context)
{
    return *std::static_pointer_cast<const RRTSimpleParam>(context.plannerParam).get();
}
}

RRTSimple::RRTSimple(const SpaceInformationPtr& si, const ProblemDefinition& pd, const PlannerContext& context) : 
                    Planner(si, pd), param_(rrtSimpleParam(context)), 
                    nearestNeighbor_(Distance(context.generalParam.bounds)) {}

const PlannerSolution& RRTSimple::solve()
{
    LOGD();
    assert(goal_->type == GoalType::JointTolerance);
    goal_ = std::static_pointer_cast<GoalWithJointTolerance>(goal_);
    opti_ = std::static_pointer_cast<OptiPathLength>(opti_);
    start_.printState("start");
    currentGoal_ = goal_->sample().value();
    currentGoal_.printState("goal");

    solution_.init();
    
    if (!si_->isStateValid(start_))
    {
        std::cout << "start state is not valid" << std::endl;
        solution_.status = PlannerStatus::INVALID_START;
        return solution_;
    } else { std::cout << "start state is valid" << std::endl; }
    if (!si_->isStateValid(currentGoal_))
    {
        std::cout << "goal state is not valid" << std::endl;
        solution_.status = PlannerStatus::INVALID_GOAL;
        return solution_;
    } else { std::cout << "goal state is is valid" << std::endl; }

    auto startVertex = std::make_shared<Vertex>(nullptr, start_);
    tree_.setRoot(startVertex);
    nearestNeighbor_.build(tree_.root());
    unsigned i = 0; 
    Status status = Status::Trapped;
    State qLast = start_;
    LOGD("*************************************");
    while ((i < param_.steps_) && (status != Status::Reached))
    {
        State qSampled = si_->sample();
        // State qSampled = si_->sampleNear(qLast, 0.1);
        if (auto validV = extend(qSampled))
        {
            const VertexPtr vNew = validV.value();
            update(vNew);
            // qLast = vNew->state; // TODO:
            if (goal_->isSatisfied(vNew->state))
                status = Status::Reached;
            else
                status = Status::Advanced;
        } else {
            status = Status::Trapped;
        }

        if (status == Status::Reached)
        {
            solution_.status = PlannerStatus::EXACT_SOLUTION;
            return solution_;
        }
        i++;
    }
    solution_.status = PlannerStatus::TIMEOUT;
    return solution_;
}

std::optional<VertexPtr> RRTSimple::extend(const State& qSampled)
{
    auto vNear = nearestNeighbor_.nearest(qSampled);
    State qNew = newState(qSampled, vNear->state, param_.stepSize_);

    if (si_->isStateValid(qNew))
    {
        if (si_->validInterpolatePath(vNear->state, qNew, param_.localPlannerStepSize_))
            return std::make_shared<Vertex>(vNear.get(), qNew, opti_->stateCost(qNew));
    }
    return {};
}

void RRTSimple::update(const VertexPtr vNew)
{
    tree_.addVertex(vNew);
    nearestNeighbor_.update(vNew);
    solution_.path = generatePath(vNew);
}
State RRTSimple::newState(const State& qSampled, const State& qNear, const double stepSize) const
{
    assert(qSampled.size() == qNear.size());
    State qNew;
    for (size_t i = 0; i < qNear.size(); i++)
    {
        double delta = stepSize * sign(normalizeAngle(qSampled[i] - qNear[i]));
        qNew.push_back(delta + qNear[i]);
    }
    return qNew;
}

Path RRTSimple::generatePath(const VertexPtr v) const
{
    Path path;
    path.push_back(v->state);
    auto currentV = v.get();
    while (auto nextV = currentV->parent())
    {
        path.push_back(nextV->state);
        currentV = nextV;
    }
    std::reverse(std::begin(path), std::end(path));
    LOGD("rrt path size: %ld", path.size());
    return path;
}

void RRTSimple::visualize(std::function<void(const RRTRecord&)> func) const
{
    RRTRecord data(start_, currentGoal_, tree_, solution_.path);
    data.visualizeCallBack(func);
}

void RRTSimple::updateRecord(PlannerRecord& record)
{
    record.start = start_;
    record.goal = currentGoal_;
    record.addTree(tree_, "rrtTree");
    record.addPath(solution_.path, "rawPath");
}
}
