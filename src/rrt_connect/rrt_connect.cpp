#include <math.h>
#include <utility>
#include "../utility/log_utility.hpp"
#include "../base/math_utility.hpp"
#include "rrt_connect.hpp"

namespace planner {
namespace {
RRTConnectParam rrtConnectParam(const PlannerContext& context)
{
    return *std::static_pointer_cast<const RRTConnectParam>(context.plannerParam).get();
}
}

RRTConnect::RRTConnect(const SpaceInformationPtr& si, const ProblemDefinition& pd, const PlannerContext& context) : 
                    Planner(si, pd), param_(rrtConnectParam(context)), sNearest_(Distance(context.generalParam.bounds)),
                    gNearest_(Distance(context.generalParam.bounds)) {}

const PlannerSolution& RRTConnect::solve()
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

    // start tree start nearest
    auto startVertex = std::make_shared<Vertex>(nullptr, start_);
    sTree_.setRoot(startVertex);
    sNearest_.build(sTree_.root());

    // goal tree goal nearest
    auto goalVertex = std::make_shared<Vertex>(nullptr, currentGoal_);
    gTree_.setRoot(goalVertex);
    gNearest_.build(gTree_.root());

    TreePair start(&sTree_, &sNearest_, TreeType::Start);
    TreePair goal(&gTree_, &gNearest_, TreeType::Goal);

    unsigned i = 0; 
    Status status = Status::Trapped;
    LOGD("*************************************");
    while ((i < param_.steps_) && (status != Status::Reached))
    {
        State qSampled;
        if (start.type == TreeType::Start)
            qSampled= si_->sample(currentGoal_);
        else
            qSampled= si_->sample(start_);

        if (auto validV = extend(qSampled, start.nearest))
        {
            const VertexPtr vNew = validV.value();
            update(vNew, start);
            if (auto near = connectTree(vNew, goal))
            {
                Path path1, path2;
                if (goal.type == TreeType::Goal)
                {
                    path1 = generatePath(vNew);
                    path2 = generatePath(near.value());
                    
                } else {
                    path1 = generatePath(near.value());
                    path2 = generatePath(vNew);
                }
                solution_.path = path1;
                solution_.path.insert(solution_.path.end(), path2.rbegin(), path2.rend());
                status = Status::Reached;
            } else {
                status = Status::Advanced;
            }
                
        } else {
            status = Status::Trapped;
        }

        if (status == Status::Reached)
        {
            solution_.status = PlannerStatus::EXACT_SOLUTION;
            std::cout << "######## sample points: " << i << std::endl;
            return solution_;
        }
        swap(start, goal);
        i++;
    }
    solution_.status = PlannerStatus::TIMEOUT;
    std::cout << "######## sample points: " << i << std::endl;
    return solution_;
}

std::optional<VertexPtr> RRTConnect::extend(const State& qSampled, NearestNeighbor* nearest)
{
    auto vNear = nearest->nearest(qSampled);
    State qNew = newState(qSampled, vNear->state, param_.stepSize_);

    if (si_->isStateValid(qNew))
    {
        if (si_->validInterpolatePath(vNear->state, qNew, param_.localPlannerStepSize_))
            return std::make_shared<Vertex>(vNear.get(), qNew, opti_->stateCost(qNew));
    }
    return {};
}
void RRTConnect::swap(TreePair& t1, TreePair& t2) const
{
    TreePair temp = t1;
    t1 = t2;
    t2 = temp;
}
std::optional<VertexPtr> RRTConnect::connectTree(const VertexPtr& vNew, TreePair& treePair)
{
    State qNew = vNew->state;
    auto vNear = treePair.nearest->nearest(qNew);
    double distance = si_->distance(vNear->state, qNew);
    if (distance > param_.connectionRange_)
        return {};

    if (si_->validInterpolatePath(vNear->state, qNew, param_.localPlannerStepSize_))
    {
        vNear->addChild(vNew);
        return vNear;
    }
    return {};
}

void RRTConnect::update(const VertexPtr vNew, TreePair& treePair)
{
    treePair.tree->addVertex(vNew);
    treePair.nearest->update(vNew);
}
State RRTConnect::newState(const State& qSampled, const State& qNear, const double stepSize) const
{
    assert(qSampled.size() == qNear.size());
    State qNew;

    if (si_->distance(qSampled, qNear) < param_.stepSize_)
        return qSampled;

    for (size_t i = 0; i < qNear.size(); i++)
    {
        double delta = stepSize * sign(normalizeAngle(qSampled[i] - qNear[i]));
        qNew.push_back(delta + qNear[i]);
    }
    return qNew;
}

Path RRTConnect::generatePath(const VertexPtr v) const
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

void RRTConnect::visualize(std::function<void(const RRTConnectRecord&)> func) const
{
    RRTConnectRecord data(start_, currentGoal_, sTree_, gTree_, solution_.path);
    data.visualizeCallBack(func);
}

void RRTConnect::updateRecord(PlannerRecord& record)
{
    record.start = start_;
    record.goal = currentGoal_;
    record.addTree(sTree_, "startTree");
    record.addTree(gTree_, "goalTree");
    record.addPath(solution_.path, "rawPath");
}
}
