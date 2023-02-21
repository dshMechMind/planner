#pragma once

#include <optional>
#include <memory>
#include "../tools/sampler/sampler.hpp"
#include "../tools/checker/state_validity_checker.hpp"
#include "../planner_data/planner_param.hpp"
#include "../utility/class_forward.hpp"
#include "../tools/local_planner/localPlanner.hpp"
#include "../tools/distance/distance.hpp"
#include "../tools/path_simplifier/path_simplifier.hpp"
#include "problem_definition.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(SpaceInformation)

class SpaceInformationBase
{
public:
    struct FlexibleTools
    {
        DistanceBasePtr distance;
        AllStateValidityCheckerPtr checker;
        SamplerBasePtr sampler;
        LocalPlannerBasePtr localPlanner;
    };
    SpaceInformationBase(std::size_t dimension, const FlexibleTools& tools) :
                        dimension_(dimension), distance_(tools.distance), checker_(tools.checker), 
                        sampler_(tools.sampler), localPlanner_(tools.localPlanner), 
                        pathSimplifier_(localPlanner_, distance_) {}
    virtual ~SpaceInformationBase() = default;

    std::optional<State> validSample();
    State sample();
    State sampleNear(const State& near, double distance);

    // const
    double distance(const State& q1, const State& q2) const { return distance_->distance(q1, q2); }
    bool isEquivalent(const State& q1, const State& q2) const { return distance_->isEquivalent(q1, q2); }
    std::size_t dimension() const { return dimension_; }

    // time[0, 1], interpolate without validity check
    std::optional<State> interpolateState(const State& q1, const State& q2, double time) const
    { return localPlanner_->interpolateState(q1, q2, time); }
    // time[0, 1], return true only if entire path (from, to) is valid
    std::optional<State> validInterpolateState(const State& q1, const State& q2, double time) const
    { return localPlanner_->validInterpolateState(q1, q2, time); }
    std::optional<Path> validInterpolatePath(const State& q1, const State& q2, double stepSize) const 
    { return localPlanner_->validInterpolatePath(q1, q2, stepSize); }

    bool isStateValid(const State& q) const { return checker_->isValid(q); } // TODO: const

    virtual PathSimplifier::Solution simplifyPath(const Path& rawPath, const PathSimplifierParamters& param) = 0;

protected:
    std::size_t dimension_;
    DistanceBasePtr distance_;
    AllStateValidityCheckerPtr checker_;
    SamplerBasePtr sampler_;
    LocalPlannerBasePtr localPlanner_;
    PathSimplifier pathSimplifier_;
};

class SpaceInformation : public SpaceInformationBase
{
public:
    SpaceInformation(const PlannerGeneralParamters& param, const SampleWithBiasParam& samplerParam, 
        const ProblemDefinition& pd, const StateValidityCheckerPtr& CollisionChecker) :
        SpaceInformationBase(param.dimension, init(param, samplerParam, pd, CollisionChecker)) {}
    ~SpaceInformation() = default;

    PathSimplifier::Solution simplifyPath(const Path& rawPath, const PathSimplifierParamters& param) override;

private:
    SpaceInformationBase::FlexibleTools init(const PlannerGeneralParamters& param, 
                                             const SampleWithBiasParam& samplerParam, 
                                             const ProblemDefinition& pd, 
                                             const StateValidityCheckerPtr& CollisionChecker);
};
}
