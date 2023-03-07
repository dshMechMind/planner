#include "space_information.hpp"

namespace planner {
std::optional<State> SpaceInformationBase::validSample()
{
    State q;
    for (unsigned int i = 0; i < sampler_->attempts; i++)
    {
        q = sampler_->sample();
        if (checker_->isValid(q))
            return q;
    }
    return {};
}

State SpaceInformationBase::sample()
{
    return sampler_->sample();
}

State SpaceInformationBase::sample(const State& goal)
{
    return sampler_->sample(goal);
}

State SpaceInformationBase::sampleNear(const State& near, double distance)
{
    return sampler_->sampleNear(near, distance);
}

SpaceInformationBase::FlexibleTools SpaceInformation::init(const PlannerGeneralParamters& param, 
                                                           const SampleWithBiasParam& samplerParam, 
                                                           const ProblemDefinition& pd, 
                                                           const StateValidityCheckerPtr& CollisionChecker)
{
    SpaceInformationBase::FlexibleTools tools;
    tools.distance = std::make_shared<Distance>(param.bounds);
    tools.sampler = std::make_shared<SampleWithBias>(param.bounds, pd.goal->sample().value(), samplerParam);
    // tools.sampler = std::make_shared<SampleUniform>(param.bounds, samplerParam.attempts);
    tools.checker = std::make_shared<AllStateValidityChecker>(param.bounds, CollisionChecker);
    tools.localPlanner = std::make_shared<LocalPlanner>(tools.distance, tools.checker);
    return tools;
}
PathSimplifier::Solution SpaceInformation::simplifyPath(const Path& rawPath, const PathSimplifierParamters& param)
{
    PathSimplifier::Solution solution;
    solution.pathSimplified = pathSimplifier_.reduceVertices(rawPath, param.stepSize, 
                                                param.reduceVerticesMaxSteps, 
                                                param.maxEmptySteps, 
                                                param.reduceVerticesRangeRatio);
    LOGD("pathSimplified size:%d", solution.pathSimplified.size());
    solution.pathSmoothed = pathSimplifier_.smoothBSpline(solution.pathSimplified, param.smoothBSplineMaxSteps, 
                                                    param.smoothBSplineMinChange, 
                                                    param.stepSize);
    LOGD("pathSmoothed size:%d", solution.pathSmoothed.size());
    return solution;
}
}
