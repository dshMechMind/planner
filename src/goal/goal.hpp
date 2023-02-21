#pragma once

#include <optional>
#include <Eigen/Geometry>
#include "assert.h"
#include "../base/state.hpp"
#include "../tools/distance/distance.hpp"

namespace planner {
PLANNER_CLASS_FORWARD(Goal)
PLANNER_CLASS_FORWARD(GoalSingleState)
PLANNER_CLASS_FORWARD(GoalWithJointTolerance)

enum class GoalType { SingleState, JointTolerance, BoundingBox, SO3Region, SE3 };

class Goal
{
public:
    Goal(const Bounds& b, GoalType goalType) : type(goalType), bounds_(b), dimension_(b.size()) {}

    virtual bool isSatisfied(const State& q) const = 0;
    virtual std::optional<State> sample() = 0;

    const GoalType type;

protected:
    const Bounds bounds_;
    const std::size_t dimension_;
};

class GoalSingleState : public Goal
{
public:
    GoalSingleState(const Bounds& b) : Goal(b, GoalType::SingleState), distance_(b) {}
    GoalSingleState(const Bounds& b, const State& g);

    bool setGoal(const State& g);
    std::optional<State> sample() override { return q_; }
    bool isSatisfied(const State& q) const override { return distance_.isEquivalent(q, q_); }

private:
    Distance distance_;
    State q_;
};

// TODO: duplicated code woth GoalSingleState
class GoalWithJointTolerance : public Goal
{
public:
    GoalWithJointTolerance(const Bounds& b, double tolerance) :
        Goal(b, GoalType::JointTolerance), distance_(b), tolerance_(tolerance) {}
    GoalWithJointTolerance(const Bounds& b, double tolerance, const State& g);

    bool setGoal(const State& g);
    std::optional<State> sample() override { return q_; }
    bool isSatisfied(const State& q) const override;

private:
    Distance distance_;
    double tolerance_;
    State q_;
};

// class ConvexConeR3
// {
// public:
//     ConvexConeR3(const Eigen::Vector3d& v, double angle) :
//          v_(v), angleLimit_(angle), tangentLimit_(tan(angle)) {}

//     bool isInRegion(Eigen::Quaterniond q)
//     { 
//         Eigen::Quaterniond v(0.0, v_[0], v_[1], v_[2]); // v: unit vector, q: versor
//         auto qTemp = q * v * q.inverse();
//         Eigen::Vector3d vRotate{qTemp.x(), qTemp.y(), qTemp.z()};
//         // (v1^2 + v2^2)^0.5 <= tangentLimit_ * abs(v3)
//         return vRotate[0] * vRotate[0] + vRotate[1] * vRotate[1] <= 
//                tangentLimit_ * tangentLimit_ * vRotate[2] * vRotate[2]; 
//     }
//     Eigen::Quaterniond sample() {} // TODO:

// private:
//     Eigen::Vector3d v_; // z axis, norm vector
//     double angleLimit_;
//     double tangentLimit_;
// };

// class BoundingBox
// {
// public:
//     BoundingBox(const Eigen::Vector3d& center, const Eigen::Vector3d& region) : 
//         center_(center), region_(region) {}

//     bool isInRegion(const Eigen::Vector3d& pose)
//     {
//         return pose[0] >= center_[0] - region_[0] && pose[0] <= center_[0] + region_[0] &&
//                pose[1] >= center_[1] - region_[1] && pose[1] <= center_[1] + region_[1] &&
//                pose[2] >= center_[2] - region_[2] && pose[2] <= center_[2] + region_[2];
//     }
//     Eigen::Vector3d sample() {} // TODO:

// private:
//     Eigen::Vector3d center_;
//     Eigen::Vector3d region_;
// };

// class SE3
// {
// public:
//     SE3(const Eigen::Vector3d& center, const Eigen::Vector3d& region,
//         const Eigen::Vector3d& v, double angle) : box_(center, region), quat_(v, angle) { }

//     bool isInRegion(const Eigen::Isometry3d& s); // TODO:
//     Eigen::Isometry3d sample() {} // TODO:

// private:
//     BoundingBox box_;
//     ConvexConeR3 quat_;
// };
}
