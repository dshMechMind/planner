#pragma once

#include <Eigen/Geometry>
#include "state.hpp"

namespace planner {
class Se3Space
{
public:
    using SE3Poses = std::vector<Eigen::Isometry3d>;

    // s1: se3 s2: se3 linearPart distance
    double distanceTrans(const Eigen::Isometry3d& p1, const Eigen::Isometry3d& p2) const;
    bool isEquivalent(const Eigen::Isometry3d& p1, const Eigen::Isometry3d& p2) const;
    std::optional<SE3Poses> InterpolatePath(const Eigen::Isometry3d& p1, const Eigen::Isometry3d& p2, double stepSize) const;
    Eigen::Isometry3d interpolateSe3(const Eigen::Isometry3d& p1, const Eigen::Isometry3d& p2, double time) const;
    
    Eigen::Isometry3d state2Se3(const State& q) const;
    State se32State(const Eigen::Isometry3d& p) const;
};
}