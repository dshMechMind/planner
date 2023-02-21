#pragma once

#include <Eigen/Geometry>

namespace planner{
Eigen::Isometry3d rotateZAxis(double angle);
double sign(double x);
double normalizeAngle(double angle); // [-M_PI, M_PI]
}
