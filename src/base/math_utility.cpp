#include "math_utility.hpp"

namespace planner {
Eigen::Isometry3d rotateZAxis(double angle)
{
    Eigen::AngleAxisd angleAxis(angle, Eigen::Vector3d(0, 0, 1));
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.rotate(angleAxis);
    return tf;
}
double sign(double x)
{
    if (x < -__DBL_MIN__)
        return -1.0;
    if (x > __DBL_MIN__)
        return 1.0;
    else return 0;
}

// [-M_PI, M_PI]
double normalizeAngle(double angle)
{
    double a = fmod(angle, 2.0 * M_PI);
    if ((-M_PI <= a) && (a <= M_PI))
        return a;
    if (a < - M_PI)
        return a + 2 * M_PI;
    else
        return a - 2 * M_PI;
}
}
