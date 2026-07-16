#ifndef RM_UTILS__TRAJECTORY_HPP
#define RM_UTILS__TRAJECTORY_HPP

#include <cmath>

namespace rm_utils
{
struct Trajectory
{
    bool   unsolvable;
    double fly_time;
    double pitch; // 抬头为正

    // 不考虑空气阻力
    // v0 子弹初速度大小，单位：m/s
    // d 目标水平距离，单位：m
    // h 目标竖直高度，单位：m
    Trajectory(const double v0, const double d, const double h);
};

} // namespace rm_utils

#endif // RM_UTILS__TRAJECTORY_HPP