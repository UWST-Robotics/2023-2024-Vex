#pragma once
#include "../odom/pose.hpp"

namespace devils
{
    struct ProfilePose : public Pose
    {
        /// @brief True if the robot is reversed
        bool isReversed = false;
    };
}