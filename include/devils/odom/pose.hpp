#pragma once

namespace devils
{
    /**
     * Represents a robot pose in 2D space.
     */
    struct Pose
    {
        double x;
        double y;
        double rotation;
    };
}