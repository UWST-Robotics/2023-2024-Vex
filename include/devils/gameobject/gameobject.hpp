#pragma once
#include "../odom/pose.hpp"

namespace devils
{
    /**
     * Represents a game object on the field
     */
    struct GameObject : public Pose
    {
        /**
         * Creates a new game object with a given pose
         * @param p The pose of the game object
         */
        GameObject(Pose p) : Pose(p) {}
    };
}