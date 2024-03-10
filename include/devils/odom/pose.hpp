#pragma once
#include <string>

namespace devils
{
    /**
     * Represents a pose in 2D space.
     */
    struct Pose
    {
        /// @brief The x position of the robot in inches
        double x;
        /// @brief The y position of the robot in inches
        double y;
        /// @brief The rotation of the robot in radians
        double rotation;

        /**
         * Prints the pose to a string
         * @return The pose as a string
         */
        const std::string toString()
        {
            return "Pose(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(rotation) + ")";
        }
    };
}