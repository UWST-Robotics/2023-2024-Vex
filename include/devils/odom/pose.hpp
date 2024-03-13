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
        double x = 0;
        /// @brief The y position of the robot in inches
        double y = 0;
        /// @brief The rotation of the robot in radians
        double rotation = 0;

        Pose operator+(const Pose &other)
        {
            return {x + other.x, y + other.y, rotation + other.rotation};
        }
        Pose operator*(const double &scalar)
        {
            return {x * scalar, y * scalar, rotation * scalar};
        }
        bool operator==(const Pose &other)
        {
            return x == other.x && y == other.y && rotation == other.rotation;
        }
        bool operator!=(const Pose &other)
        {
            return !(*this == other);
        }

        double distanceTo(const Pose &other)
        {
            return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
        }

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