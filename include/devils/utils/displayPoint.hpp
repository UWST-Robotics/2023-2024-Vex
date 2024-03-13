#pragma once
#include <string>

namespace devils
{
    /**
     * Represents a pose in 2D space.
     */
    struct DisplayPoint
    {
        /// @brief The x position of the point
        double x = 0;
        /// @brief The y position of the point
        double y = 0;

        /**
         * Prints the pose to a string
         * @return The pose as a string
         */
        const std::string toString()
        {
            return "Point(" + std::to_string(x) + ", " + std::to_string(y) + ")";
        }
    };
}