#pragma once
#include "pros/rtos.hpp"
#include "../utils/logger.hpp"
#include "../odom/pose.hpp"

namespace devils
{
    /**
     * Represents a rectangular area.
     */
    class Rect
    {
    public:
        /**
         * Creates a new rectangle.
         * @param x The x position in inches.
         * @param y The y position in inches.
         * @param width The width in inches.
         * @param height The height in inches.
         */
        Rect(double x, double y, double width, double height)
            : x(x),
              y(y),
              width(width),
              height(height)
        {
        }

        /**
         * Return true if the odometry pose is within the rectangle.
         * @param pose The odometry pose to check.
         * @return True if the odometry pose is within the rectangle.
         */
        bool contains(const Pose pose)
        {
            return pose.x >= x && pose.x <= x + width && pose.y >= y && pose.y <= y + height;
        }

    private:
        double x;
        double y;
        double width;
        double height;
    };
}