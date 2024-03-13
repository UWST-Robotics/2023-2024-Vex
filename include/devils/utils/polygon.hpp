#pragma once
#include "pros/rtos.hpp"
#include "../utils/logger.hpp"
#include "../odom/pose.hpp"

namespace devils
{
    /**
     * Represents a polygonal area.
     */
    struct Polygon
    {
        /**
         * Creates a new polygon.
         * @param points The points of the polygon.
         */
        Polygon(std::initializer_list<Pose> points)
            : points(points)
        {
        }

        /**
         * Return true if the odometry pose is within the polygon.
         * @param pose The odometry pose to check.
         * @return True if the odometry pose is within the polygon.
         */
        bool contains(Pose &pose)
        {
            int i, j;
            bool c = false;
            for (i = 0, j = points.size() - 1; i < points.size(); j = i++)
            {
                if (((points[i].y > pose.y) != (points[j].y > pose.y)) &&
                    (pose.x < (points[j].x - points[i].x) * (pose.y - points[i].y) / (points[j].y - points[i].y) + points[i].x))
                    c = !c;
            }
            return c;
        }

        std::vector<Pose> points;
    };
}