#pragma once
#include "pros/rtos.hpp"
#include "../utils/logger.hpp"
#include "pose.hpp"

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

        /**
         * Randomly sets the seed for the random number generator.
         */
        void _setRandomSeed()
        {
            srand(pros::millis());
        }

        /**
         * Gets a random pose within the polygon.
         * @return A random pose within the polygon.
         */
        Pose getRandomPose()
        {
            // If the polygon is empty, return an empty pose
            if (points.size() == 0)
                return Pose();

            // Calculate the bounding box of the polygon
            double minX = points[0].x;
            double maxX = points[0].x;
            double minY = points[0].y;
            double maxY = points[0].y;
            for (auto &point : points)
            {
                minX = std::min(minX, point.x);
                maxX = std::max(maxX, point.x);
                minY = std::min(minY, point.y);
                maxY = std::max(maxY, point.y);
            }

            // Randomly select a point within the bounding box until it is within the polygon
            Pose pose = Pose();
            _setRandomSeed();
            do
            {
                double rand1 = (rand() % 1000) / 1000.0;
                double rand2 = (rand() % 1000) / 1000.0;
                double x = minX + (maxX - minX) * rand1;
                double y = minY + (maxY - minY) * rand2;
                pose = Pose(x, y, 0);
            } while (!contains(pose));

            return pose;
        }

        std::vector<Pose> points;
    };
}