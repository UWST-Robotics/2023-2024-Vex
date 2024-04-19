#pragma once
#include "pathFileReader.hpp"
#include "pathFile.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/units.hpp"
#include <vector>

namespace devils
{
    /**
     * Represents a path generated from a series of control points.
     */
    struct GeneratedPath
    {
        /// @brief The amount of indices between each point in the path.
        double dt = 0;

        /// @brief A list of control points that the path was generated from.
        ControlPoints controlPoints = {};

        /// @brief The list of points interpolated between the control points. `1/dt` points per control point.
        PoseSequence pathPoints = {};

        /**
         * Gets a single point on the path at a given time.
         * @param t The index of the point to get.
         * @return The point on the path at the given time.
         */
        Pose *getPoseAtTime(double t)
        {
            if (pathPoints.size() == 0)
                return nullptr;
            if (t < 0)
                return &pathPoints[0];
            if (t > pathPoints.size() * dt)
                return &pathPoints[pathPoints.size() - 1];

            return &pathPoints[(int)(t / dt)];
        }

        /**
         * Gets the starting pose of the motion profile.
         * @return The starting pose of the motion profile as an Pose.
         */
        Pose *getStartingPose()
        {
            if (controlPoints.size() <= 0)
            {
                Logger::warn("No control points in path");
                return nullptr;
            }

            return &controlPoints[0];
        }

        /**
         * Gets whether the motion profile has been generated.
         * @return Whether the motion profile has been generated.
         */
        bool isGenerated()
        {
            return pathPoints.size() > 0;
        }
    };
}