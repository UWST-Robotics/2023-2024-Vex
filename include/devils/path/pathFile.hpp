#pragma once
#include <string>
#include <vector>

namespace devils
{

    /**
     * A struct representing an event at a point in the robot path.
     */
    struct PathEvent
    {
        /// @brief The name of the event
        std::string name;
        /// @brief The parameters of the event
        std::string params;
    };

    /**
     * A struct representing a point in the robot path.
     */
    struct PathPoint
    {
        /// @brief The x position of the robot in inches
        double x = 0;
        /// @brief The y position of the robot in inches
        double y = 0;
        /// @brief The rotation of the robot in degrees
        double rotation = 0;
        /// @brief The entry delta of the robot in inches
        double enterDelta = 0;
        /// @brief The exit delta of the robot in inches
        double exitDelta = 0;
        /// @brief Whether the robot is reversed at this point
        bool isReversed = false;
        /// @brief The events at this point
        std::vector<PathEvent> events = {};
    };

    /**
     * A struct representing a robot path.
     */
    struct PathFile
    {
        /// @brief The version of the path file
        int version = 0;
        /// @brief The points in the path
        std::vector<PathPoint> points;
    };
}