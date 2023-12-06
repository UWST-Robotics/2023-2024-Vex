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

        /**
         * Converts the event to a string.
         * @return The event as a string.
         */
        std::string toString() const
        {
            return "Event: " + name + " (" + params + ")";
        }
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

        /**
         * Converts the point to a string.
         * @return The point as a string.
         */
        std::string toString() const
        {
            std::string str = "Point: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(rotation) + ")[" + std::to_string(enterDelta) + ", " + std::to_string(exitDelta) + "] R=" + std::to_string(isReversed) + "\n";
            for (auto event : events)
                str += "    " + event.toString() + "\n";
            return str;
        }
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

        /**
         * Converts the path to a string.
         * @return The path as a string.
         */
        std::string toString() const
        {
            std::string str = "PathFile " + std::to_string(version) + "\n====================\n";
            for (auto point : points)
                str += point.toString();
            str += "====================";
            return str;
        }
    };
}