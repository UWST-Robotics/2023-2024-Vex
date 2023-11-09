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
        std::string name;
        std::string params;
    };

    /**
     * A struct representing a point in the robot path.
     */
    struct PathPoint
    {
        double x = 0;
        double y = 0;
        double rotation = 0;
        double enterDelta = 0;
        double exitDelta = 0;
        bool isReversed = false;
        std::vector<PathEvent> events = {};
    };

    /**
     * A struct representing a robot path.
     */
    struct PathFile
    {
        int version = 0;
        std::vector<PathPoint> points;
    };
}