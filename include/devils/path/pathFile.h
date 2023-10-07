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
        double x;
        double y;
        double rotation;
        double enterDelta;
        double exitDelta;
        std::vector<PathEvent> events;
    };

    /**
     * A struct representing a robot path.
     */
    struct PathFile
    {
        int version;
        std::vector<PathPoint> points;
    };
}