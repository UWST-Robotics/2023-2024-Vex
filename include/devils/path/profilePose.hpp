#pragma once

namespace devils
{
    struct ProfilePose : public PathPoint
    {
        ProfilePose() : PathPoint()
        {
        }

        ProfilePose(double x, double y, double rotation)
            : PathPoint{x, y, rotation}
        {
        }

        ProfilePose(double x, double y, double rotation, double enterDelta, double exitDelta, bool isReversed, std::vector<PathEvent> events)
            : PathPoint{x, y, rotation, enterDelta, exitDelta, isReversed, events}
        {
        }

        ProfilePose(PathPoint point)
        {
            x = point.x;
            y = point.y;
            rotation = point.rotation;
            enterDelta = point.enterDelta;
            exitDelta = point.exitDelta;
            isReversed = point.isReversed;
            events = point.events;
        }
    };
}