#pragma once
#include "devils/path/motionProfile.h"
#include "okapi/impl/control/async/asyncMotionProfileControllerBuilder.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include "okapi/api/units/QLength.hpp"

void devils::MotionProfile::GenerateMotionProfile()
{
    auto profileController = okapi::AsyncMotionProfileControllerBuilder()
                                 .withLimits({PATH_MAX_VEL, PATH_MAX_ACCEL, PATH_MAX_JERK})
                                 .buildMotionProfileController();

    auto pathFile = devils::PathFileReader::ReadFromSD();
    // profileController->generatePath(getPoints(pathFile).data());
    // TODO: Generate Path
}

std::vector<okapi::PathfinderPoint> devils::MotionProfile::getPoints(devils::PathFile path)
{
    std::vector<okapi::PathfinderPoint> points;
    for (int i = 0; i < path.points.size(); i++)
    {
        auto point = path.points[i];
        points.push_back({okapi::QLength(point.x),
                          okapi::QLength(point.y),
                          okapi::QAngle(point.rotation)});
    }
    return points;
}