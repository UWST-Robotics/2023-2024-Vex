#pragma once
#include "devils/odom/gpsOdometry.h"
#include "devils/utils/logger.h"

devils::GPSOdometry::GPSOdometry(uint8_t gpsPort) : gps(gpsPort)
{
    if (errno != 0)
        Logger::error("GPSOdometry: GPS port is invalid");
}

void devils::GPSOdometry::update()
{
    auto status = gps.get_status();
    if (errno != 0)
        Logger::error("GPSOdometry: GPS update failed");
    currentPose.x = status.x;
    currentPose.y = status.y;
    currentPose.rotation = status.yaw;
}

void devils::GPSOdometry::setPose(Pose pose)
{
    gps.set_position(pose.x, pose.y, pose.rotation);
    if (errno != 0)
        Logger::error("GPSOdometry: GPS set position failed");
}

const devils::Pose devils::GPSOdometry::getPose()
{
    return currentPose;
}