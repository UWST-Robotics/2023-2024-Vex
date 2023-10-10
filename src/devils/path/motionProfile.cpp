#pragma once
#include "devils/path/motionProfile.h"

devils::MotionProfile::MotionProfile(
    float maxVelocity,
    float maxAcceleration,
    float maxJerk,
    float robotTrackWidth)
    : constraints(squiggles::Constraints(maxVelocity, maxAcceleration, maxJerk)),
      model(std::make_shared<squiggles::TankModel>(robotTrackWidth, constraints)),
      generator(squiggles::SplineGenerator(constraints, model, DT))
{
}

std::vector<squiggles::Pose> devils::MotionProfile::getPointsFromSD()
{
    // Read from SD
    auto path = devils::PathFileReader::ReadFromSD();

    // Convert to squiggles::Pose
    bool isReversed = false;
    std::vector<squiggles::Pose> points;
    for (int i = 0; i < path.points.size(); i++)
    {
        auto point = path.points[i];
        points.push_back({point.x,
                          point.y,
                          point.rotation + (isReversed ? 180 : 0)});

        // Flip the robot if the point is reversed
        if (point.isReversed)
            isReversed = !isReversed;
    }
    return points;
}

void devils::MotionProfile::generate()
{
    motionPath = generator.generate(getPointsFromSD());
}

squiggles::ProfilePoint devils::MotionProfile::getPoint(float t)
{
    if (motionPath.size() == 0)
        return squiggles::ProfilePoint();
    if (t < 0)
        return motionPath[0];
    if (t > motionPath.size() * DT)
        return motionPath[motionPath.size() - 1];

    return motionPath[t / DT];
}