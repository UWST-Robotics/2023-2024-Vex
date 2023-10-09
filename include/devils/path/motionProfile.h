#pragma once
#include "pathFileReader.h"
#include "pathFile.h"
#include "okapi/squiggles/squiggles.hpp"
#include <vector>

namespace devils
{
    class MotionProfile
    {
    public:
        /**
         * Represents a spline path that can sample velocities and positions at any time t.
         */
        MotionProfile(
            float maxVelocity,
            float maxAcceleration,
            float maxJerk,
            float robotTrackWidth);

        /**
         * Generates the motion profile from the path file.
         * This is computationally expensive, so it should only be done once before the match.
         */
        void generate();

        /**
         * Returns the position, velocity, and heading at time t.
         * @param t The time to sample at.
         * @return squiggles::ProfilePoint at time t.
         */
        squiggles::ProfilePoint getPoint(float t);

    private:
        const float DT = 0.01; // seconds

        squiggles::Constraints constraints;
        std::shared_ptr<squiggles::TankModel> model;
        squiggles::SplineGenerator generator;
        std::vector<squiggles::ProfilePoint> motionPath = {};

        std::vector<squiggles::Pose> getPointsFromSD();
    };
}