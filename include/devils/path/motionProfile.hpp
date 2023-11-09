#pragma once
#include "pathFileReader.hpp"
#include "pathFile.hpp"
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
            float robotTrackWidth)
            : constraints(squiggles::Constraints(maxVelocity, maxAcceleration, maxJerk)),
              model(std::make_shared<squiggles::TankModel>(robotTrackWidth, constraints)),
              generator(squiggles::SplineGenerator(constraints, model, DT))
        {
        }

        /**
         * Generates the motion profile from the path file.
         * This is computationally expensive, so it should only be done once before the match.
         */
        void generate()
        {
            motionPath = generator.generate(getPointsFromSD());
        }

        /**
         * Returns the position, velocity, and heading at time t.
         * @param t The time to sample at.
         * @return squiggles::ProfilePoint at time t.
         */
        squiggles::ProfilePoint getPoint(float t)
        {
            if (motionPath.size() == 0)
                return squiggles::ProfilePoint();
            if (t < 0)
                return motionPath[0];
            if (t > motionPath.size() * DT)
                return motionPath[motionPath.size() - 1];

            return motionPath[t / DT];
        }

    private:
        const float DT = 0.01; // seconds

        squiggles::Constraints constraints;
        std::shared_ptr<squiggles::TankModel> model;
        squiggles::SplineGenerator generator;
        std::vector<squiggles::ProfilePoint> motionPath = {};

        std::vector<squiggles::Pose> getPointsFromSD()
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
    };
}