#pragma once
#include "pathFileReader.hpp"
#include "pathFile.hpp"
#include "okapi/squiggles/squiggles.hpp"
#include "../utils/units.hpp"
#include <vector>

namespace devils
{
    class MotionProfile
    {
    public:
        /**
         * Represents a spline path that can sample velocities and positions at any time t.
         * @param maxVelocity The maximum velocity of the robot in inches per second.
         * @param maxAcceleration The maximum acceleration of the robot in inches per second squared.
         * @param maxJerk The maximum jerk of the robot in inches per second cubed.
         * @param robotTrackWidth The width of the robot in inches.
         */
        MotionProfile(
            float maxVelocity,
            float maxAcceleration,
            float maxJerk,
            float robotTrackWidth)
            : constraints(Units::inToMeters(maxVelocity), Units::inToMeters(maxAcceleration), Units::inToMeters(maxJerk)),
              model(std::make_shared<squiggles::TankModel>(Units::inToMeters(robotTrackWidth), constraints)),
              generator(constraints, model, DT)
        {
        }

        /**
         * Generates the motion profile from the path file.
         * This is computationally expensive, so it should only be done once before the match.
         */
        void generate()
        {
            pathPoints = getTestPoints();
            motionPath = generator.generate(pathPoints);
        }

        /**
         * Returns the duration of the motion profile.
         * @return The duration of the motion profile in seconds.
         */
        double getDuration()
        {
            return motionPath.size() * DT;
        }

        /**
         * Returns the path of the motion profile.
         * @return The path of the motion profile.
         */
        std::vector<squiggles::ProfilePoint> &getPath()
        {
            return motionPath;
        }

        /**
         * Returns the control points of the motion profile.
         * @return The control points of the motion profile.
         */
        std::vector<squiggles::Pose> &getPathPoints()
        {
            return pathPoints;
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
        static constexpr float DT = 0.1; // seconds

        squiggles::Constraints constraints;
        std::shared_ptr<squiggles::TankModel> model;
        squiggles::SplineGenerator generator;
        std::vector<squiggles::Pose> pathPoints = {};
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

        std::vector<squiggles::Pose> getTestPoints()
        {
            std::vector<squiggles::Pose> points;
            points.push_back({0, 0, 0});
            points.push_back({2, 1, M_PI / 2.0});
            points.push_back({3, 3, 0});
            points.push_back({4, 1, -M_PI / 2.0});
            points.push_back({0, 0, M_PI});
            return points;
        }
    };
}