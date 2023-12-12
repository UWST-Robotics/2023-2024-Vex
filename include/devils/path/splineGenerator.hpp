#pragma once
#include "motionProfile.hpp"
#include "../utils/pathUtils.hpp"
#include <vector>

namespace devils
{
    class SplineGenerator
    {
    public:
        SplineGenerator()
        {
        }

        /**
         * Generates a motion profile from a set of control points using lerp.
         * @param motionProfile The motion profile to store the generated profile in.
         */
        void generate(MotionProfile *motionProfile)
        {
            auto controlPoints = devils::PathFileReader::readFromSD();
            std::vector<ProfilePose> profilePoints;

            double t = 0;
            while (t < controlPoints.points.size())
            {
                // Get the two points to lerp between
                auto p1 = controlPoints.points[std::floor(t)];
                auto p2 = controlPoints.points[std::ceil(t)];

                // Convert to ProfilePose
                auto p1Profile = ProfilePose{p1.x, p1.y, p1.rotation};
                auto p2Profile = ProfilePose{p2.x, p2.y, p2.rotation};

                // Get Anchor Points
                auto a1 = ProfilePose{
                    p1.x + p1.exitDelta * std::cos(p1.rotation) * (p1.isReversed ? -1 : 1),
                    p1.y + p1.exitDelta * std::sin(p1.rotation) * (p1.isReversed ? -1 : 1),
                    p1.rotation};
                auto a2 = ProfilePose{
                    p2.x - p2.enterDelta * std::cos(p2.rotation),
                    p2.y - p2.enterDelta * std::sin(p2.rotation),
                    p2.rotation};

                // Lerp
                auto lerp = PathUtils::cubicLerpPoints(p1Profile, a1, a2, p2Profile, t - std::floor(t));

                t += DT;
            }

            // Set profile
            motionProfile->setProfilePoints(controlPoints.points, profilePoints, DT);
        }

    private:
        static constexpr double DT = 0.1; // seconds
    };
}