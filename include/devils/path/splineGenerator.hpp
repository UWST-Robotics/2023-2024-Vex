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
            if (!PathFileReader::isSDInserted())
            {
                Logger::warn("SplineGenerator: SD Card not inserted");
                return;
            }

            auto controlPoints = PathFileReader::readFromSD();
            std::vector<ProfilePose> profilePoints;
            profilePoints.reserve(controlPoints.points.size() * (1 / DT));

            auto initialPoint = controlPoints.points[0];
            bool isReversed = false;
            for (int i = 0; i < controlPoints.points.size() - 1; i++)
            {
                // Get the two points to lerp between
                auto p1 = controlPoints.points[i];
                auto p2 = controlPoints.points[i + 1];

                // Convert to ProfilePose
                auto p1Profile = ProfilePose{
                    p1.x - initialPoint.x,
                    p1.y - initialPoint.y,
                    p1.rotation};
                auto p2Profile = ProfilePose{
                    p2.x - initialPoint.x,
                    p2.y - initialPoint.y,
                    p2.rotation};

                // Get Anchor Points
                auto a1 = ProfilePose{
                    p1Profile.x + p1.exitDelta * std::cos(p1.rotation) * (isReversed ? -1 : 1),
                    p1Profile.y + p1.exitDelta * std::sin(p1.rotation) * (isReversed ? -1 : 1),
                    p1.rotation};
                auto a2 = ProfilePose{
                    p2Profile.x - p2.enterDelta * std::cos(p2.rotation),
                    p2Profile.y - p2.enterDelta * std::sin(p2.rotation),
                    p2.rotation};

                // Lerp between points
                for (double t = 0; t < 1; t += DT)
                {
                    auto lerp = PathUtils::cubicLerpPoints(p1Profile, a1, a2, p2Profile, t);
                    lerp.isReversed = isReversed;
                    profilePoints.push_back(lerp);
                }

                // Reverse Point
                if (p2.isReversed)
                    isReversed = !isReversed;
            }

            // Set profile
            motionProfile->setProfilePoints(controlPoints.points, profilePoints, DT);
        }

    private:
        static constexpr double DT = 0.025; // seconds
    };
}