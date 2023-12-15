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
            profilePoints.reserve(controlPoints.points.size() * (1 / DT));

            auto initialPoint = controlPoints.points[0];
            bool isReversed = false;
            for (int i = 0; i < controlPoints.points.size() - 1; i++)
            {
                // Get the two points to lerp between
                auto p1 = controlPoints.points[i];
                auto p2 = controlPoints.points[i + 1];

                // Convert to Radians
                auto p1Radians = Units::degToRad(p1.rotation);
                auto p2Radians = Units::degToRad(p2.rotation);

                // Convert to ProfilePose
                auto p1Profile = ProfilePose{
                    p1.x - initialPoint.x,
                    p1.y - initialPoint.y,
                    p1Radians};
                auto p2Profile = ProfilePose{
                    p2.x - initialPoint.x,
                    p2.y - initialPoint.y,
                    p2Radians};

                // Get Anchor Points
                auto a1 = ProfilePose{
                    p1Profile.x + p1.exitDelta * std::cos(p1Radians) * (isReversed ? -1 : 1),
                    p1Profile.y + p1.exitDelta * std::sin(p1Radians) * (isReversed ? -1 : 1),
                    p1Radians};
                auto a2 = ProfilePose{
                    p2Profile.x - p2.enterDelta * std::cos(p2Radians),
                    p2Profile.y - p2.enterDelta * std::sin(p2Radians),
                    p2Radians};

                // Reverse Point
                if (p2.isReversed)
                    isReversed = !isReversed;

                // Lerp between points
                for (double t = 0; t < 1; t += DT)
                {
                    auto lerp = PathUtils::cubicLerpPoints(p1Profile, a1, a2, p2Profile, t);
                    // lerp.x *= 2;
                    // lerp.y *= 2;
                    profilePoints.push_back(lerp);
                }
            }

            // Set profile
            motionProfile->setProfilePoints(controlPoints.points, profilePoints, DT);
        }

    private:
        static constexpr double DT = 0.025; // seconds
    };
}