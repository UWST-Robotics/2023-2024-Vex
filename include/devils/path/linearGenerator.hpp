#pragma once
#include "motionProfile.hpp"
#include "../utils/pathUtils.hpp"
#include <vector>

namespace devils
{
    class LinearGenerator
    {
    public:
        LinearGenerator()
        {
        }

        /**
         * Generates a motion profile from a set of control points using lerp.
         * @param motionProfile The motion profile to store the generated profile in.
         */
        void generate(MotionProfile *motionProfile)
        {
            Logger::info("Generating Linear Motion Profile...");
            if (!PathFileReader::isSDInserted())
            {
                Logger::warn("LinearGenerator: SD Card not inserted");
                return;
            }

            auto controlPoints = PathFileReader::readFromSD();
            std::vector<ProfilePose> profilePoints;
            profilePoints.reserve(controlPoints.points.size() * (1 / DT));

            // auto initialPoint = controlPoints.points[0];
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
                    p1.x,
                    p1.y,
                    p1Radians};
                auto p2Profile = ProfilePose{
                    p2.x,
                    p2.y,
                    p2Radians};

                // Lerp between points
                for (double t = 0; t < 1; t += DT)
                {
                    auto lerp = PathUtils::lerpPoints(p1Profile, p2Profile, t);
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