#pragma once
#include "pathFileReader.hpp"
#include "pathFile.hpp"
#include "profilePose.hpp"
#include "../odom/odomPose.hpp"
#include "../utils/units.hpp"
#include <vector>

namespace devils
{
    /**
     * Represents a spline path that can sample velocities and positions at any time t.
     */
    class MotionProfile
    {
    public:
        /**
         * Sets the points of the motion profile.
         * @param points The points of the motion profile.
         * @param dt The time between each point in seconds.
         */
        void setProfilePoints(
            std::vector<PathPoint> controlPoints,
            std::vector<ProfilePose> profilePoints,
            double dt)
        {
            this->controlPoints = controlPoints;
            this->profilePoints = profilePoints;
            this->dt = dt;
        }

        /**
         * Gets all points of the motion profile.
         * @return All points of the motion profile.
         */
        std::vector<ProfilePose> getProfilePoints()
        {
            return profilePoints;
        }

        /**
         * Gets all control points of the motion profile.
         * @return All control points of the motion profile.
         */
        std::vector<PathPoint> getControlPoints()
        {
            return controlPoints;
        }

        /**
         * Gets a single point of the motion profile at a given time.
         * @param t The time to get the point at in seconds.
         * @return The point of the motion profile at the given time.
         */
        const ProfilePose getPointAtTime(double t)
        {
            if (profilePoints.size() == 0)
                return ProfilePose();
            if (t < 0)
                return profilePoints[0];
            if (t > profilePoints.size() * dt)
                return profilePoints[profilePoints.size() - 1];

            return profilePoints[(int)(t / dt)];
        }

        /**
         * Gets the starting pose of the motion profile.
         * @return The starting pose of the motion profile as an OdomPose.
         */
        const OdomPose getStartingPose()
        {
            return {0, 0, Units::degToRad(controlPoints[0].rotation)};
        }

        /**
         * Gets the path of the motion profile.
         * @return The path of the motion profile.
         */
        double getDuration()
        {
            return duration;
        }

        /**
         * Gets whether the motion profile has been generated.
         * @return Whether the motion profile has been generated.
         */
        bool isGenerated()
        {
            return profilePoints.size() > 0;
        }

    private:
        double dt = 0;
        double duration = 0;
        std::vector<PathPoint> controlPoints = {};
        std::vector<ProfilePose> profilePoints = {};
    };
}