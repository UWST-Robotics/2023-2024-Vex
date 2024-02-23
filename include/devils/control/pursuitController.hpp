#pragma once
#include "../chassis/chassis.hpp"
#include "../path/motionProfile.hpp"
#include "../path/pathFile.hpp"
#include "../odom/odomSource.hpp"
#include "../utils/logger.hpp"
#include "../utils/curve.hpp"
#include "../path/profilePose.hpp"
#include "../utils/pid.hpp"
#include "autoController.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <vector>

namespace devils
{
    /**
     * Controller for follwing a motion profile with sensor feedback using Basic Pure Pursuit.
     */
    class PursuitController : public AutoController
    {
    public:
        /**
         * Constructs a new PursuitController.
         * @param chassis The chassis to control.
         * @param motionProfile The motion profile to follow.
         */
        PursuitController(BaseChassis &chassis, MotionProfile &motionProfile, OdomSource &odometry)
            : chassis(chassis), motionProfile(motionProfile), odometry(odometry)
        {
        }

        /**
         * Returns the current target point of the motion profile.
         * Controller tries to choose LOOKAHEAD_DISTANCE inches ahead of the robot.
         * @return The current target point of the motion profile.
         */
        const ProfilePose getCurrentProfilePoint() override
        {
            if (!motionProfile.isGenerated())
                return ProfilePose();

            auto allPoints = motionProfile.getProfilePoints();
            return allPoints[lookaheadPathIndex];
        }

        /**
         * Restarts the motion profile from the beginning.
         */
        void restart() override
        {
            lookaheadPathIndex = 0;
            odometry.setPose(motionProfile.getStartingPose());
        }

        /**
         * Gets list of current events.
         * @return List of current events.
         */
        const std::vector<PathEvent> getCurrentEvents()
        {
            auto controlPoints = motionProfile.getControlPoints();
            if (currentEventIndex >= controlPoints.size() || currentEventIndex < 0)
                return {};
            return controlPoints[currentEventIndex].events;
        }

        /**
         * Gets the current event index.
         * @return The current event index.
         */
        int getCurrentEventIndex()
        {
            return currentEventIndex;
        }

        /**
         * Updates the chassis based on the current point of the motion profile.
         * Also updates the chassis input.
         */
        void update() override
        {
            // Get Current Pose
            auto currentPose = odometry.getPose();
            auto controlPoints = motionProfile.getControlPoints();
            auto profilePoints = motionProfile.getProfilePoints();

            // Update Closest Motion Profile Point
            double closestDistance = 100000;
            for (int i = lookaheadPathIndex; i < profilePoints.size(); i++)
            {
                auto point = profilePoints[i];
                double distance = abs(LOOKAHEAD_DISTANCE - sqrt(pow(point.x - currentPose.x, 2) + pow(point.y - currentPose.y, 2)));
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    lookaheadPathIndex = i;
                }
                if (i - lookaheadPathIndex > LOOKAHEAD_MAX_INDICES)
                    break;
            }

            // Update Event Index
            auto firstPoint = controlPoints.front();
            for (int i = currentEventIndex; i < controlPoints.size(); i++)
            {
                auto point = controlPoints[i];
                double xPos = point.x - firstPoint.x;
                double yPos = point.y - firstPoint.y;
                double distance = pow(xPos - currentPose.x, 2) + pow(yPos - currentPose.y, 2);
                if (distance < EVENT_TRIGGER_RANGE * EVENT_TRIGGER_RANGE)
                    currentEventIndex = i;
                if (i - currentEventIndex >= EVENT_MAX_INDICES)
                    break;
            }

            // Get Current Point
            auto currentPathPoint = profilePoints[lookaheadPathIndex];
            auto currentProfilePoint = getCurrentProfilePoint();

            // Calculate Forward & Turn
            double deltaX = currentProfilePoint.x - currentPose.x;
            double deltaY = currentProfilePoint.y - currentPose.y;
            double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
            double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;

            // Handle Reversed
            if (currentPathPoint.isReversed)
                deltaRotation = Units::diffRad(deltaRotation, M_PI);

            double forward = translationPID.update(deltaForward);
            double turn = rotationPID.update(deltaRotation);

            // Clamp Values
            forward = Curve::clamp(-1.0, 1.0, forward) * SPEED_SCALE;
            turn = Curve::clamp(-1.0, 1.0, turn) * SPEED_SCALE;

            // Stop when near finish
            auto lastPoint = profilePoints.back();
            bool withinRangeOfLastPoint = pow(lastPoint.x - currentPose.x, 2) + pow(lastPoint.y - currentPose.y, 2) < EVENT_TRIGGER_RANGE * EVENT_TRIGGER_RANGE;
            bool lookingAtLastPoint = lookaheadPathIndex == profilePoints.size() - 1;
            if (withinRangeOfLastPoint && lookingAtLastPoint)
            {
                pause();
                return;
            }

            // Move Chassis
            chassis.move(forward, turn);
        }

        /**
         * Pauses the controller.
         */
        void pause()
        {
            chassis.move(0, 0);
        }

    private:
        static constexpr double LOOKAHEAD_DISTANCE = 4.0; // in
        static constexpr int LOOKAHEAD_MAX_INDICES = 6;
        static constexpr double SPEED_SCALE = 0.4;

        static constexpr double EVENT_TRIGGER_RANGE = 8; // in
        static constexpr int EVENT_MAX_INDICES = 1;      // Max number of indices to check for events

        PID translationPID = PID(0.2, 0, 0); // <-- Translation
        PID rotationPID = PID(0.5, 0, 0);    // <-- Rotation

        BaseChassis &chassis;
        MotionProfile &motionProfile;
        OdomSource &odometry;
        int lookaheadPathIndex = 0; // Closest index of the lookahead point
        int currentEventIndex = 0;  // Current index of the event
    };
}