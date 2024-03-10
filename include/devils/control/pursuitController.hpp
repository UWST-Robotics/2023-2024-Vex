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
        const ProfilePose getTargetPose() override
        {
            if (!motionProfile.isGenerated())
                return ProfilePose();

            auto allPoints = motionProfile.getProfilePoints();
            return allPoints[lookaheadPathIndex];
        }

        /**
         * Restarts the motion profile from the beginning.
         */
        void reset()
        {
            lookaheadPathIndex = 0;
            odometry.setPose(motionProfile.getStartingPose());
        }

        /**
         * Gets list of current events.
         * @return List of current events.
         */
        const std::vector<PathEvent> getCurrentEvents() override
        {
            auto controlPoints = motionProfile.getControlPoints();
            if (currentEventIndex >= controlPoints.size() || currentEventIndex < 0)
                return {};
            return controlPoints[currentEventIndex].events;
        }

        /**
         * Updates the chassis based on the current point of the motion profile.
         * Also updates the chassis input.
         */
        void _run() override
        {
            // Get Current Pose
            auto controlPoints = motionProfile.getControlPoints();
            auto profilePoints = motionProfile.getProfilePoints();
            auto lastPoint = profilePoints.back();

            while (!hasFinished)
            {
                // Check if paused
                if (isPaused)
                {
                    chassis.move(0, 0);
                    pros::delay(20);
                    continue;
                }

                // Get Current Pose
                auto currentPose = odometry.getPose();

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
                if (currentEventIndex < controlPoints.size() - 1)
                {
                    auto controlPoint = controlPoints[currentEventIndex + 1];
                    double distanceSq = pow(controlPoint.x - currentPose.x, 2) + pow(controlPoint.y - currentPose.y, 2);
                    if (distanceSq < EVENT_TRIGGER_RANGE * EVENT_TRIGGER_RANGE)
                        currentEventIndex++;
                }

                // Get Current Point
                auto currentPathPoint = profilePoints[lookaheadPathIndex];
                auto currentTargetPoint = getTargetPose();

                // Calculate Forward & Turn
                double deltaX = currentTargetPoint.x - currentPose.x;
                double deltaY = currentTargetPoint.y - currentPose.y;
                double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
                double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;

                // Handle Reversed
                if (currentPathPoint.isReversed)
                    deltaRotation = Units::diffRad(deltaRotation, M_PI);

                double forward = translationPID.update(deltaForward);
                double turn = rotationPID.update(deltaRotation);

                Logger::debug(std::to_string(forward) + ", " + std::to_string(turn));

                // Clamp Values
                forward = Curve::clamp(-1.0, 1.0, forward) * maxSpeed;
                turn = Curve::clamp(-1.0, 1.0, turn) * maxSpeed;

                // Finish
                bool withinRangeOfLastPoint = pow(lastPoint.x - currentPose.x, 2) + pow(lastPoint.y - currentPose.y, 2) < EVENT_TRIGGER_RANGE * EVENT_TRIGGER_RANGE;
                bool lookingAtLastPoint = lookaheadPathIndex == profilePoints.size() - 1;
                if (withinRangeOfLastPoint && lookingAtLastPoint)
                    hasFinished = true;

                // Move Chassis
                chassis.move(forward, turn);

                // Delay Task
                pros::delay(20);
            }
        }

        /**
         * Sets the speed of the controller.
         * @param speed The speed to set.
         */
        void setSpeed(double speed) override
        {
            maxSpeed = speed;
        }

        /**
         * Returns whether the controller has finished.
         * @return Whether the controller has finished.
         */
        bool isFinished() override
        {
            return hasFinished;
        }

    private:
        static constexpr double LOOKAHEAD_DISTANCE = 4.0; // in
        static constexpr int LOOKAHEAD_MAX_INDICES = 10;
        static constexpr double EVENT_TRIGGER_RANGE = 6; // in

        PID translationPID = PID(0.1, 0, 0); // <-- Translation
        PID rotationPID = PID(0.2, 0, 0);    // <-- Rotation

        BaseChassis &chassis;
        MotionProfile &motionProfile;
        OdomSource &odometry;
        int lookaheadPathIndex = 0; // Closest index of the lookahead point
        int currentEventIndex = 0;  // Current index of the event
        float maxSpeed = 0.5;

        bool isPaused = false;
        bool hasFinished = false;
    };
}