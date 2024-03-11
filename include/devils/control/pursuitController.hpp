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
        const Pose getTargetPose() override
        {
            if (!motionProfile.isGenerated())
                return Pose();

            auto allPoints = motionProfile.getProfilePoints();
            return allPoints[pathPointIndex];
        }

        /**
         * Restarts the motion profile from the beginning.
         */
        void reset()
        {
            pathPointIndex = 0;
            odometry.setPose(motionProfile.getStartingPose());
        }

        /**
         * Gets list of current events.
         * @return List of current events.
         */
        const std::vector<PathEvent> getCurrentEvents() override
        {
            auto controlPoints = motionProfile.getControlPoints();
            if (controlPointIndex >= controlPoints.size() || controlPointIndex < 0)
                return {};
            return controlPoints[controlPointIndex].events;
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

                // Update Path Point Index
                double closestDistance = 100000;
                for (int i = pathPointIndex; i < profilePoints.size(); i++)
                {
                    auto point = profilePoints[i];
                    double distance = abs(LOOKAHEAD_DISTANCE - sqrt(pow(point.x - currentPose.x, 2) + pow(point.y - currentPose.y, 2)));
                    if (distance < closestDistance)
                    {
                        closestDistance = distance;
                        pathPointIndex = i;
                    }
                    if (i - pathPointIndex > LOOKAHEAD_MAX_INDICES)
                        break;
                }

                // Update Control Point Index
                if (controlPointIndex < controlPoints.size() - 1)
                {
                    auto controlPoint = controlPoints[controlPointIndex + 1];
                    double distanceSq = pow(controlPoint.x - currentPose.x, 2) + pow(controlPoint.y - currentPose.y, 2);
                    if (distanceSq < EVENT_TRIGGER_RANGE * EVENT_TRIGGER_RANGE)
                        controlPointIndex++;
                }

                // Get Current Point
                auto currentPathPoint = profilePoints[pathPointIndex];

                // Calculate Forward & Turn
                double deltaX = currentPathPoint.x - currentPose.x;
                double deltaY = currentPathPoint.y - currentPose.y;
                double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
                double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;

                // Handle Reversed
                if (currentPathPoint.isReversed)
                    deltaRotation = Units::diffRad(deltaRotation, M_PI);

                // Calculate PID
                double forward = translationPID.update(deltaForward);
                double turn = rotationPID.update(deltaRotation);

                // Clamp Values
                forward = Curve::clamp(-1.0, 1.0, forward) * speed;
                turn = Curve::clamp(-1.0, 1.0, turn) * speed;

                // Prevent Sparatic Rotation
                // if (abs(deltaForward) < DISABLE_ROTATION_RANGE)
                //    turn = 0;
                // else if (abs(deltaRotation) > DISABLE_ACCEL_RANGE)
                //    forward = 0;

                // Handle Finish
                bool withinRangeOfLastPoint = pow(lastPoint.x - currentPose.x, 2) + pow(lastPoint.y - currentPose.y, 2) < EVENT_TRIGGER_RANGE * EVENT_TRIGGER_RANGE;
                bool lookingAtLastPoint = pathPointIndex == profilePoints.size() - 1;
                if (withinRangeOfLastPoint && lookingAtLastPoint)
                    hasFinished = true;

                // Execute
                chassis.move(forward, turn);
                pros::delay(20);
            }
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
        static constexpr int LOOKAHEAD_MAX_INDICES = 10;
        static constexpr double LOOKAHEAD_DISTANCE = 4.0;        // in
        static constexpr double EVENT_TRIGGER_RANGE = 6;         // in
        static constexpr double DISABLE_ROTATION_RANGE = 3.0;    // in
        static constexpr double DISABLE_ACCEL_RANGE = M_PI / 10; // rads

        PID translationPID = PID(0.1, 0, 0); // <-- Translation
        PID rotationPID = PID(0.2, 0, 0);    // <-- Rotation

        BaseChassis &chassis;
        MotionProfile &motionProfile;
        OdomSource &odometry;
        int pathPointIndex = 0;    // Closest index of the lookahead point
        int controlPointIndex = 0; // Current index of the event

        bool isPaused = false;
        bool hasFinished = false;
    };
}