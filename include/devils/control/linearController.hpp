#pragma once
#include "../chassis/chassis.hpp"
#include "../path/motionProfile.hpp"
#include "autoController.hpp"
#include "pros/rtos.hpp"
#include "../utils/pid.hpp"
#include "../utils/curve.hpp"
#include "../odom/odomSource.hpp"

namespace devils
{
    /**
     * Controller for follwing a motion profile in linear motions.
     * Usually used with LinearGenerator for the motion profile.
     */
    class LinearController : public AutoController
    {
    public:
        /**
         * Constructs a new LinearController.
         * @param chassis The chassis to control.
         * @param motionProfile The motion profile to follow.
         */
        LinearController(BaseChassis &chassis, MotionProfile &motionProfile, OdomSource &odometry)
            : chassis(chassis), motionProfile(motionProfile), odometry(odometry)
        {
        }

        /**
         * Gets list of current events.
         * @return List of current events.
         */
        const std::vector<PathEvent> getCurrentEvents()
        {
            auto controlPoints = motionProfile.getControlPoints();
            if (currentIndex > controlPoints.size() || currentIndex <= 0)
                return {};
            return controlPoints[currentIndex - 1].events;
        }

        /**
         * Gets the current index of the motion profile.
         * @return The current index of the motion profile.
         */
        const int getCurrentIndex()
        {
            return currentIndex;
        }

        /**
         * Returns the current point of the motion profile.
         * @return The current point of the motion profile.
         */
        const ProfilePose getTargetPose() override
        {
            auto controlPoints = motionProfile.getControlPoints();
            if (currentIndex >= controlPoints.size() || currentIndex < 0)
                return ProfilePose();
            return controlPoints[currentIndex];
        }

        /**
         * Resets the motion profile from the beginning.
         */
        void reset()
        {
            isReversed = false;
            currentIndex = 1;
            lastCheckpointTime = pros::millis();
        }

        /**
         * Returns true if the controller has finished.
         * @return True if the controller has finished.
         */
        bool isFinished() override
        {
            return currentIndex >= motionProfile.getControlPoints().size();
        }

        /**
         * Updates the chassis based on the current point of the motion profile.
         * Also updates the chassis input.
         */
        void _run() override
        {
            // Reset
            reset();

            // Get Path
            auto controlPoints = motionProfile.getControlPoints();

            // Loop
            while (true)
            {
                // Get Current Pose
                auto currentPose = odometry.getPose();

                // Calculate time since last checkpoint
                double timeSinceLastCheckpoint = pros::millis() - lastCheckpointTime;
                bool skipCheckpoint = timeSinceLastCheckpoint > CHECKPOINT_TIMEOUT && CHECKPOINT_TIMEOUT > 0;

                // Calculate Point
                auto point = controlPoints[currentIndex];
                double distanceSquared = pow(point.x - currentPose.x, 2) + pow(point.y - currentPose.y, 2);

                // Check within trigger range
                if (distanceSquared < CHECKPOINT_RANGE_SQ || skipCheckpoint)
                {
                    currentIndex++;
                    lastCheckpointTime = pros::millis();
                    if (point.isReversed)
                        isReversed = !isReversed;
                    continue;
                }

                // Get Current Point
                auto currentPoint = controlPoints[currentIndex % controlPoints.size()];

                // Calculate direction of travel
                double deltaX = currentPoint.x - currentPose.x;
                double deltaY = currentPoint.y - currentPose.y;
                double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
                double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;

                // Handle Reversed
                if (isReversed)
                    deltaRotation = Units::diffRad(deltaRotation, M_PI);

                // Calculate Speeds
                double forward = translationPID.update(deltaForward);
                double turn = rotationPID.update(deltaRotation);

                // Clamp Values
                forward = Curve::clamp(-maxSpeed, maxSpeed, forward);
                turn = Curve::clamp(-maxSpeed, maxSpeed, turn);

                // Disable forward if need to rotate
                if (abs(deltaRotation) > DISABLE_ACCEL_RANGE)
                {
                    forward = 0;
                    lastRotationTime = pros::millis();
                }

                // Delay acceleration after rotation
                if (pros::millis() - lastRotationTime < ACCEL_DELAY)
                    forward = 0;

                // Finish
                if (currentIndex >= controlPoints.size())
                {
                    pause();
                    break;
                }

                // Move Chassis
                chassis.move(forward, turn);

                // Delay Task
                pros::delay(10);
            }
        }

        /**
         * Pauses the chassis movement
         */
        void pause()
        {
            chassis.stop();
            lastCheckpointTime = pros::millis();
        }

        /**
         * Sets the maximum speed of the chassis.
         * @param speed The maximum speed of the chassis from 0 to 1.
         */
        void setSpeed(double speed) override
        {
            maxSpeed = speed;
        }

    private:
        // Constants
        static constexpr double DEFAULT_MAX_SPEED = 0.45;
        static constexpr double CHECKPOINT_TIMEOUT = 3000;       // ms
        static constexpr double DISABLE_ACCEL_RANGE = M_PI / 10; // rads
        static constexpr double ACCEL_DELAY = 200;               // ms
        static constexpr double CHECKPOINT_RANGE = 4;            // in

        // Derived Constants
        static constexpr double CHECKPOINT_RANGE_SQ = CHECKPOINT_RANGE * CHECKPOINT_RANGE; // sq in

        // PID
        PID translationPID = PID(0.1, 0, 0); // <-- Translation
        PID rotationPID = PID(0.2, 0, 0);    // <-- Rotation

        // Members
        BaseChassis &chassis;
        MotionProfile &motionProfile;
        OdomSource &odometry;
        int currentIndex = 1; // Current control point driving towards
        double lastCheckpointTime = 0;
        double lastRotationTime = 0;
        double maxSpeed = DEFAULT_MAX_SPEED;
        bool isReversed = false;
    };
}