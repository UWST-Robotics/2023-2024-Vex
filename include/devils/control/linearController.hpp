#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "autoController.hpp"
#include "pros/rtos.hpp"
#include "../utils/pid.hpp"
#include "../geometry/lerp.hpp"
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
         * @param odometry The odometry source to use.
         * @param generatedPath The generated path to follow.
         */
        LinearController(BaseChassis &chassis, OdomSource &odometry, GeneratedPath &generatedPath)
            : chassis(chassis), odometry(odometry), generatedPath(generatedPath)
        {
        }

        std::vector<PathEvent> &getCurrentEvents() override
        {
            if (currentIndex > generatedPath.controlPoints.size() || currentIndex <= 0)
                return NO_EVENTS;
            return generatedPath.controlPoints[currentIndex - 1].events;
        }

        Pose *getTargetPose() override
        {
            if (currentIndex >= generatedPath.controlPoints.size() || currentIndex < 0)
                return nullptr;
            return &generatedPath.controlPoints[currentIndex];
        }

        bool getFinished() override
        {
            return currentIndex >= generatedPath.controlPoints.size();
        }

        void update() override
        {
            // Get Current Pose
            auto currentPose = odometry.getPose();

            // Calculate time since last checkpoint
            double timeSinceLastCheckpoint = pros::millis() - lastCheckpointTime;
            bool skipCheckpoint = timeSinceLastCheckpoint > CHECKPOINT_TIMEOUT && CHECKPOINT_TIMEOUT > 0;

            // Calculate Point
            auto &controlPoints = generatedPath.controlPoints;
            auto &point = controlPoints[currentIndex];
            double distance = point.distanceTo(currentPose);

            // Check within trigger range
            if (distance < CHECKPOINT_RANGE || skipCheckpoint)
            {
                currentIndex++;
                lastCheckpointTime = pros::millis();
                // continue;
            }

            // Get Current Point
            auto &currentPoint = controlPoints[currentIndex % controlPoints.size()];

            // Calculate direction of travel
            double deltaX = currentPoint.x - currentPose.x;
            double deltaY = currentPoint.y - currentPose.y;
            double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
            double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;

            // Handle Reversed
            if (currentPoint.isReversed)
                deltaRotation = Units::diffRad(deltaRotation, M_PI);

            // Calculate Speeds
            double forward = translationPID.update(deltaForward);
            double turn = rotationPID.update(deltaRotation);

            // Clamp Values
            forward = std::clamp(forward, -1.0, 1.0);
            turn = std::clamp(turn, -1.0, 1.0);

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
                isFinished = true;
                pause();
            }

            // Move Chassis
            chassis.move(forward, turn);
        }

        /**
         * Resets the path from the beginning.
         */
        void reset() override
        {
            isFinished = false;
            currentIndex = 1;
            lastCheckpointTime = pros::millis();
        }

        /**
         * Pauses the chassis movement and prevents checkpoint timeout.
         */
        void pause()
        {
            chassis.stop();
            lastCheckpointTime = pros::millis();
        }

    private:
        // Constants
        static constexpr double DEFAULT_MAX_SPEED = 0.45;
        static constexpr double CHECKPOINT_TIMEOUT = 3000;       // ms
        static constexpr double DISABLE_ACCEL_RANGE = M_PI / 10; // rads
        static constexpr double ACCEL_DELAY = 200;               // ms
        static constexpr double CHECKPOINT_RANGE = 4;            // in

        // PID
        PID translationPID = PID(0.1, 0, 0); // <-- Translation
        PID rotationPID = PID(0.2, 0, 0);    // <-- Rotation

        // Components
        BaseChassis &chassis;
        GeneratedPath &generatedPath;
        OdomSource &odometry;

        // State
        bool isFinished = false;
        int currentIndex = 1; // Current control point driving towards
        double lastCheckpointTime = 0;
        double lastRotationTime = 0;
    };
}