#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "../path/pathFile.hpp"
#include "../odom/odomSource.hpp"
#include "../utils/logger.hpp"
#include "../utils/curve.hpp"
#include "../utils/pid.hpp"
#include "autoController.hpp"
#include <cmath>
#include <vector>

namespace devils
{
    /**
     * Controller for going directly to a point using Odometry
     */
    class DirectController : public AutoController
    {
    public:
        /**
         * Constructs a new PursuitController.
         * @param chassis The chassis to control.
         * @param odometry The odometry source to use.
         */
        DirectController(BaseChassis &chassis, OdomSource &odometry)
            : chassis(chassis),
              odometry(odometry)
        {
        }

        std::vector<PathEvent> *getCurrentEvents() override
        {
            return &NO_EVENTS;
        }

        Pose *getTargetPose() override
        {
            return targetPose;
        }

        void update() override
        {
            // Get Current Pose
            auto currentPose = odometry.getPose();

            // Calculate Forward & Turn
            double deltaX = targetPose->x - currentPose.x;
            double deltaY = targetPose->y - currentPose.y;
            double normal = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
            double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
            double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;

            // Handle Reversed
            if (isReversed)
                deltaRotation = Units::diffRad(deltaRotation, M_PI);

            // Calculate PID
            double forward = translationPID.update(deltaForward);
            double turn = rotationPID.update(deltaRotation);

            // Clamp Values
            if (isReversed)
                forward = Curve::clamp(-1.0, 0.0, forward);
            else
                forward = Curve::clamp(0.0, 1.0, forward);
            turn = Curve::clamp(-1.0, 1.0, turn) * normal;

            // Drive
            chassis.move(forward, turn);
        }

        bool getFinished() override
        {
            return isFinished;
        }

        void reset() override {}

        /**
         * Reverse the robot's direction.
         * @param isReversed Whether the robot is driving in reverse.
         */
        void setReverse(bool isReversed)
        {
            this->isReversed = isReversed;
        }

        /**
         * Sets the target pose for the controller.
         */
        void setTargetPose(Pose &targetPose)
        {
            this->targetPose = &targetPose;
        }

    private:
        PID translationPID = PID(5.0, 0, 0); // <-- Translation
        PID rotationPID = PID(0.3, 0, 0);    // <-- Rotation
        std::vector<PathEvent> NO_EVENTS = {};

        // Object Handles
        BaseChassis &chassis;
        OdomSource &odometry;

        // State
        Pose *targetPose;
        bool isReversed = false;
        bool isFinished = false;
    };
}