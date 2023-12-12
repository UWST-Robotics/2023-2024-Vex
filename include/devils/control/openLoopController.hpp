#pragma once
#include "../chassis/chassis.hpp"
#include "../path/motionProfile.hpp"
#include "autoController.hpp"
#include "pros/rtos.hpp"

namespace devils
{
    /**
     * Controller for follwing a motion profile without sensor feedback.
     */
    class OpenLoopController : public AutoController
    {
    public:
        /**
         * Constructs a new OpenLoopController.
         * @param chassis The chassis to control.
         * @param motionProfile The motion profile to follow.
         */
        OpenLoopController(BaseChassis &chassis, MotionProfile &motionProfile)
            : chassis(chassis), motionProfile(motionProfile)
        {
        }

        /**
         * Returns the current point of the motion profile.
         * @return The current point of the motion profile.
         */
        const ProfilePose getCurrentProfilePoint() override
        {
            double time = (pros::millis() - startTime) / 1000.0;
            return motionProfile.getPointAtTime(time);
        }

        /**
         * Restarts the motion profile from the beginning.
         */
        void restart() override
        {
            startTime = pros::millis();
        }

        /**
         * Updates the chassis based on the current point of the motion profile.
         * Also updates the chassis input.
         */
        void update() override
        {
            auto currentPoint = getCurrentProfilePoint();

            double leftVelocity = currentPoint.leftWheelVelocity;   // inches per second
            double rightVelocity = currentPoint.rightWheelVelocity; // inches per second

            double forward = ((rightVelocity + leftVelocity) / 2) * TRANSLATION_SCALE;
            double turn = ((rightVelocity - leftVelocity) / 2) * ROTATION_SCALE;

            chassis.move(forward, turn);
        }

    private:
        static constexpr double TRANSLATION_SCALE = 0.024;
        static constexpr double ROTATION_SCALE = 0.02;

        uint32_t startTime = 0;
        BaseChassis &chassis;
        MotionProfile &motionProfile;
    };
}