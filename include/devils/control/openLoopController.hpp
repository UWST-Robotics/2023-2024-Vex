#pragma once
#include "../chassis/chassis.hpp"
#include "../path/motionProfile.hpp"
#include "autoController.hpp"
#include "pros/rtos.hpp"

namespace devils
{
    class OpenLoopController : public AutoController
    {
    public:
        OpenLoopController(BaseChassis &chassis, MotionProfile &motionProfile)
            : chassis(chassis), motionProfile(motionProfile)
        {
            motionProfile.generate();
        }

        squiggles::ProfilePoint getCurrentPoint() override
        {
            double time = (pros::millis() - startTime) / 1000.0;
            return motionProfile.getPointAtTime(time);
        }

        void restart() override
        {
            startTime = pros::millis();
        }

        void update() override
        {
            auto currentPoint = getCurrentPoint();

            double leftVelocity = Units::metersToIn(currentPoint.wheel_velocities[0]);  // inches per second
            double rightVelocity = Units::metersToIn(currentPoint.wheel_velocities[1]); // inches per second

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