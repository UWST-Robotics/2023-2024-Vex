#pragma once
#include "chassis.hpp"
#include "../hardware/smartMotorGroup.hpp"
#include <vector>
#include <iostream>
#include "../utils/logger.hpp"
#include "../odom/odomSource.hpp"

namespace devils
{
    /**
     * Represents a chassis that maintains a virtual position and orientation.
     */
    class DummyChassis : public BaseChassis, OdomSource
    {
    public:
        /**
         * Creates a new dummy chassis.
         */
        DummyChassis() {}

        void move(double forward, double turn, double strafe = 0) override
        {
            forward = std::clamp(forward, -1.0, 1.0);
            turn = std::clamp(turn, -1.0, 1.0);
            strafe = std::clamp(strafe, -1.0, 1.0);

            currentPose.x += (cos(currentPose.rotation) * forward + sin(currentPose.rotation) * strafe) * TRANSLATION_SPEED;
            currentPose.y += (sin(currentPose.rotation) * forward + cos(currentPose.rotation) * strafe) * TRANSLATION_SPEED;
            currentPose.rotation += turn * ROTATION_SPEED;
        }

        bool isHolonomic() override
        {
            return false;
        }

        void stop() override {}

        void setPose(const Pose pose) override
        {
            currentPose = pose;
        }

        const Pose getPose() override
        {
            return currentPose;
        }

    private:
        static constexpr double TRANSLATION_SPEED = 4;
        static constexpr double ROTATION_SPEED = 2;

        Pose currentPose;
    };
}