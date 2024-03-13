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
     * This is useful for testing autonomous routines without a physical robot.
     */
    class DummyChassis : public BaseChassis, OdomSource
    {
    public:
        /**
         * Creates a new dummy chassis.
         */
        DummyChassis() : updateTask([=]
                                    { _update(); })
        {
        }

        ~DummyChassis()
        {
            updateTask.remove();
        }

        void move(double forward, double turn, double strafe = 0) override
        {
            forward = std::clamp(forward, -1.0, 1.0);
            turn = std::clamp(turn, -1.0, 1.0);
            strafe = std::clamp(strafe, -1.0, 1.0);

            lastForward = forward;
            lastTurn = turn;
            lastStrafe = strafe;
        }

        /**
         * PROS Task to update the position of the robot.
         */
        void _update()
        {
            while (true)
            {
                // Calculate Acceleration
                currentAcceleration.x += (cos(currentPose.rotation) * lastForward + sin(currentPose.rotation) * lastStrafe) * TRANSLATION_ACCEL;
                currentAcceleration.y += (sin(currentPose.rotation) * lastForward + cos(currentPose.rotation) * lastStrafe) * TRANSLATION_ACCEL;
                currentAcceleration.rotation += lastTurn * ROTATION_ACCEL;
                currentAcceleration = currentAcceleration * ACCEL_DECAY;

                // Update Pose
                currentPose = currentPose + currentAcceleration;

                // Delay
                pros::delay(20);
            }
        }

        bool isHolonomic() override
        {
            return false;
        }

        void stop() override
        {
            move(0, 0);
        }

        void setPose(Pose &pose) override
        {
            currentPose = pose;
        }

        Pose &getPose() override
        {
            return currentPose;
        }

    private:
        static constexpr double TRANSLATION_ACCEL = 0.2;
        static constexpr double ROTATION_ACCEL = 0.1;
        static constexpr double ACCEL_DECAY = 0.8;

        pros::Task updateTask;
        double lastForward = 0;
        double lastTurn = 0;
        double lastStrafe = 0;
        Pose currentAcceleration = Pose();
        Pose currentPose = Pose();
    };
}