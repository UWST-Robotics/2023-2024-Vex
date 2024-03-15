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
    class DummyChassis : public BaseChassis, OdomSource, AutoRunnable
    {
    public:
        void move(double forward, double turn, double strafe = 0) override
        {
            forward = std::clamp(forward, -1.0, 1.0) * speed;
            turn = std::clamp(turn, -1.0, 1.0) * speed;
            strafe = std::clamp(strafe, -1.0, 1.0) * speed;

            lastForward = forward;
            lastTurn = turn;
            lastStrafe = strafe;
        }

        void update() override
        {
            // Calculate Acceleration
            currentAcceleration.x += (cos(currentPose.rotation) * lastForward + sin(currentPose.rotation) * lastStrafe) * TRANSLATION_ACCEL;
            currentAcceleration.y += (sin(currentPose.rotation) * lastForward + cos(currentPose.rotation) * lastStrafe) * TRANSLATION_ACCEL;
            currentAcceleration.rotation += lastTurn * ROTATION_ACCEL;
            currentAcceleration = currentAcceleration * ACCEL_DECAY;

            // Update Pose
            currentPose = currentPose + currentAcceleration;
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
        static constexpr double TRANSLATION_ACCEL = 0.3;
        static constexpr double ROTATION_ACCEL = 0.15;
        static constexpr double ACCEL_DECAY = 0.8;

        double lastForward = 0;
        double lastTurn = 0;
        double lastStrafe = 0;
        Pose currentAcceleration = Pose();
        Pose currentPose = Pose();
    };
}