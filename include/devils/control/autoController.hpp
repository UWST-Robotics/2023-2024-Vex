#pragma once
#include "squiggles.hpp"
#include "pros/rtos.hpp"
#include "../odom/pose.hpp"

namespace devils
{
    /**
     * Represents a stateful system for controlling the robot chassis autonomously.
     */
    struct AutoController
    {
        /**
         * Gets the current control point of the controller.
         * @return The current control point of the controller.
         */
        virtual PathPoint *getControlPoint() = 0;

        /**
         * Gets the current target point of the controller, for debugging purposes.
         * @return The current target point of the controller.
         */
        virtual Pose *getTargetPose() = 0;

        /**
         * Returns true if the controller has finished.
         * @return True if the controller has finished.
         */
        virtual bool isFinished() = 0;

        /**
         * Sets the speed of the controller.
         * @param speed The speed of the controller from 0 to 1.s
         */
        virtual void setSpeed(double speed)
        {
            this->speed = speed;
        }

        /**
         * Gets the speed of the controller.
         * @return The speed of the controller from 0 to 1.
         */
        virtual double getSpeed()
        {
            return speed;
        }

        /**
         * Runs the controller as a PROS task.
         * @return The PROS task that runs the controller.
         */
        pros::Task run()
        {
            return pros::Task([=]
                              { _run(); });
        }
        virtual void _run() = 0;

    protected:
        static constexpr double DEFAULT_SPEED = 0.5;

        float speed = DEFAULT_SPEED;
    };
}