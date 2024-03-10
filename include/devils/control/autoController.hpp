#pragma once
#include "squiggles.hpp"
#include "pros/rtos.hpp"

namespace devils
{
    /**
     * Represents a system for controlling the robot chassis autonomously.
     */
    struct AutoController
    {
        /**
         * Gets a list of current events.
         * @return List of current events.
         */
        virtual const std::vector<PathEvent> getCurrentEvents() = 0;

        /**
         * Gets the current target point of the controller.
         * @return The current target point of the controller.
         */
        virtual const ProfilePose getTargetPose() = 0;

        /**
         * Returns true if the controller has finished.
         * @return True if the controller has finished.
         */
        virtual bool isFinished() = 0;

        /**
         * Sets the speed of the controller.
         */
        virtual void setSpeed(double speed) = 0;

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
    };
}