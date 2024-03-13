#pragma once
#include "squiggles.hpp"
#include "pros/rtos.hpp"
#include "../odom/pose.hpp"
#include "../utils/runnable.hpp"

namespace devils
{
    /**
     * Represents a stateful system for controlling the robot chassis autonomously.
     */
    struct AutoController : public Runnable
    {
        /**
         * Gets the current control point of the controller.
         * @return The current control point of the controller.
         */
        virtual std::vector<PathEvent> *getCurrentEvents() { return nullptr; };

        /**
         * Gets the current target point of the controller, for debugging purposes.
         * @return The current target point of the controller.
         */
        virtual Pose *getTargetPose() = 0;

        /**
         * Returns true if the controller has finished.
         * @return True if the controller has finished.
         */
        virtual bool getFinished() = 0;

        /**
         * Resets the controller.
         */
        virtual void reset() = 0;

        /**
         * Updates the controller.
         */
        virtual void update() override = 0;

        /**
         * Runs the controller synchronously.
         */
        void runSync() override
        {
            reset();
            while (!getFinished())
            {
                update();
                pros::delay(20);
            }
        }

        /**
         * Runs the controller as asyncronously
         * @return The PROS task that runs the controller.
         */
        pros::Task runAsync()
        {
            return pros::Task([=]
                              { runSync(); });
        }
    };
}