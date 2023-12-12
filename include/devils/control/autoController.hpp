#pragma once
#include "squiggles.hpp"

namespace devils
{
    /**
     * Represents a system for controlling the robot chassis autonomously.
     */
    struct AutoController
    {
        /**
         * Gets the current target point of the controller.
         */
        virtual const ProfilePose getCurrentProfilePoint() = 0;

        /**
         * Restarts the controller from the beginning.
         */
        virtual void restart() = 0;

        /**
         * Updates the controller & calls move on the chassis.
         */
        virtual void update() = 0;
    };
}