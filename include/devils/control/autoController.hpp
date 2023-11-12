#pragma once
#include "squiggles.hpp"

namespace devils
{
    struct AutoController
    {
        /**
         * Gets the current target point of the controller.
         */
        virtual squiggles::ProfilePoint getCurrentPoint() { return squiggles::ProfilePoint(); }

        /**
         * Restarts the controller from the beginning.
         */
        virtual void restart() {}

        /**
         * Updates the controller & calls move on the chassis.
         */
        virtual void update() {}
    };
}