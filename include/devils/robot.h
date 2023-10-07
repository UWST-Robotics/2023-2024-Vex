#pragma once
#include "utils/logger.h"

namespace devils
{
    class Robot
    {
    public:
        /**
         * Constructs a new robot instance that is passed to other classes
         * in order to control the robot and its subsystems.
         */
        Robot();

        Logger *getLogger();

    private:
        Logger logger;
    };
}