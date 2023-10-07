#pragma once
#include "okapi/api/util/logging.hpp"

namespace devils
{
    class Logger
    {
    public:
        /**
         * A logger that logs to the SD card.
         */
        Logger();

        /**
         * Gets the Okapi logging object.
         */
        okapi::Logger *getLogger();

        /**
         * Logs an info message.
         * @param message The message to log.
         */
        void info(std::string);

        /**
         * Logs a warning message.
         * @param message The message to log.
         */
        void warn(std::string);

        /**
         * Logs an error message.
         * @param message The message to log.
         */
        void error(std::string);

        /**
         * Logs a debug message.
         * @param message The message to log.
         */
        void debug(std::string);

    private:
        const std::string LOG_FILE_PATH = "/usd/latest-log.txt";

        okapi::Logger logger;
    };
}