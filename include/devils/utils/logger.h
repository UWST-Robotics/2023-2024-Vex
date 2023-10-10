#pragma once
#include "okapi/api/util/logging.hpp"

namespace devils
{
    // TODO: Check to see if this logs to the SD card or if it only logs to the terminal
    class Logger
    {
    public:
        /**
         * Initializes the logger.
         */
        static void init();

        /**
         * Logs a message to the LCD.
         * @param message The message to send.
         */
        static void sendToLCD(std::string message);

        /**
         * Logs an info message.
         * @param message The message to log.
         */
        static void info(std::string);

        /**
         * Logs a warning message.
         * @param message The message to log.
         */
        static void warn(std::string);

        /**
         * Logs an error message.
         * @param message The message to log.
         */
        static void error(std::string);

        /**
         * Logs a debug message.
         * @param message The message to log.
         */
        static void debug(std::string);

        /**
         * Gets the Okapi logger.
         * @return The Okapi logger.
         */
        static std::shared_ptr<okapi::Logger> getLogger();

    private:
        inline static const std::string LOG_TERMINAL = "/ser/sout";
        inline static const std::string LOG_FILE_PATH = "/usd/log.txt";
        inline static const bool LOG_TO_DISPLAY = true;
        inline static const bool LOG_TO_FILE = true;
    };
}