#pragma once
#include "okapi/api/util/logging.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace devils
{
    // TODO: Check to see if this logs to the SD card or if it only logs to the terminal
    class Logger
    {
    public:
        /**
         * Initializes the logger.
         */
        static void init()
        {
            okapi::Logger::setDefaultLogger(
                std::make_shared<okapi::Logger>(
                    okapi::TimeUtilFactory::createDefault().getTimer(),
                    LOG_TO_FILE ? LOG_FILE_PATH : LOG_TERMINAL,
                    okapi::Logger::LogLevel::warn));
        }

        /**
         * Logs a message to the LCD.
         * @param message The message to send.
         */
        static void sendToLCD(std::string message)
        {
            static int line = 0;
            pros::lcd::set_text(line, message);
            line = (line + 1) % 8;
        }

        /**
         * Logs an info message.
         * @param message The message to log.
         */
        static void info(std::string message)
        {
            // okapi::Logger::getDefaultLogger()->info(message);
            if (LOG_TO_DISPLAY)
                sendToLCD(message);
        }

        /**
         * Logs a warning message.
         * @param message The message to log.
         */
        static void warn(std::string message)
        {
            // okapi::Logger::getDefaultLogger()->warn(message);
            if (LOG_TO_DISPLAY)
                sendToLCD(message);
        }

        /**
         * Logs an error message.
         * @param message The message to log.
         */
        static void error(std::string message)
        {
            // okapi::Logger::getDefaultLogger()->error(message);
            if (LOG_TO_DISPLAY)
                sendToLCD(message);
        }

        /**
         * Logs a debug message.
         * @param message The message to log.
         */
        static void debug(std::string message)
        {
            // okapi::Logger::getDefaultLogger()->debug(message);
            if (LOG_TO_DISPLAY)
                sendToLCD(message);
        }

        /**
         * Gets the Okapi logger.
         * @return The Okapi logger.
         */
        static std::shared_ptr<okapi::Logger> getLogger()
        {
            return okapi::Logger::getDefaultLogger();
        }

    private:
        inline static const std::string LOG_TERMINAL = "/ser/sout";
        inline static const std::string LOG_FILE_PATH = "/usd/log.txt";
        inline static const bool LOG_TO_DISPLAY = true;
        inline static const bool LOG_TO_FILE = true;
    };
}