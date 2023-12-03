#pragma once
#include "okapi/api/util/logging.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include <string>
#include <unistd.h>

namespace devils
{
    // TODO: Check to see if this logs to the SD card or if it only logs to the terminal

    /**
     * Represents a global logging utility.
     */
    class Logger
    {
    public:
        /**
         * Initializes the logger.
         */
        static void init()
        {
            pros::lcd::initialize();
            okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
                okapi::TimeUtilFactory::createDefault().getTimer(),
                LOG_TO_FILE ? _getLogFilePath() : LOG_TERMINAL,
                okapi::Logger::LogLevel::debug));
        }

        /**
         * Gets a file path for a new log file. Increments the index until a file that doesn't exist is found.
         * @return The file path for a new log file.
         */
        static std::string _getLogFilePath()
        {
            int index = 0;
            std::string path = "";
            do
            {
                path = "/usd/log-" + std::to_string(index) + ".txt";
                index++;
            } while (access(path.c_str(), F_OK) != -1);
            return path;
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
            okapi::Logger::getDefaultLogger()->info([=]()
                                                    { return std::string(message); });
            if (LOG_TO_DISPLAY)
                sendToLCD(message);
        }

        /**
         * Logs a warning message.
         * @param message The message to log.
         */
        static void warn(std::string message)
        {
            okapi::Logger::getDefaultLogger()->warn([=]()
                                                    { return std::string(message); });
            if (LOG_TO_DISPLAY)
                sendToLCD(message);
        }

        /**
         * Logs an error message.
         * @param message The message to log.
         */
        static void error(std::string message)
        {
            okapi::Logger::getDefaultLogger()->error([=]()
                                                     { return std::string(message); });
            if (LOG_TO_DISPLAY)
                sendToLCD(message);
        }

        /**
         * Logs a debug message.
         * @param message The message to log.
         */
        static void debug(std::string message)
        {
            okapi::Logger::getDefaultLogger()->debug([=]()
                                                     { return std::string(message); });
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
        inline static const bool LOG_TO_DISPLAY = false;
        inline static const bool LOG_TO_FILE = false;
    };
}