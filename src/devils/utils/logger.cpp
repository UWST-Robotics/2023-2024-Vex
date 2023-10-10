#pragma once
#include "devils/utils/logger.h"
#include "okapi/impl/util/timeUtilFactory.hpp"

void devils::Logger::init()
{
    okapi::Logger::setDefaultLogger(
        std::make_shared<okapi::Logger>(
            okapi::TimeUtilFactory::createDefault().getTimer(),
            LOG_TO_FILE ? LOG_FILE_PATH : LOG_TERMINAL,
            okapi::Logger::LogLevel::warn));
}

void devils::Logger::sendToLCD(std::string message)
{
    static int line = 0;
    pros::lcd::set_text(line, message);
    line = (line + 1) % 8;
}

void devils::Logger::info(std::string message)
{
    okapi::Logger::getDefaultLogger()->info(message);
    if (LOG_TO_DISPLAY)
        sendToLCD(message);
}

void devils::Logger::warn(std::string message)
{
    okapi::Logger::getDefaultLogger()->warn(message);
    if (LOG_TO_DISPLAY)
        sendToLCD(message);
}

void devils::Logger::error(std::string message)
{
    okapi::Logger::getDefaultLogger()->error(message);
    if (LOG_TO_DISPLAY)
        sendToLCD(message);
}

void devils::Logger::debug(std::string message)
{
    okapi::Logger::getDefaultLogger()->debug(message);
    if (LOG_TO_DISPLAY)
        sendToLCD(message);
}

std::shared_ptr<okapi::Logger> devils::Logger::getLogger()
{
    return okapi::Logger::getDefaultLogger();
}