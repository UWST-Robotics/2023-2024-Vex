#pragma once
#include "devils/utils/logger.h"
#include "okapi/impl/util/timeUtilFactory.hpp"

void devils::Logger::init()
{
    okapi::Logger::setDefaultLogger(
        std::make_shared<okapi::Logger>(
            okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
            LOG_FILE_PATH,                                      // Output to the PROS terminal
            okapi::Logger::LogLevel::warn                       // Show errors and warnings
            ));
}

void devils::Logger::info(std::string message)
{
    okapi::Logger::getDefaultLogger()->info(message);
}

void devils::Logger::warn(std::string message)
{
    okapi::Logger::getDefaultLogger()->warn(message);
}

void devils::Logger::error(std::string message)
{
    okapi::Logger::getDefaultLogger()->error(message);
}

void devils::Logger::debug(std::string message)
{
    okapi::Logger::getDefaultLogger()->debug(message);
}

std::shared_ptr<okapi::Logger> devils::Logger::getLogger()
{
    return okapi::Logger::getDefaultLogger();
}