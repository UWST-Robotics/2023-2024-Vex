#pragma once
#include "devils/utils/logger.h"
#include "okapi/impl/util/timeUtilFactory.hpp"

devils::Logger::Logger() : logger(
                               okapi::TimeUtilFactory::createDefault().getTimer(),
                               LOG_FILE_PATH,
                               okapi::Logger::LogLevel::debug)
{
    info("DevilBots Logger Initialized");
}

okapi::Logger *devils::Logger::getLogger()
{
    return &logger;
}

void devils::Logger::info(std::string message)
{
    logger.info(message);
}

void devils::Logger::warn(std::string message)
{
    logger.warn(message);
}

void devils::Logger::error(std::string message)
{
    logger.error(message);
}

void devils::Logger::debug(std::string message)
{
    logger.debug(message);
}