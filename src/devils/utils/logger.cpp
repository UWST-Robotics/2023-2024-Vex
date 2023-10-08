#pragma once
#include "devils/utils/logger.h"
#include "okapi/impl/util/timeUtilFactory.hpp"

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