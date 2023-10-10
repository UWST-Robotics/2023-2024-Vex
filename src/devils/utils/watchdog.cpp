#pragma once
#include "devils/utils/watchdog.h"
#include "logger.h"

void devils::Watchdog::addCheck(std::function<bool()> check)
{
    checks.push_back(check);
}

void devils::Watchdog::update()
{
    for (auto check : checks)
    {
        if (check())
        {
            Logger::error("Watchdog: Robot is unsafe, sending abort signal");
            isAborted = true;
            return;
        }
    }
}

void devils::Watchdog::reset()
{
    isAborted = false;
}

bool devils::Watchdog::shouldAbort()
{
    return isAborted;
}