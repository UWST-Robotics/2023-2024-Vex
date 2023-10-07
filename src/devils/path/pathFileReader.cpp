#pragma once
#include "devils/path/pathFileReader.h"
#include "devils/utils/stringUtils.h"
#include <fstream>
#include <iostream>

devils::PathPoint devils::PathFileReader::ParsePoint(std::string line)
{
    // Split the line into properties
    auto split = StringUtils::split(line, ' ');
    int index = 0;
    PathPoint point;

    // Iterate through each property
    for (int i = 0; i < split.size(); i++)
    {
        // Ignore POINT
        if (split[i].rfind("POINT") == 0)
            continue;
        // Parse Index to Property
        if (index == 0)
            point.x = std::stof(split[i]);
        if (index == 1)
            point.y = std::stof(split[i]);
        if (index == 2)
            point.rotation = std::stof(split[i]);
        if (index == 3)
            point.enterDelta = std::stof(split[i]);
        if (index == 4)
            point.exitDelta = std::stof(split[i]);
        // Increment Index
        index++;
    }

    // Return the point
    return point;
}

devils::PathEvent devils::PathFileReader::ParseEvent(std::string line)
{
    // Split the line into properties
    auto split = StringUtils::split(line, ' ');
    int index = 0;
    PathEvent event;

    // Iterate through each property
    for (int i = 0; i < split.size(); i++)
    {
        // Ignore EVENT
        if (split[i].rfind("EVENT") == 0)
            continue;
        // Parse Index to Property
        if (index == 0)
            event.name = split[i];
        if (index == 1)
            event.params = split[i];
        // Increment Index
        index++;
    }

    // Return the event
    return event;
}

devils::PathFile devils::PathFileReader::ReadFromSD()
{
    // Create a new path file
    PathFile pathFile;
    pathFile.version = 1;

    // Read from SD card
    std::ifstream file("/usd/paths.txt");
    std::string line;

    // Iterate through each line
    while (std::getline(file, line))
    {
        if (line.empty())
            continue;
        if (line.rfind("ENDPATH") == 0)
            break;
        if (line.rfind("PATH 1") == 0)
            continue;
        if (line.rfind("POINT") == 0)
        {
            PathPoint point = ParsePoint(line);
            pathFile.points.push_back(point);
        }
        if (line.rfind("EVENT") == 0)
        {
            PathEvent event = ParseEvent(line);
            pathFile.points.back().events.push_back(event);
        }
    }
    file.close();
}