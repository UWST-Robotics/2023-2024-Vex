#pragma once
#include "pathFile.hpp"
#include "../utils/logger.hpp"
#include "../utils/stringUtils.hpp"
#include "../utils/units.hpp"
#include <fstream>
#include <iostream>

namespace devils
{
    /**
     * Reads a path file from the SD card.
     */
    class PathFileReader
    {
    public:
        /**
         * Checks if the SD card is inserted.
         * @return True if the SD card is inserted.
         */
        static bool isSDInserted()
        {
            return pros::usd::is_installed() == 1;
        }

        /**
         * Reads a path file from the SD card.
         * @return The path file.
         */
        static PathFile readFromSD()
        {
            if (!pros::usd::is_installed())
            {
                Logger::warn("PathFileReader: SD card is not installed!");
                return PathFile();
            }

            // Create a new path file
            PathFile pathFile;
            pathFile.version = 1;

            // Read from SD card
            std::ifstream file(PATH_FILE_PATH);
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
                    PathPoint point = _parsePoint(line);
                    pathFile.points.push_back(point);
                }
                if (line.rfind("EVENT") == 0)
                {
                    PathEvent event = _parseEvent(line);
                    pathFile.points.back().events.push_back(event);
                }
                if (line.rfind("REVERSE") == 0)
                {
                    pathFile.points.back().isReversed = true;
                }
            }
            file.close();

            return pathFile;
        }

        /**
         * Parses a point from a line in the path file.
         * @param line The line to parse.
         * @return The parsed point.
         */
        static PathPoint _parsePoint(std::string line)
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
                    point.rotation = Units::radToDeg(std::stof(split[i]));
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

        /**
         * Parses an event from a line in the path file.
         * @param line The line to parse.
         * @return The parsed event.
         */
        static PathEvent _parseEvent(std::string line)
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

    private:
        inline static const std::string PATH_FILE_PATH = "/usd/path.txt";
    };
}