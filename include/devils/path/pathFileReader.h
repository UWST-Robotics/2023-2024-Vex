#pragma once
#include "pathFile.h"

namespace devils
{
    class PathFileReader
    {
    public:
        /**
         * Reads a path file from the SD card.
         * @return The path file.
         */
        static PathFile ReadFromSD();

    private:
        inline static const std::string PATH_FILE_PATH = "/usd/path.txt";

        /**
         * Parses a point from a line in the path file.
         * @param line The line to parse.
         * @return The parsed point.
         */
        static PathPoint ParsePoint(std::string);

        /**
         * Parses an event from a line in the path file.
         * @param line The line to parse.
         * @return The parsed event.
         */
        static PathEvent ParseEvent(std::string);
    };
}