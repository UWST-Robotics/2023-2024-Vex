#pragma once
#include "occupancyGrid.hpp"
#include "../utils/logger.hpp"
#include "../utils/stringUtils.hpp"
#include "../utils/units.hpp"
#include "../utils/sdUtils.hpp"
#include <fstream>
#include <iostream>

namespace devils
{
    /**
     * Reads an occupancy file from the SD card.
     */
    class OccupancyFileReader
    {
    public:
        /**
         * Deserializes an occupancy file from the SD card.
         * @return The deserialized occupancy file.
         */
        static OccupancyGrid readFromSD()
        {
            return deserialize(SDUtils::readToString(OCCUPANCY_FILE_PATH));
        }

        /**
         * Deserializes a occupancy file from a string.
         * @param data The data to deserialize.
         * @return The deserialized occupancy file.
         */
        static OccupancyGrid deserialize(std::string data)
        {
            // Create a new occupancy grid
            OccupancyGrid grid;

            // Read from string
            std::string line;
            std::istringstream readStream(data);

            // Iterate through each line
            while (std::getline(readStream, line))
            {
                if (line.empty())
                    continue;
                if (line.rfind("ENDOCCUPANCY") == 0)
                    break;
                if (line.rfind("OCCUPANCY 1") == 0)
                    continue;

                // Update Dimensions
                grid.height = line.length();
                grid.width++;

                for (char v : line)
                    grid.values.push_back(v == '1');
            }

            return grid;
        }

    private:
        inline static const std::string OCCUPANCY_FILE_PATH = "occupancy.txt";
    };
}