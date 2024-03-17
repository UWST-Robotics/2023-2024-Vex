#pragma once
#include <vector>

namespace devils
{
    /**
     * A struct representing a grid of cells and whether or not each cell is occupied
     */
    struct OccupancyGrid
    {
        /// @brief Width of the grid in cells
        int width = 0;

        /// @brief Height of the grid in cells
        int height = 0;

        /// @brief 2D array of grid cells. Every `width` indices is a new row.
        std::vector<bool> occupancy;

        /**
         * Gets whether or not a cell is occupied
         * @param x - X position in cells
         * @param y - Y position in cells
         * @returns True if the cell is occupied, false otherwise
         */
        bool getOccupied(int x, int y)
        {
            // Get the index of the vector
            int index = height * x + y;

            // Default to occupied if oob
            if (index < 0 || index >= occupancy.size())
                return true;

            // Return the occupancy value
            return occupancy.at(index);
        }
    };
}