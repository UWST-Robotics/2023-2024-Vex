#pragma once
#include <vector>
#include "../utils/grid.hpp"

namespace devils
{
    /**
     * A struct representing a grid of cells and whether or not each cell is occupied
     */
    struct OccupancyGrid : public Grid<bool>
    {
        /**
         * Gets whether or not a cell is occupied
         * @param x - X position in cells
         * @param y - Y position in cells
         * @returns True if the cell is occupied, false otherwise
         */
        bool getOccupied(int x, int y)
        {
            auto val = this->getCell(x, y);
            if (val == nullptr)
                return true;
            return *val;
        }
    };
}