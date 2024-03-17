#pragma once
#include <vector>

namespace devils
{
    /**
     * A struct representing a grid of traversable cells
     */
    template <typename T>
    struct Grid
    {
        /**
         * Initializes an empty 0x0 grid
         */
        Grid() {}

        /**
         * Initializes a grid with a width and height
         * @param width - Width of the grid in cells
         * @param height - Height of the grid in cells
         */
        Grid(int width, int height) : width(width),
                                      height(height),
                                      values(width * height)
        {
        }

        /**
         * Initializes a grid with a default value at every cell
         * @param width - Width of the grid in cells
         * @param height - Height of the grid in cells
         * @param defaultValue - The default value to fill every cell of the grid
         */
        Grid(int width, int height, T defaultValue) : width(width),
                                                      height(height),
                                                      values(width * height)
        {
            for (int i = 0; i < values.size(); i++)
                values[i] = defaultValue;
        }

        /// @brief Width of the grid in cells
        int width = 0;

        /// @brief Height of the grid in cells
        int height = 0;

        /// @brief 2D array of grid cells. Every `width` indices is a new row.
        std::vector<T> values;

        /**
         * Gets the value of a cell at the given position
         * @param x - X position in cells
         * @param y - Y position in cells
         * @returns True if the cell is occupied, false otherwise
         */
        T *getCell(int x, int y)
        {
            // Get the index of the vector
            int index = height * x + y;

            // Default to occupied if oob
            if (index < 0 || index >= occupancy.size())
                return nullptr;

            // Return the occupancy value
            return *values.at(index);
        }
    };
}