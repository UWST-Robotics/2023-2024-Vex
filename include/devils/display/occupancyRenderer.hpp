#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../utils/units.hpp"
#include "../path/occupancyGrid.hpp"
#include "displayUtils.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays the occupancy grid.
     */
    class OccupancyRenderer : public Renderer
    {
    public:
        /**
         * Creates a new OccupancyRenderer
         * @param grid The occupancy grid to render
         */
        OccupancyRenderer(OccupancyGrid &grid) : grid(grid)
        {
        }

        ~OccupancyRenderer()
        {
            for (lv_obj_t *obj : cellObjects)
                lv_obj_del(obj);
        }

        void create(lv_obj_t *root) override
        {
            // Calculate Dimensions
            double cellWidth = (FIELD_WIDTH / (double)grid.width) * DisplayUtils::PX_PER_IN;
            double cellHeight = (FIELD_HEIGHT / (double)grid.height) * DisplayUtils::PX_PER_IN;

            // Iterate Through Grid
            for (int x = 0; x < grid.width; x++)
            {
                for (int y = 0; y < grid.height; y++)
                {
                    // Skip unoccupied cells
                    if (!grid.getOccupied(x, y))
                        continue;

                    // Create LVGL Object
                    lv_obj_t *cellObj = lv_obj_create(root, NULL);
                    {
                        // Pos/Size
                        lv_obj_set_size(cellObj, cellWidth, cellHeight);
                        lv_obj_set_pos(
                            cellObj,
                            x * cellWidth + OFFSET_X,
                            y * cellHeight + OFFSET_Y);

                        // Style
                        static lv_style_t rectStyle;
                        lv_style_copy(&rectStyle, &lv_style_plain);
                        rectStyle.body.main_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                        rectStyle.body.grad_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                        rectStyle.body.opa = 50;
                        lv_obj_set_style(cellObj, &rectStyle);

                        // Append to vector
                        cellObjects.push_back(cellObj);
                    }
                }
            }
        }

    private:
        static constexpr int FIELD_WIDTH = 144;  // in
        static constexpr int FIELD_HEIGHT = 144; // in

        static constexpr double FIELD_WIDTH_PX = FIELD_WIDTH * DisplayUtils::PX_PER_IN;
        static constexpr double FIELD_HEIGHT_PX = FIELD_HEIGHT * DisplayUtils::PX_PER_IN;
        static constexpr double OFFSET_X = (DisplayUtils::DISPLAY_WIDTH - FIELD_WIDTH_PX) / 2;
        static constexpr double OFFSET_Y = (DisplayUtils::DISPLAY_HEIGHT - FIELD_HEIGHT_PX) / 2;

        std::vector<lv_point_t> linePointVector;
        lv_point_t *linePoints;

        OccupancyGrid &grid;
        std::vector<lv_obj_t *> cellObjects;
    };
}