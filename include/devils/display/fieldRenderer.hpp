#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../utils/units.hpp"
#include "../control/autoController.hpp"
#include "../utils/perspective.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays the field as a grid.
     */
    class FieldRenderer : public Renderer
    {
    public:
        ~FieldRenderer()
        {
            lv_obj_del(boundingRect);
            for (int i = 0; i < FIELD_WIDTH + FIELD_HEIGHT; i++)
                lv_obj_del(fieldLines[i]);
        }

        void create(lv_obj_t *root) override
        {
            // Line Style
            static lv_style_t lineStyle;
            lv_style_copy(&lineStyle, &lv_style_plain);
            lineStyle.line.color = LV_COLOR_MAKE(0x55, 0x55, 0x55);
            lineStyle.line.width = 1;

            static lv_point_t linePoints[FIELD_WIDTH + FIELD_HEIGHT][2];

            // Horizontal Lines
            for (int i = 1; i < FIELD_WIDTH; i++)
            {
                lv_point_t *points = linePoints[i];
                points[0].x = FIELD_OFFSET_X;
                points[0].y = FIELD_OFFSET_Y + i * TILE_SIZE;
                points[1].x = FIELD_OFFSET_X + FIELD_HEIGHT * TILE_SIZE;
                points[1].y = FIELD_OFFSET_Y + i * TILE_SIZE;

                fieldLines[i] = lv_line_create(root, NULL);
                lv_line_set_points(fieldLines[i], points, 2);
                lv_obj_set_style(fieldLines[i], &lineStyle);
            }

            // Vertical Lines
            for (int i = 1; i < FIELD_HEIGHT; i++)
            {
                lv_point_t *points = linePoints[i + FIELD_WIDTH];
                points[0].x = FIELD_OFFSET_X + i * TILE_SIZE;
                points[0].y = FIELD_OFFSET_Y;
                points[1].x = FIELD_OFFSET_X + i * TILE_SIZE;
                points[1].y = FIELD_OFFSET_Y + FIELD_WIDTH * TILE_SIZE;

                fieldLines[i + FIELD_WIDTH] = lv_line_create(root, NULL);
                lv_line_set_points(fieldLines[i + FIELD_WIDTH], points, 2);
                lv_obj_set_style(fieldLines[i + FIELD_WIDTH], &lineStyle);
            }

            // Bounding Rect
            boundingRect = lv_obj_create(root, NULL);
            {
                lv_obj_set_size(boundingRect, FIELD_WIDTH * TILE_SIZE, FIELD_HEIGHT * TILE_SIZE);
                lv_obj_set_pos(boundingRect, FIELD_OFFSET_X, FIELD_OFFSET_Y);

                static lv_style_t rectStyle;
                lv_style_copy(&rectStyle, &lv_style_plain);
                rectStyle.body.empty = 1;
                rectStyle.body.border.width = 2;
                rectStyle.body.border.color = LV_COLOR_MAKE(0x55, 0x55, 0x55);
                lv_obj_set_style(boundingRect, &rectStyle);
            }
        }

    private:
        static constexpr int FIELD_WIDTH = 6;                          // tiles
        static constexpr int FIELD_HEIGHT = 6;                         // tiles
        static constexpr int TILE_SIZE = 24 * DisplayUtils::PX_PER_IN; // inches

        static constexpr int FIELD_WIDTH_PX = FIELD_WIDTH * TILE_SIZE;
        static constexpr int FIELD_HEIGHT_PX = FIELD_HEIGHT * TILE_SIZE;
        static constexpr int FIELD_OFFSET_X = (DisplayUtils::DISPLAY_WIDTH - FIELD_WIDTH_PX) / 2;
        static constexpr int FIELD_OFFSET_Y = (DisplayUtils::DISPLAY_HEIGHT - FIELD_HEIGHT_PX) / 2;

        lv_obj_t *fieldLines[FIELD_WIDTH + FIELD_HEIGHT];
        lv_obj_t *boundingRect;
    };
}