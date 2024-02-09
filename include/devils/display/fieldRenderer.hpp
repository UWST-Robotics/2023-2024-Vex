#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../utils/units.hpp"
#include "../control/autoController.hpp"
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
        FieldRenderer()
        {
        }

        ~FieldRenderer()
        {
            for (int i = 0; i < FIELD_WIDTH + FIELD_HEIGHT; i++)
                lv_obj_del(fieldLines[i]);
        }

        void create(lv_obj_t *root)
        {
            static lv_style_t lineStyle;
            lv_style_copy(&lineStyle, &lv_style_plain);
            lineStyle.line.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
            lineStyle.line.width = 1;

            static lv_point_t linePoints[FIELD_WIDTH + FIELD_HEIGHT][2];

            // Horizontal Lines
            for (int i = 0; i < FIELD_WIDTH; i++)
            {
                lv_point_t *points = linePoints[i];
                points[0].x = 50;
                points[0].y = 50 + i * TILE_SIZE;
                points[1].x = 50 + FIELD_HEIGHT * TILE_SIZE;
                points[1].y = 50 + i * TILE_SIZE;

                fieldLines[i] = lv_line_create(root, NULL);
                lv_line_set_points(fieldLines[i], points, 2);
                lv_obj_set_style(fieldLines[i], &lineStyle);
            }

            // Vertical Lines
            for (int i = 0; i < FIELD_HEIGHT; i++)
            {
                lv_point_t *points = linePoints[i + FIELD_WIDTH];
                points[0].x = 50 + i * TILE_SIZE;
                points[0].y = 50;
                points[1].x = 50 + i * TILE_SIZE;
                points[1].y = 50 + FIELD_WIDTH * TILE_SIZE;

                fieldLines[i + FIELD_WIDTH] = lv_line_create(root, NULL);
                lv_line_set_points(fieldLines[i + FIELD_WIDTH], points, 2);
                lv_obj_set_style(fieldLines[i + FIELD_WIDTH], &lineStyle);
            }

            // Bounding Rect
            static lv_obj_t *boundingRect = lv_obj_create(root, NULL);
            {
                lv_obj_set_size(boundingRect, FIELD_WIDTH * TILE_SIZE, FIELD_HEIGHT * TILE_SIZE);
                lv_obj_set_pos(boundingRect, 50, 50);

                static lv_style_t rectStyle;
                lv_style_copy(&rectStyle, &lv_style_plain);
                rectStyle.body.empty = 1;
                rectStyle.body.border.width = 2;
                rectStyle.body.border.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
                lv_obj_set_style(boundingRect, &rectStyle);
            }
        }

        void update() {}

    private:
        static constexpr int TILE_SIZE = 24;   // in
        static constexpr int FIELD_WIDTH = 6;  // tiles
        static constexpr int FIELD_HEIGHT = 6; // tiles

        lv_obj_t *fieldLines[FIELD_WIDTH + FIELD_HEIGHT];
    };
}