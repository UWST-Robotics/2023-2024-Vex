#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../control/autoController.hpp"
#include "../geometry/perspective.hpp"
#include "../geometry/units.hpp"
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
            lv_obj_del(redGoalBox);
            lv_obj_del(blueGoalBox);
            lv_obj_del(centerBar);
            lv_obj_del(topBar);
            lv_obj_del(bottomBar);
        }

        void create(lv_obj_t *root) override
        {
            // Bar Style
            static lv_style_t barStyle;
            lv_style_copy(&barStyle, &lv_style_plain);
            barStyle.line.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
            barStyle.line.width = 3 * DisplayUtils::PX_PER_IN;

            // Red Style
            static lv_style_t redStyle;
            lv_style_copy(&redStyle, &lv_style_plain);
            redStyle.line.color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
            redStyle.body.main_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
            redStyle.body.grad_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
            redStyle.body.opa = 128;

            // Blue Style
            static lv_style_t blueStyle;
            lv_style_copy(&blueStyle, &lv_style_plain);
            blueStyle.line.color = LV_COLOR_MAKE(0x00, 0x00, 0xff);
            blueStyle.body.main_color = LV_COLOR_MAKE(0x00, 0x00, 0xff);
            blueStyle.body.grad_color = LV_COLOR_MAKE(0x00, 0x00, 0xff);
            blueStyle.body.opa = 128;

            // Red Goal Box
            redGoalBox = lv_obj_create(root, NULL);
            lv_obj_set_size(
                redGoalBox,
                GOAL_BOX_WIDTH * DisplayUtils::PX_PER_IN,
                GOAL_BOX_HEIGHT * DisplayUtils::PX_PER_IN);
            lv_obj_set_pos(
                redGoalBox,
                GOAL_BOX_X * DisplayUtils::PX_PER_IN + FIELD_OFFSET_X,
                GOAL_BOX_Y * DisplayUtils::PX_PER_IN + FIELD_OFFSET_Y);
            lv_obj_set_style(redGoalBox, &redStyle);

            // Blue Goal Box
            blueGoalBox = lv_obj_create(root, NULL);
            lv_obj_set_size(
                blueGoalBox,
                GOAL_BOX_WIDTH * DisplayUtils::PX_PER_IN,
                GOAL_BOX_HEIGHT * DisplayUtils::PX_PER_IN);
            lv_obj_set_pos(
                blueGoalBox,
                -GOAL_BOX_X * DisplayUtils::PX_PER_IN + FIELD_OFFSET_X - GOAL_BOX_WIDTH * DisplayUtils::PX_PER_IN,
                GOAL_BOX_Y * DisplayUtils::PX_PER_IN + FIELD_OFFSET_Y);
            lv_obj_set_style(blueGoalBox, &blueStyle);

            // Center Bar
            centerBar = lv_line_create(root, NULL);
            static lv_point_t centerBarPoints[2];
            centerBarPoints[0].x = CENTER_BAR_X * DisplayUtils::PX_PER_IN + FIELD_OFFSET_X;
            centerBarPoints[0].y = CENTER_BAR_Y * DisplayUtils::PX_PER_IN + FIELD_OFFSET_Y;
            centerBarPoints[1].x = CENTER_BAR_X * DisplayUtils::PX_PER_IN + FIELD_OFFSET_X;
            centerBarPoints[1].y = -CENTER_BAR_Y * DisplayUtils::PX_PER_IN + FIELD_OFFSET_Y;
            lv_line_set_points(centerBar, centerBarPoints, 2);
            lv_obj_set_style(centerBar, &barStyle);

            // Top Bar
            topBar = lv_line_create(root, NULL);
            static lv_point_t topBarPoints[2];
            topBarPoints[0].x = TOP_BAR_X * DisplayUtils::PX_PER_IN + FIELD_OFFSET_X;
            topBarPoints[0].y = TOP_BAR_Y * DisplayUtils::PX_PER_IN + FIELD_OFFSET_Y;
            topBarPoints[1].x = -TOP_BAR_X * DisplayUtils::PX_PER_IN + FIELD_OFFSET_X;
            topBarPoints[1].y = TOP_BAR_Y * DisplayUtils::PX_PER_IN + FIELD_OFFSET_Y;
            lv_line_set_points(topBar, topBarPoints, 2);
            lv_obj_set_style(topBar, &barStyle);

            // Bottom Bar
            bottomBar = lv_line_create(root, NULL);
            static lv_point_t bottomBarPoints[2];
            bottomBarPoints[0].x = TOP_BAR_X * DisplayUtils::PX_PER_IN + FIELD_OFFSET_X;
            bottomBarPoints[0].y = -TOP_BAR_Y * DisplayUtils::PX_PER_IN + FIELD_OFFSET_Y;
            bottomBarPoints[1].x = -TOP_BAR_X * DisplayUtils::PX_PER_IN + FIELD_OFFSET_X;
            bottomBarPoints[1].y = -TOP_BAR_Y * DisplayUtils::PX_PER_IN + FIELD_OFFSET_Y;
            lv_line_set_points(bottomBar, bottomBarPoints, 2);
            lv_obj_set_style(bottomBar, &barStyle);
        }

    private:
        static constexpr int FIELD_OFFSET_X = DisplayUtils::DISPLAY_WIDTH / 2;
        static constexpr int FIELD_OFFSET_Y = DisplayUtils::DISPLAY_HEIGHT / 2;

        static constexpr int GOAL_BOX_X = -72;
        static constexpr int GOAL_BOX_Y = -24;
        static constexpr int GOAL_BOX_WIDTH = 24;
        static constexpr int GOAL_BOX_HEIGHT = 48;

        static constexpr int CENTER_BAR_X = 0;
        static constexpr int CENTER_BAR_Y = -48;

        static constexpr int TOP_BAR_X = -24;
        static constexpr int TOP_BAR_Y = -48;

        lv_obj_t *redGoalBox;
        lv_obj_t *blueGoalBox;
        lv_obj_t *centerBar;
        lv_obj_t *topBar;
        lv_obj_t *bottomBar;
    };
}