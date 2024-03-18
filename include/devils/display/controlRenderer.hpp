#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../geometry/units.hpp"
#include "../control/autoController.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays the controller status.
     */
    class ControlRenderer : public Renderer
    {
    public:
        /**
         * Creates a new control renderer with the autonomous controller
         * @param controller The autonomous controller to use.
         */
        ControlRenderer(AutoController *controller, OdomSource *odomSource)
            : controller(controller), odomSource(odomSource)
        {
        }

        /**
         * Called when the renderer is destroyed.
         */
        ~ControlRenderer()
        {
            lv_obj_del(lineObject);
            lv_obj_del(pointObject);
        }

        void create(lv_obj_t *root) override
        {
            lineObject = lv_line_create(root, NULL);
            {
                static lv_style_t lineStyle;
                lv_style_copy(&lineStyle, &lv_style_plain);
                lineStyle.line.color = LV_COLOR_MAKE(0xff, 0x22, 0x22);
                lineStyle.line.width = 3;
                lv_obj_set_style(lineObject, &lineStyle);
            }

            pointObject = lv_obj_create(root, NULL);
            {
                lv_obj_set_size(pointObject, POINT_SIZE, POINT_SIZE);
                lv_obj_set_pos(pointObject, 0, 0);

                static lv_style_t pointStyle;
                lv_style_copy(&pointStyle, &lv_style_plain);
                pointStyle.body.main_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                pointStyle.body.grad_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                pointStyle.body.radius = LV_RADIUS_CIRCLE;
                lv_obj_set_style(pointObject, &pointStyle);
            }
        }

        void update() override
        {
            if (controller->getFinished())
            {
                lv_obj_set_hidden(pointObject, true);
                lv_obj_set_hidden(lineObject, true);
                return;
            }

            // Get Poses
            auto pose = odomSource->getPose();
            auto point = controller->getTargetPose();

            // Abort if no point
            if (point == nullptr)
            {
                lv_obj_set_hidden(pointObject, true);
                lv_obj_set_hidden(lineObject, true);
                return;
            }

            // Show Objects
            lv_obj_set_hidden(pointObject, false);
            lv_obj_set_hidden(lineObject, false);

            // Update Line to Target
            static lv_point_t linePoints[] = {{0, 0}, {0, 0}};
            linePoints[0].x = pose.x * DisplayUtils::PX_PER_IN + DISPLAY_OFFSET_X;
            linePoints[0].y = pose.y * DisplayUtils::PX_PER_IN + DISPLAY_OFFSET_Y;
            linePoints[1].x = point->x * DisplayUtils::PX_PER_IN + DISPLAY_OFFSET_X;
            linePoints[1].y = point->y * DisplayUtils::PX_PER_IN + DISPLAY_OFFSET_Y;
            lv_line_set_points(lineObject, linePoints, 2);

            // Update Target
            lv_obj_set_pos(
                pointObject,
                point->x * DisplayUtils::PX_PER_IN + POINT_OFFSET_X,
                point->y * DisplayUtils::PX_PER_IN + POINT_OFFSET_Y);
        }

    private:
        static constexpr int POINT_SIZE = 6;
        static constexpr int POINT_OFFSET_X = (DisplayUtils::DISPLAY_WIDTH - POINT_SIZE) / 2;
        static constexpr int POINT_OFFSET_Y = (DisplayUtils::DISPLAY_HEIGHT - POINT_SIZE) / 2;
        static constexpr int DISPLAY_OFFSET_X = DisplayUtils::DISPLAY_WIDTH / 2;
        static constexpr int DISPLAY_OFFSET_Y = DisplayUtils::DISPLAY_HEIGHT / 2;

        lv_obj_t *lineObject;
        lv_obj_t *pointObject;
        AutoController *controller;
        OdomSource *odomSource;
    };
}